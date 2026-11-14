"""
Image → 輪郭 (polyline) 抽出。

Pipeline:
  1. リサイズ (長辺 max_dim)
  2. CLAHE で L-チャンネル強調
  3. Otsu + adaptive threshold → インクマスク
  4. Zhang-Suen thinning → 1-px スケルトン
  5. 最大連結成分を選び、端点起点のグラフ追跡でポリラインを得る
  6. 弧長等間隔リサンプル
"""
from __future__ import annotations

import math

import cv2
import numpy as np
from skimage.morphology import thin


def decode_image(raw: bytes) -> np.ndarray:
    """PNG/JPEG バイト列を BGR に変換。アルファは白背景合成。"""
    arr = np.frombuffer(raw, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise ValueError("画像のデコードに失敗しました")
    if img.ndim == 2:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if img.shape[2] == 4:
        alpha = img[:, :, 3:4].astype(np.float32) / 255.0
        bgr = img[:, :, :3].astype(np.float32)
        white = np.full_like(bgr, 255.0)
        return (bgr * alpha + white * (1.0 - alpha)).astype(np.uint8)
    return img


def preprocess(img_bgr: np.ndarray, max_dim: int = 1024) -> np.ndarray:
    h, w = img_bgr.shape[:2]
    if max(h, w) > max_dim:
        s = max_dim / max(h, w)
        img_bgr = cv2.resize(img_bgr, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)
    lab = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2LAB)
    L, A, B = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    L = clahe.apply(L)
    return cv2.cvtColor(cv2.merge([L, A, B]), cv2.COLOR_LAB2BGR)


def build_ink_mask(img_bgr: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    _, otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    adapt = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 31, 5,
    )
    mask = cv2.bitwise_or(otsu, adapt)
    # 暗い場所のみ (背景テクスチャ除去)
    mean_val = gray.mean()
    dark = (gray < mean_val * 0.6).astype(np.uint8) * 255
    mask = cv2.bitwise_and(mask, dark)
    # クロージング → オープニング
    kc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    ko = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kc)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, ko)
    return mask


def extract_all_polylines(
    mask: np.ndarray,
    min_arc_px: float = 15.0,
    max_turn_deg_at_branch: float = 120.0,
) -> list[np.ndarray]:
    """
    skeleton グラフを「ストロークが分岐を貫通する」前提で複数の折れ線に分解する。

    アルゴリズム:
      1. Zhang-Suen 細線化
      2. 8近傍隣接グラフを構築
      3. junction (端点 deg=1 / 分岐点 deg≥3) を抽出
      4. 端点起点で「最も直進する次のエッジ」を選びながら walk
         - 分岐で `cos(turn) < cos(max_turn_deg_at_branch)` ならストロークを終了
      5. 残った未訪問エッジ (ループ・分岐間) も同様に walk
      6. 各 polyline を [x, y] float ndarray にし、弧長 < min_arc_px は除外
      7. 弧長降順でソート
    """
    binary = (mask > 127).astype(bool)
    sk = thin(binary)
    ys, xs = np.where(sk)
    if len(ys) == 0:
        return []
    pts_set = set(zip(ys.tolist(), xs.tolist()))
    nb: dict[tuple[int, int], list[tuple[int, int]]] = {}
    for p in pts_set:
        y, x = p
        adj = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                q = (y + dy, x + dx)
                if q in pts_set:
                    adj.append(q)
        nb[p] = adj
    degree = {p: len(a) for p, a in nb.items()}
    cos_thresh = math.cos(math.radians(max_turn_deg_at_branch))
    visited_edges: set[frozenset] = set()

    def walk(start, first_neighbor):
        path = [start, first_neighbor]
        visited_edges.add(frozenset((start, first_neighbor)))
        prev = start
        cur = first_neighbor
        while True:
            unv = [
                q for q in nb[cur] if frozenset((cur, q)) not in visited_edges
            ]
            if not unv:
                break
            if len(unv) == 1:
                nxt = unv[0]
            else:
                cy, cx = cur
                py, px = prev
                dy0, dx0 = cy - py, cx - px
                norm0 = math.hypot(dy0, dx0) + 1e-9
                best = None
                best_dot = -2.0
                for q in unv:
                    qy, qx = q
                    dy1, dx1 = qy - cy, qx - cx
                    norm1 = math.hypot(dy1, dx1) + 1e-9
                    dot = (dy0 * dy1 + dx0 * dx1) / (norm0 * norm1)
                    if dot > best_dot:
                        best_dot, best = dot, q
                if best_dot < cos_thresh:
                    break
                nxt = best
            visited_edges.add(frozenset((cur, nxt)))
            path.append(nxt)
            prev, cur = cur, nxt
        return path

    polylines: list[list[tuple[int, int]]] = []
    # 端点起点 (左上順)
    endpoints = sorted([p for p, d in degree.items() if d == 1])
    for ep in endpoints:
        for q in nb[ep]:
            if frozenset((ep, q)) in visited_edges:
                continue
            polylines.append(walk(ep, q))
    # 残りエッジ (分岐間 / ループ)
    for p in list(pts_set):
        for q in nb[p]:
            if frozenset((p, q)) in visited_edges:
                continue
            polylines.append(walk(p, q))

    result: list[np.ndarray] = []
    for path in polylines:
        if len(path) < 4:
            continue
        coords = np.array([[float(x), float(y)] for (y, x) in path], dtype=np.float32)
        seg = np.linalg.norm(np.diff(coords, axis=0), axis=1)
        arc = float(np.sum(seg))
        if arc >= min_arc_px:
            result.append(coords)
    result.sort(key=lambda c: -float(np.sum(np.linalg.norm(np.diff(c, axis=0), axis=1))))
    return result


def skeleton_longest_path(mask: np.ndarray) -> np.ndarray:
    """
    Zhang-Suen thinning → 最長経路を抽出して [N, 2] (x, y) で返す。
    交差・分岐は「最も直進方向」を採用して単一ポリラインに。
    """
    binary = (mask > 127).astype(bool)
    sk = thin(binary)
    ys, xs = np.where(sk)
    if len(ys) == 0:
        return np.zeros((0, 2), dtype=np.float32)
    pts = set(zip(ys.tolist(), xs.tolist()))
    nb: dict[tuple[int, int], list[tuple[int, int]]] = {}
    for (y, x) in pts:
        adj = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                q = (y + dy, x + dx)
                if q in pts:
                    adj.append(q)
        nb[(y, x)] = adj
    # 端点 (次数=1) から追跡
    endpoints = [p for p, a in nb.items() if len(a) == 1]
    start = min(endpoints, key=lambda p: (p[0], p[1])) if endpoints else min(pts, key=lambda p: (p[0], p[1]))

    # DFS-like trace preferring straight direction
    path = [start]
    visited = {start}
    prev = None
    cur = start
    while True:
        adj_un = [n for n in nb[cur] if n not in visited]
        if not adj_un:
            break
        if prev is None or len(adj_un) == 1:
            nxt = adj_un[0]
        else:
            cy, cx = cur; py, px = prev
            dy0, dx0 = cy - py, cx - px
            best = adj_un[0]; best_dot = -1e9
            for q in adj_un:
                qy, qx = q
                dy1, dx1 = qy - cy, qx - cx
                dot = dy0 * dy1 + dx0 * dx1
                if dot > best_dot:
                    best_dot, best = dot, q
            nxt = best
        path.append(nxt)
        visited.add(nxt)
        prev = cur
        cur = nxt
    return np.array([[float(x), float(y)] for (y, x) in path], dtype=np.float32)


def resample_arclen(poly: np.ndarray, step: float = 1.0) -> np.ndarray:
    """弧長 step 間隔でポリラインをリサンプル。"""
    if len(poly) < 2:
        return poly.copy()
    seg = np.linalg.norm(np.diff(poly, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(cum[-1])
    if total <= step:
        return poly.copy()
    n = max(int(round(total / step)), 2)
    ts = np.linspace(0.0, total, n + 1)
    out = np.zeros((ts.shape[0], 2), dtype=np.float32)
    j = 0
    for i, t in enumerate(ts):
        while j + 1 < len(cum) and cum[j + 1] < t:
            j += 1
        if j + 1 >= len(cum):
            out[i] = poly[-1]
            continue
        span = max(cum[j + 1] - cum[j], 1e-9)
        a = (t - cum[j]) / span
        out[i] = poly[j] + a * (poly[j + 1] - poly[j])
    return out


def smooth_gaussian(poly: np.ndarray, sigma_px: float = 1.0) -> np.ndarray:
    """簡易 1D Gaussian smoothing along the polyline arclength."""
    if len(poly) < 5 or sigma_px <= 0:
        return poly
    from scipy.ndimage import gaussian_filter1d
    return np.stack([
        gaussian_filter1d(poly[:, 0], sigma=sigma_px, mode="nearest"),
        gaussian_filter1d(poly[:, 1], sigma=sigma_px, mode="nearest"),
    ], axis=-1).astype(np.float32)


def chain_polylines(
    polylines: list[np.ndarray],
    max_chain_dist_px: float = 60.0,
    direction_dot_thresh: float = -0.3,
    min_group_arc_px: float = 30.0,
) -> list[np.ndarray]:
    """
    端点の近い polyline を貪欲に連結して、できるだけ少ない数のチェーンを作る
    (一筆書きに近づける)。

    アルゴリズム:
      1. 未使用 polyline の中から「最も左上の端点」を始点として選ぶ
      2. 現チェーンの末尾に最近接する未使用 polyline を探し、
         端点距離 < max_chain_dist_px かつ方向連続性 (内積 > direction_dot_thresh) なら接続
         必要に応じて反転して結合
      3. ジャンプが 1px を超える場合は線形補間ポイントを挿入 (pen-up セグメント)
      4. 接続できる polyline がなくなったら次のチェーンを開始

    出力: list[np.ndarray]  各チェーンは [N, 2] (x, y)
    """
    if len(polylines) <= 1:
        return [p.copy() for p in polylines]
    polys = [p.astype(np.float32) for p in polylines]
    used = [False] * len(polys)

    def endpoint_dir(p: np.ndarray, from_head: bool) -> np.ndarray:
        if len(p) < 2:
            return np.zeros(2, dtype=np.float32)
        if from_head:
            v = p[min(4, len(p) - 1)] - p[0]
        else:
            v = p[-1] - p[max(-5, -len(p))]
        n = float(np.linalg.norm(v)) + 1e-9
        return v / n

    chains: list[np.ndarray] = []

    while True:
        # 未使用の中から左上 (y, x) 最小の端点を持つ polyline を初期チェーンに
        best_i = -1
        best_key = (math.inf, math.inf)
        for i, p in enumerate(polys):
            if used[i]:
                continue
            for ep in (p[0], p[-1]):
                key = (float(ep[1]), float(ep[0]))
                if key < best_key:
                    best_key = key
                    best_i = i
        if best_i < 0:
            break
        # 始点が末尾側なら反転
        cand = polys[best_i]
        if (cand[-1, 1], cand[-1, 0]) < (cand[0, 1], cand[0, 0]):
            cand = cand[::-1].copy()
        chain = cand
        used[best_i] = True

        # 末尾側に貪欲に拡張
        while True:
            tail_pt = chain[-1]
            tail_dir = endpoint_dir(chain, from_head=False)
            best_j = -1
            best_d = max_chain_dist_px
            best_reverse = False
            for j, p in enumerate(polys):
                if used[j]:
                    continue
                for from_head in (True, False):
                    pt = p[0] if from_head else p[-1]
                    d = float(np.linalg.norm(pt - tail_pt))
                    if d >= best_d:
                        continue
                    # 方向連続性チェック (距離が小さければ緩める)
                    in_dir = endpoint_dir(p, from_head=from_head)
                    if not from_head:
                        in_dir = -in_dir
                    dot = float(np.dot(tail_dir, in_dir))
                    weight = min(d / 10.0, 1.0)   # 距離が小さいと角度ペナルティ無視
                    if dot * weight + (1 - weight) < direction_dot_thresh:
                        continue
                    best_d = d
                    best_j = j
                    best_reverse = not from_head
            if best_j < 0:
                break
            piece = polys[best_j][::-1].copy() if best_reverse else polys[best_j].copy()
            if best_d > 1.0:
                # ペンアップ補間
                start = chain[-1]
                end = piece[0]
                n_interp = max(int(best_d / 1.5), 2)
                interp = np.array(
                    [start + (end - start) * (k + 1) / (n_interp + 1) for k in range(n_interp)],
                    dtype=np.float32,
                )
                chain = np.vstack([chain, interp, piece])
            else:
                chain = np.vstack([chain, piece])
            used[best_j] = True

        # 短すぎるチェーンは捨てる
        seg = np.linalg.norm(np.diff(chain, axis=0), axis=1)
        if float(np.sum(seg)) >= min_group_arc_px:
            chains.append(chain)

    return chains


def extract_all_contours(
    raw: bytes,
    max_dim: int = 1024,
    resample_step_px: float = 1.0,
    smooth_sigma_px: float = 1.5,
    min_arc_px: float = 15.0,
    max_groups: int = 20,
    chain_max_dist_px: float = 60.0,
) -> dict:
    """
    画像 → multi-stroke の輪郭群を抽出する。
    内部で polyline を可能な限り連結し、グループ数を最小化する (一筆書き化)。
    """
    img = decode_image(raw)
    img_pre = preprocess(img, max_dim=max_dim)
    mask = build_ink_mask(img_pre)
    polylines = extract_all_polylines(mask, min_arc_px=min_arc_px)
    chains = chain_polylines(polylines, max_chain_dist_px=chain_max_dist_px)
    contours: list[np.ndarray] = []
    for raw_poly in chains[:max_groups]:
        if len(raw_poly) < 4:
            continue
        rs = resample_arclen(raw_poly, step=resample_step_px)
        if len(rs) < 4:
            continue
        sm = smooth_gaussian(rs, sigma_px=smooth_sigma_px)
        contours.append(sm)
    return {
        "image_width": img_pre.shape[1],
        "image_height": img_pre.shape[0],
        "mask": mask,
        "contours": contours,
    }


def extract_contour(
    raw: bytes,
    max_dim: int = 1024,
    resample_step_px: float = 1.0,
    smooth_sigma_px: float = 1.5,
) -> dict:
    img = decode_image(raw)
    img_pre = preprocess(img, max_dim=max_dim)
    mask = build_ink_mask(img_pre)
    poly_raw = skeleton_longest_path(mask)
    if len(poly_raw) < 4:
        return {
            "image_width": img_pre.shape[1],
            "image_height": img_pre.shape[0],
            "mask": mask,
            "contour": np.zeros((0, 2), dtype=np.float32),
            "contour_dense": np.zeros((0, 2), dtype=np.float32),
        }
    poly = resample_arclen(poly_raw, step=resample_step_px)
    poly_smooth = smooth_gaussian(poly, sigma_px=smooth_sigma_px)
    return {
        "image_width": img_pre.shape[1],
        "image_height": img_pre.shape[0],
        "mask": mask,
        "contour": poly_smooth,   # smoothed, uniformly resampled
        "contour_dense": poly_raw,
    }
