# Sigma-Lognormal Studio

Berio 2015 (CompModels Graffiti Tag Strokes) と Berio 2025 (IGS/DSLM) の手法を実装した、
グラフィティタグ軌跡の解析・編集・合成 Web アプリ。

仕様書: [sigma_lognormal_studio_spec.md](../sigma_lognormal_studio_spec.md)

## 機能（v1.0）

- **解析モード**: 画像アップロード → 輪郭抽出 → 優位点検出 → 円弧フィットによる φ 推定 → 鋭さ指標で Δt 推定 → 反復的ターゲット補正 → `TrajectoryPlan` 出力
- **描画モード**: ブラウザキャンバスで時刻付きストロークを記録し、解析
- **対話編集**: 仮想ターゲットをドラッグ → 自動 refit
- **バリエーション生成**: 仮想ターゲット位置・Δt・φ・XYスケールを摂動し N 個の派生を生成
- **可視化**: 入力輪郭・Action plan・生成軌跡・速度プロファイルをレイヤ切替で表示

## クイックスタート

```bash
./start.sh
```

- Backend: http://localhost:8001  (uvicorn + FastAPI)
- Frontend: http://localhost:3001 (Vite dev server; `/api/*` は自動プロキシ)

初回のみ venv と npm install を実行するため数分かかります。

## 手動起動

```bash
# Backend
cd backend
python3 -m venv .venv
.venv/bin/pip install -e .
.venv/bin/uvicorn app.main:app --port 8001 --reload

# Frontend (別ターミナル)
cd frontend
npm install
npm run dev
```

## API

| Endpoint | 役割 |
|---|---|
| `POST /api/analyze` | 画像または座標列から `TrajectoryPlan` を生成 |
| `POST /api/refit` | 編集済み `plan` を再反復 |
| `POST /api/variations` | 摂動バリエーション生成 |
| `GET /health` | ヘルスチェック |

## ディレクトリ

```
sigma-lognormal-studio/
├── backend/
│   ├── app/          FastAPI ルート + Pydantic スキーマ
│   ├── sigma_lognormal/  数学コア (reparam, sl_model, contour, segmentation, arc_fit, sharpness, fitting, variation)
│   └── pyproject.toml
├── frontend/
│   └── src/          React + TS UI (components, store, api)
├── start.sh
└── README.md
```

## 論文との対応

主要な実装モジュールと論文の式番号の対応は仕様書 §14 を参照。

## ライセンス

内部プロトタイプ。論文実装の参考用。
