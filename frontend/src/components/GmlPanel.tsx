import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import type { GmlDocument, GmlIndexEntry, MpcResult, MpcStrokeResult } from "../api/types";
import * as api from "../api/client";
import { useStudio } from "../store/studioStore";
import { GmlPreview } from "./GmlPreview";

const PEN_UP_GAP_S = 0.3;

/** Chain per-stroke durations into a global timeline. */
function buildGmlTimeline(doc: GmlDocument): {
  starts: number[];
  durations: number[];
  total: number;
} {
  const starts: number[] = [];
  const durations: number[] = [];
  let acc = 0;
  for (let i = 0; i < doc.strokes.length; i++) {
    const pts = doc.strokes[i].points;
    const dur =
      pts.length > 1 ? Math.max(pts[pts.length - 1][2] - pts[0][2], 0) : 0;
    starts.push(acc);
    durations.push(dur);
    acc += dur + (i < doc.strokes.length - 1 ? PEN_UP_GAP_S : 0);
  }
  return { starts, durations, total: acc };
}

type SubMode = "corpus" | "upload";
type Model = "SL_STAR" | "GL_STAR";

const SAMPLE_SIZE = 100;

export function GmlPanel() {
  const status = useStudio((s) => s.status);
  const gmlDoc = useStudio((s) => s.gmlDoc);
  const gmlSelected = useStudio((s) => s.gmlSelected);
  const setGmlDoc = useStudio((s) => s.setGmlDoc);
  const setGmlSelected = useStudio((s) => s.setGmlSelected);
  const analyzeGmlId = useStudio((s) => s.analyzeGmlId);
  const analyzeGmlFile = useStudio((s) => s.analyzeGmlFile);
  const mpcResult = useStudio((s) => s.mpcResult);
  const mpcLoading = useStudio((s) => s.mpcLoading);
  const runMpc = useStudio((s) => s.runMpc);
  const clearMpc = useStudio((s) => s.clearMpc);

  const [subMode, setSubMode] = useState<SubMode>("corpus");
  const [model, setModel] = useState<Model>("GL_STAR");

  // MPC params
  const [mpcNVia, setMpcNVia] = useState(5);
  const [mpcOrder, setMpcOrder] = useState(4);
  const [mpcAutoOptimize, setMpcAutoOptimize] = useState(true);
  const [mpcTolPx, setMpcTolPx] = useState(3.0);
  const [mpcMinVia, setMpcMinVia] = useState(3);
  const [mpcMaxVia, setMpcMaxVia] = useState(15);
  const [mpcError, setMpcError] = useState<string | null>(null);

  // corpus (random sampling)
  const [query, setQuery] = useState("");
  const [items, setItems] = useState<GmlIndexEntry[]>([]);
  const [total, setTotal] = useState(0);
  const [listLoading, setListLoading] = useState(false);
  const [listError, setListError] = useState<string | null>(null);
  const [activeId, setActiveId] = useState<number | null>(null);

  // upload
  const [uploadedFile, setUploadedFile] = useState<File | null>(null);
  const [uploadError, setUploadError] = useState<string | null>(null);
  const fileRef = useRef<HTMLInputElement>(null);

  const didInitialLoad = useRef(false);
  const lastLoadedQuery = useRef<string | null>(null);

  const loadSample = useCallback(async (q: string) => {
    setListLoading(true);
    setListError(null);
    lastLoadedQuery.current = q;
    try {
      const resp = await api.fetchGmlList(0, SAMPLE_SIZE, q || undefined, {
        random: true,
      });
      setItems(resp.items);
      setTotal(resp.total);
    } catch (e: any) {
      setListError(e?.response?.data?.detail ?? e?.message ?? String(e));
    } finally {
      setListLoading(false);
      didInitialLoad.current = true;
    }
  }, []);

  useEffect(() => {
    if (subMode !== "corpus") return;
    if (didInitialLoad.current) return;
    void loadSample("");
  }, [subMode, loadSample]);

  useEffect(() => {
    if (subMode !== "corpus") return;
    if (!didInitialLoad.current) return;
    if (lastLoadedQuery.current === query) return;
    const handle = setTimeout(() => {
      void loadSample(query);
    }, 300);
    return () => clearTimeout(handle);
  }, [query, subMode, loadSample]);

  const onSelectCorpus = async (entry: GmlIndexEntry) => {
    setActiveId(entry.gml_id);
    setUploadedFile(null);
    clearMpc();
    try {
      const doc = await api.fetchGmlDoc(entry.gml_id);
      setGmlDoc(doc);
    } catch (e: any) {
      setListError(e?.response?.data?.detail ?? e?.message ?? String(e));
      setGmlDoc(null);
    }
  };

  const onFilePicked = async (f: File) => {
    setUploadError(null);
    setActiveId(null);
    setUploadedFile(f);
    clearMpc();
    try {
      const doc = await api.previewGmlFile(f);
      setGmlDoc(doc);
    } catch (e: any) {
      setUploadError(e?.response?.data?.detail ?? e?.message ?? String(e));
      setGmlDoc(null);
    }
  };

  const toggleStroke = (idx: number) => {
    if (!gmlDoc) return;
    clearMpc();
    const cur = gmlSelected ?? gmlDoc.strokes.map((s) => s.index);
    if (cur.includes(idx)) {
      const next = cur.filter((i) => i !== idx);
      setGmlSelected(next.length === gmlDoc.strokes.length ? null : next);
    } else {
      const next = [...cur, idx].sort((a, b) => a - b);
      setGmlSelected(
        next.length === gmlDoc.strokes.length ? null : next,
      );
    }
  };

  const selectAll = () => { clearMpc(); setGmlSelected(null); };
  const selectNone = () => { clearMpc(); setGmlSelected([]); };

  const onAnalyze = async () => {
    if (!gmlDoc) return;
    const selectedForRequest = gmlSelected;
    if (uploadedFile) {
      await analyzeGmlFile(uploadedFile, selectedForRequest, model);
    } else if (activeId != null) {
      await analyzeGmlId(activeId, selectedForRequest, model);
    }
  };

  const onRunMpc = async () => {
    if (!gmlDoc) return;
    setMpcError(null);
    try {
      const source: api.GmlSource = uploadedFile
        ? { file: uploadedFile }
        : { id: activeId! };
      await runMpc(source, gmlSelected, {
        n_via: mpcNVia,
        order: mpcOrder,
        auto_optimize: mpcAutoOptimize,
        tol_px: mpcTolPx,
        min_via: mpcMinVia,
        max_via: mpcMaxVia,
      });
    } catch (e: any) {
      setMpcError(e?.response?.data?.detail ?? e?.message ?? String(e));
    }
  };

  const selCount = gmlDoc
    ? (gmlSelected === null ? gmlDoc.strokes.length : gmlSelected.length)
    : 0;
  const canAnalyze =
    status !== "loading" && gmlDoc !== null && selCount > 0 &&
    (uploadedFile !== null || activeId !== null);
  const canMpc =
    !mpcLoading && gmlDoc !== null && selCount > 0 &&
    (uploadedFile !== null || activeId !== null);

  return (
    <div className="p-4 bg-white rounded-lg shadow-sm space-y-3">
      <h2 className="font-semibold text-sm text-slate-700">GML 入力</h2>

      {/* サブモード切替 */}
      <div className="flex gap-1 bg-slate-100 rounded p-1 text-xs">
        <button
          className={`flex-1 py-1 rounded ${subMode === "corpus" ? "bg-slate-900 text-white" : ""}`}
          onClick={() => setSubMode("corpus")}
        >
          コーパス
        </button>
        <button
          className={`flex-1 py-1 rounded ${subMode === "upload" ? "bg-slate-900 text-white" : ""}`}
          onClick={() => setSubMode("upload")}
        >
          アップロード
        </button>
      </div>

      {subMode === "corpus" && (
        <div className="space-y-2">
          <input
            type="text"
            value={query}
            onChange={(e) => setQuery(e.target.value)}
            placeholder="client 名 or gml_id で検索"
            className="w-full border rounded px-2 py-1 text-sm"
          />
          {listError && (
            <div className="text-xs text-red-600">{listError}</div>
          )}
          <div className="border rounded max-h-48 overflow-y-auto text-xs">
            {listLoading && (
              <div className="p-2 text-slate-500">読み込み中...</div>
            )}
            {!listLoading && items.length === 0 && (
              <div className="p-2 text-slate-500">該当なし</div>
            )}
            {items.map((it) => (
              <button
                key={it.gml_id}
                onClick={() => void onSelectCorpus(it)}
                className={`block w-full text-left px-2 py-1 border-b hover:bg-slate-50 ${
                  activeId === it.gml_id ? "bg-indigo-50" : ""
                }`}
              >
                <div className="flex justify-between">
                  <span className="font-mono">{it.gml_id}</span>
                  <span className="text-slate-500">
                    {it.n_strokes}本 / {it.total_duration_s.toFixed(1)}s
                  </span>
                </div>
                <div className="text-slate-600 truncate">
                  {it.client_name ?? "(unknown)"}
                </div>
              </button>
            ))}
          </div>
          <div className="flex items-center justify-between text-xs text-slate-600">
            <span>
              {total > 0
                ? `ランダム ${items.length} / ${total}`
                : "—"}
            </span>
            <button
              disabled={listLoading || total === 0}
              onClick={() => void loadSample(query)}
              className="px-2 py-0.5 border rounded hover:bg-slate-50 disabled:opacity-40"
              title="再抽選"
            >
              🎲 再抽選
            </button>
          </div>
        </div>
      )}

      {subMode === "upload" && (
        <div className="space-y-2">
          <input
            ref={fileRef}
            type="file"
            accept=".gml,application/xml,text/xml"
            className="hidden"
            onChange={(e) => e.target.files?.[0] && void onFilePicked(e.target.files[0])}
          />
          <div
            onDrop={(e) => {
              e.preventDefault();
              const f = e.dataTransfer.files[0];
              if (f) void onFilePicked(f);
            }}
            onDragOver={(e) => e.preventDefault()}
            onClick={() => fileRef.current?.click()}
            className="border-2 border-dashed border-slate-300 rounded-lg p-4 text-center cursor-pointer hover:bg-slate-50 text-xs text-slate-500"
          >
            {uploadedFile ? uploadedFile.name : ".gml をドロップ / クリック選択"}
          </div>
          {uploadError && (
            <div className="text-xs text-red-600">{uploadError}</div>
          )}
        </div>
      )}

      {/* プレビュー + ストローク選択 */}
      {gmlDoc && (
        <GmlDocPreview
          doc={gmlDoc}
          selected={gmlSelected}
          mpcResult={mpcResult}
        />
      )}
      {gmlDoc && (
        <div className="space-y-1">
          <div className="flex items-center justify-between text-xs text-slate-600">
            <span>
              選択中 {selCount} / {gmlDoc.strokes.length} 本
            </span>
            <div className="flex gap-1">
              <button
                onClick={selectAll}
                className="px-2 py-0.5 border rounded hover:bg-slate-50"
              >
                全選択
              </button>
              <button
                onClick={selectNone}
                className="px-2 py-0.5 border rounded hover:bg-slate-50"
              >
                解除
              </button>
            </div>
          </div>
          <div className="border rounded max-h-28 overflow-y-auto text-xs">
            {gmlDoc.strokes.map((s) => {
              const checked =
                gmlSelected === null ? true : gmlSelected.includes(s.index);
              return (
                <label
                  key={s.index}
                  className="flex items-center gap-2 px-2 py-0.5 border-b cursor-pointer hover:bg-slate-50"
                >
                  <input
                    type="checkbox"
                    checked={checked}
                    onChange={() => toggleStroke(s.index)}
                  />
                  <span className="font-mono">#{s.index}</span>
                  <span className="text-slate-500">{s.n_points}点</span>
                  <span className="text-slate-500 ml-auto">
                    {s.duration_s.toFixed(2)}s
                  </span>
                </label>
              );
            })}
          </div>
        </div>
      )}

      {/* モデル選択 + 解析ボタン */}
      <div className="flex items-center gap-2 text-sm">
        <label>モデル:</label>
        <select
          className="border rounded px-2 py-1"
          value={model}
          onChange={(e) => setModel(e.target.value as Model)}
        >
          <option value="SL_STAR">SL* (円弧)</option>
          <option value="GL_STAR">GL* (Euler spiral)</option>
        </select>
      </div>

      <button
        disabled={!canAnalyze}
        onClick={() => void onAnalyze()}
        className="w-full bg-indigo-600 text-white rounded py-1.5 text-sm disabled:opacity-40"
      >
        {status === "loading" ? "解析中..." : "選択したストロークで解析"}
      </button>

      {/* MPC 解析セクション */}
      <div className="border-t pt-3 space-y-2">
        <div className="flex items-center justify-between">
          <span className="text-xs font-semibold text-slate-600">
            MPC ストローク推定
          </span>
          {mpcResult && (
            <button
              onClick={clearMpc}
              className="text-xs text-slate-400 hover:text-slate-600"
              title="MPC結果をクリア"
            >
              ✕ クリア
            </button>
          )}
        </div>

        {/* 自動最適化トグル */}
        <label className="flex items-center gap-2 text-xs cursor-pointer select-none">
          <input
            type="checkbox"
            checked={mpcAutoOptimize}
            onChange={(e) => setMpcAutoOptimize(e.target.checked)}
            className="rounded"
          />
          <span className="text-slate-700 font-medium">自動最適化</span>
          <span className="text-slate-400">
            (誤差が収束するまで経由点を自動増加)
          </span>
        </label>

        {mpcAutoOptimize ? (
          /* 自動最適化モード設定 */
          <div className="space-y-1.5 text-xs">
            <div className="flex gap-2 items-center">
              <label className="text-slate-600 w-20 shrink-0">誤差しきい値:</label>
              <input
                type="number"
                min={0.5}
                max={20}
                step={0.5}
                value={mpcTolPx}
                onChange={(e) => setMpcTolPx(Number(e.target.value))}
                className="w-16 border rounded px-1 py-0.5"
              />
              <span className="text-slate-400">px</span>
            </div>
            <div className="flex gap-2 items-center">
              <label className="text-slate-600 w-20 shrink-0">経由点 範囲:</label>
              <input
                type="number"
                min={2}
                max={mpcMaxVia}
                value={mpcMinVia}
                onChange={(e) => setMpcMinVia(Number(e.target.value))}
                className="w-12 border rounded px-1 py-0.5"
              />
              <span className="text-slate-400">〜</span>
              <input
                type="number"
                min={mpcMinVia}
                max={30}
                value={mpcMaxVia}
                onChange={(e) => setMpcMaxVia(Number(e.target.value))}
                className="w-12 border rounded px-1 py-0.5"
              />
            </div>
            <div className="flex gap-2 items-center">
              <label className="text-slate-600 w-20 shrink-0">次数:</label>
              <select
                value={mpcOrder}
                onChange={(e) => setMpcOrder(Number(e.target.value))}
                className="border rounded px-1 py-0.5"
              >
                {[2, 3, 4, 5, 6].map((o) => (
                  <option key={o} value={o}>{o}</option>
                ))}
              </select>
            </div>
          </div>
        ) : (
          /* 手動モード設定 */
          <div className="flex gap-2 text-xs items-center">
            <label className="text-slate-600">経由点:</label>
            <input
              type="number"
              min={2}
              max={20}
              value={mpcNVia}
              onChange={(e) => setMpcNVia(Number(e.target.value))}
              className="w-14 border rounded px-1 py-0.5"
            />
            <label className="text-slate-600 ml-1">次数:</label>
            <select
              value={mpcOrder}
              onChange={(e) => setMpcOrder(Number(e.target.value))}
              className="border rounded px-1 py-0.5"
            >
              {[2, 3, 4, 5, 6].map((o) => (
                <option key={o} value={o}>{o}</option>
              ))}
            </select>
          </div>
        )}

        {mpcError && (
          <div className="text-xs text-red-600">{mpcError}</div>
        )}

        <button
          disabled={!canMpc}
          onClick={() => void onRunMpc()}
          className="w-full bg-emerald-600 text-white rounded py-1.5 text-sm disabled:opacity-40"
        >
          {mpcLoading ? "MPC 計算中..." : "MPC で軌跡を推定"}
        </button>

        {/* 結果サマリ */}
        {mpcResult && (
          <div className="space-y-1">
            {mpcResult.strokes.map((s) => (
              <MpcStrokeSummary key={s.stroke_index} stroke={s} tol={mpcResult.tol_px} />
            ))}
          </div>
        )}
      </div>
    </div>
  );
}

function GmlDocPreview({
  doc, selected, mpcResult,
}: {
  doc: GmlDocument;
  selected: number[] | null;
  mpcResult: MpcResult | null;
}) {
  const timeline = useMemo(() => buildGmlTimeline(doc), [doc]);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [currentTime, setCurrentTime] = useState(timeline.total);

  useEffect(() => {
    setPlaying(false);
    setCurrentTime(timeline.total);
  }, [doc, timeline.total]);

  const baseT = useRef(0);
  const startWall = useRef(0);
  useEffect(() => {
    if (!playing) return;
    if (timeline.total <= 0) return;
    baseT.current = currentTime >= timeline.total ? 0 : currentTime;
    startWall.current = performance.now();
    let raf = 0;
    const tick = (now: number) => {
      const elapsed = ((now - startWall.current) / 1000) * speed;
      let t = baseT.current + elapsed;
      if (t >= timeline.total) {
        baseT.current = 0;
        startWall.current = now;
        t = 0;
      }
      setCurrentTime(t);
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [playing, speed, timeline.total]);

  const togglePlay = () => {
    if (timeline.total <= 0) return;
    if (!playing && currentTime >= timeline.total) setCurrentTime(0);
    setPlaying(!playing);
  };

  const reset    = () => { setPlaying(false); setCurrentTime(0); };
  const showFull = () => { setPlaying(false); setCurrentTime(timeline.total); };

  return (
    <div className="space-y-1">
      <div className="text-xs text-slate-600">
        <span className="font-mono">{doc.filename}</span>
        {doc.client_name && ` · ${doc.client_name}`}
        <span className="ml-2 text-slate-400">
          ({doc.screen_bounds[0]}×{doc.screen_bounds[1]})
        </span>
      </div>
      <GmlPreview
        doc={doc}
        selected={selected}
        width={360}
        height={240}
        currentTime={currentTime}
        strokeStarts={timeline.starts}
        mpcResult={mpcResult}
      />
      <div className="flex items-center gap-2 text-xs">
        <button
          onClick={togglePlay}
          disabled={timeline.total <= 0}
          className="px-2 py-0.5 rounded bg-slate-900 text-white disabled:opacity-40"
        >
          {playing ? "⏸" : "▶"}
        </button>
        <button
          onClick={reset}
          className="px-2 py-0.5 border rounded hover:bg-slate-50"
        >
          ⏮
        </button>
        <button
          onClick={showFull}
          className="px-2 py-0.5 border rounded hover:bg-slate-50"
          title="全ストロークを表示"
        >
          ⏭
        </button>
        <input
          type="range"
          min={0}
          max={Math.max(timeline.total, 0.01)}
          step={Math.max(timeline.total / 500, 0.001)}
          value={Math.min(currentTime, timeline.total)}
          onChange={(e) => {
            setPlaying(false);
            setCurrentTime(Number(e.target.value));
          }}
          className="flex-1"
        />
        <select
          value={speed}
          onChange={(e) => setSpeed(Number(e.target.value))}
          className="border rounded px-1"
        >
          <option value={0.25}>0.25×</option>
          <option value={0.5}>0.5×</option>
          <option value={1}>1×</option>
          <option value={2}>2×</option>
          <option value={4}>4×</option>
        </select>
      </div>
      <div className="font-mono text-[10px] text-slate-500 text-right">
        {currentTime.toFixed(2)} / {timeline.total.toFixed(2)} s
      </div>
    </div>
  );
}

// ── MPC 結果サマリ (1ストローク分) ────────────────────────────────────────────

function MpcStrokeSummary({
  stroke, tol,
}: { stroke: MpcStrokeResult; tol: number }) {
  const ok = stroke.converged;
  const hist = stroke.error_history;
  const barMax = hist.length > 0 ? Math.max(...hist.filter(isFinite), 1) : 1;

  return (
    <div className={`rounded px-2 py-1.5 text-xs ${ok ? "bg-emerald-50" : "bg-amber-50"}`}>
      {/* ヘッダー行 */}
      <div className="flex items-center justify-between">
        <span className={`font-medium ${ok ? "text-emerald-700" : "text-amber-700"}`}>
          {ok ? "✓" : "△"} ストローク #{stroke.stroke_index}
        </span>
        <span className={`font-mono ${ok ? "text-emerald-600" : "text-amber-600"}`}>
          {stroke.reconstruction_error_px.toFixed(2)} px
        </span>
      </div>

      {/* 経由点数・次数 */}
      <div className="text-slate-500 mt-0.5">
        経由点 {stroke.n_via} · 次数 {stroke.order}
        {!ok && ` · しきい値 ${tol} px 未達`}
      </div>

      {/* 誤差履歴スパークライン */}
      {hist.length > 1 && (
        <div className="flex items-end gap-px mt-1.5 h-5">
          {hist.map((v, i) => {
            const pct = isFinite(v) ? Math.max(2, Math.round((v / barMax) * 20)) : 20;
            const isLast = i === hist.length - 1;
            return (
              <div
                key={i}
                title={`n_via=${stroke.n_via_start + i}: ${isFinite(v) ? v.toFixed(2) : "—"} px`}
                style={{ height: `${pct}px` }}
                className={`flex-1 rounded-sm ${
                  isLast
                    ? ok ? "bg-emerald-500" : "bg-amber-500"
                    : "bg-slate-300"
                }`}
              />
            );
          })}
          {/* しきい値ライン */}
          <div
            className="absolute border-t border-dashed border-slate-400 w-full pointer-events-none"
            style={{ bottom: `${Math.round((tol / barMax) * 20)}px` }}
          />
        </div>
      )}
    </div>
  );
}
