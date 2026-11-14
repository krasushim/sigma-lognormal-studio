import { useEffect, useRef } from "react";

interface Props {
  totalDuration: number;
  currentTime: number;
  setCurrentTime: (t: number) => void;
  playing: boolean;
  setPlaying: (b: boolean) => void;
  speed: number;
  setSpeed: (v: number) => void;
}

export function AnimationControls({
  totalDuration, currentTime, setCurrentTime,
  playing, setPlaying, speed, setSpeed,
}: Props) {
  const startRef = useRef<number>(0);
  const baseT = useRef<number>(0);

  useEffect(() => {
    if (!playing) return;
    if (totalDuration <= 0) return;
    // 終端から再開したら最初に戻して再生 (手動シーク後の再生を自然に)
    baseT.current = currentTime >= totalDuration ? 0 : currentTime;
    startRef.current = performance.now();

    let raf = 0;
    const tick = (now: number) => {
      const elapsed = ((now - startRef.current) / 1000) * speed;
      let t = baseT.current + elapsed;
      // ループ: totalDuration を超えたら 0 に巻き戻し (浮動小数点ドリフト対策で baseline もリセット)
      if (t >= totalDuration) {
        baseT.current = t % totalDuration;
        startRef.current = now;
        t = baseT.current;
      }
      setCurrentTime(t);
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [playing, speed, totalDuration]);

  const reset = () => {
    setPlaying(false);
    setCurrentTime(0);
  };

  return (
    <div className="flex items-center gap-3 text-xs px-2 py-2 bg-slate-50 border-t">
      <button
        onClick={() => setPlaying(!playing)}
        className="px-3 py-1 rounded bg-slate-900 text-white"
      >
        {playing ? "⏸ Pause" : "▶ Play"}
      </button>
      <button onClick={reset} className="px-2 py-1 border rounded">
        Reset
      </button>
      <input
        type="range"
        min={0}
        max={totalDuration}
        step={totalDuration / 500}
        value={currentTime}
        onChange={(e) => {
          setPlaying(false);
          setCurrentTime(Number(e.target.value));
        }}
        className="flex-1"
      />
      <span className="font-mono w-24 text-right text-slate-600">
        {currentTime.toFixed(2)} / {totalDuration.toFixed(2)} s
      </span>
      <label className="flex items-center gap-1">
        Speed:
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
      </label>
    </div>
  );
}
