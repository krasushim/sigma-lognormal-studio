import { useRef, useState } from "react";
import { useStudio } from "../store/studioStore";

export function UploadPanel() {
  const fileRef = useRef<HTMLInputElement>(null);
  const [preview, setPreview] = useState<string | null>(null);
  const [model, setModel] = useState<"SL_STAR" | "GL_STAR">("GL_STAR");
  const analyze = useStudio((s) => s.analyzeImage);
  const status = useStudio((s) => s.status);

  const onFile = (f: File) => {
    setPreview(URL.createObjectURL(f));
    analyze(f, model);
  };

  return (
    <div className="p-4 bg-white rounded-lg shadow-sm space-y-3">
      <h2 className="font-semibold text-sm text-slate-700">画像アップロード</h2>
      <input
        ref={fileRef}
        type="file"
        accept="image/png,image/jpeg"
        className="hidden"
        onChange={(e) => e.target.files && onFile(e.target.files[0])}
      />
      <div
        onDrop={(e) => {
          e.preventDefault();
          const f = e.dataTransfer.files[0];
          if (f) onFile(f);
        }}
        onDragOver={(e) => e.preventDefault()}
        onClick={() => fileRef.current?.click()}
        className="border-2 border-dashed border-slate-300 rounded-lg p-6 text-center cursor-pointer hover:bg-slate-50"
      >
        {preview ? (
          <img src={preview} alt="preview" className="max-h-32 mx-auto" />
        ) : (
          <span className="text-sm text-slate-500">
            タグ画像をドロップ / クリック選択
          </span>
        )}
      </div>

      <div className="flex items-center gap-2 text-sm">
        <label>モデル:</label>
        <select
          className="border rounded px-2 py-1"
          value={model}
          onChange={(e) => setModel(e.target.value as any)}
        >
          <option value="SL_STAR">SL* (円弧)</option>
          <option value="GL_STAR">GL* (Euler spiral)</option>
        </select>
      </div>

      <button
        disabled={!preview || status === "loading"}
        onClick={() => fileRef.current?.files?.[0] && analyze(fileRef.current.files[0], model)}
        className="w-full bg-indigo-600 text-white rounded py-1.5 text-sm disabled:opacity-40"
      >
        {status === "loading" ? "解析中..." : "再解析"}
      </button>
    </div>
  );
}
