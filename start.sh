#!/usr/bin/env bash
# ==============================================================================
# Sigma-Lognormal Studio — ローカル起動 (Windows / macOS / Linux)
#   Backend  → http://localhost:8001
#   Frontend → http://localhost:3001
# ==============================================================================
set -e
ROOT="$(cd "$(dirname "$0")" && pwd)"
BACKEND="$ROOT/backend"
FRONTEND="$ROOT/frontend"

# ---- venv のレイアウトを OS に応じて解決 ----
# Windows (Git Bash/MSYS) の venv は Scripts/、Unix系は bin/
if [ -d "$BACKEND/.venv/Scripts" ]; then
  VENV_BIN="$BACKEND/.venv/Scripts"
  EXE=".exe"
else
  VENV_BIN="$BACKEND/.venv/bin"
  EXE=""
fi

# python コマンドを検出 (Windows は python、Unix は python3 が一般的)
if command -v python3 >/dev/null 2>&1; then
  PYTHON=python3
elif command -v python >/dev/null 2>&1; then
  PYTHON=python
else
  echo "ERROR: python / python3 が見つかりません" >&2
  exit 1
fi

# npm コマンド (Windows では npm.cmd の場合あり)
if command -v npm >/dev/null 2>&1; then
  NPM=npm
elif command -v npm.cmd >/dev/null 2>&1; then
  NPM=npm.cmd
else
  echo "ERROR: npm が見つかりません。Node.js をインストールしてください。" >&2
  exit 1
fi

# ---- Backend venv ----
if [ ! -d "$BACKEND/.venv" ]; then
  echo ">>> backend venv を作成中..."
  "$PYTHON" -m venv "$BACKEND/.venv"
  # 作り直したのでパスを再評価
  if [ -d "$BACKEND/.venv/Scripts" ]; then
    VENV_BIN="$BACKEND/.venv/Scripts"
    EXE=".exe"
  else
    VENV_BIN="$BACKEND/.venv/bin"
    EXE=""
  fi
fi
echo ">>> backend 依存をインストール..."
"$VENV_BIN/pip$EXE" install --quiet --upgrade pip
"$VENV_BIN/pip$EXE" install --quiet -e "$BACKEND" httpx

# ---- Frontend deps ----
if [ ! -d "$FRONTEND/node_modules" ]; then
  echo ">>> frontend npm install..."
  (cd "$FRONTEND" && "$NPM" install --silent)
fi

echo ""
echo ">>> Backend: http://localhost:8001"
echo ">>> Frontend: http://localhost:3001"
echo ""

# バックエンド起動
cd "$BACKEND"
"$VENV_BIN/uvicorn$EXE" app.main:app --host 0.0.0.0 --port 8001 --reload &
BK_PID=$!

# フロントエンド起動
cd "$FRONTEND"
"$NPM" run dev &
FE_PID=$!

trap "kill $BK_PID $FE_PID 2>/dev/null; exit" INT TERM
wait
