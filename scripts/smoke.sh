#!/usr/bin/env bash
set -euo pipefail

ROOT="/c/Users/AliEray/Desktop/Staj-Proje/ai-learning-enhanced-tracking"
cd "$ROOT"

PY="$ROOT/.venv/Scripts/python.exe"
if [[ ! -f "$PY" ]]; then
  echo "[SMOKE] Missing venv python at $PY"
  echo "Create it in PowerShell:"
  echo "  cd C:\\Users\\AliEray\\Desktop\\Staj-Proje\\ai-learning-enhanced-tracking"
  echo "  py -m venv .venv"
  echo "  .\\.venv\\Scripts\\python.exe -m pip install -r tools\\requirements.txt"
  exit 1
fi

cmake -S . -B build -G Ninja
cmake --build build -j

# Baseline eval (small deterministic)
"$PY" tools/eval.py --smoke

# Compute FNV-1a 64 of the JSON report (stable)
"$PY" - <<'PY'
from pathlib import Path

def fnv1a64(data: bytes) -> int:
  h = 1469598103934665603
  for b in data:
    h ^= b
    h = (h * 1099511628211) & 0xFFFFFFFFFFFFFFFF
  return h

p = Path("reports/smoke_baseline_metrics.json")
b = p.read_bytes()
h = fnv1a64(b)
Path("reports/smoke_hash.txt").write_text(f"{h:016x}\n")
print(f"SMOKE_BASELINE_FNV1A64={h:016x}")
PY

CUR="$(cat reports/smoke_hash.txt | tr -d '\r\n')"
EXP_LINE="$(grep -E '^SMOKE_BASELINE_FNV1A64=' scripts/expected.txt || true)"
EXP="${EXP_LINE#SMOKE_BASELINE_FNV1A64=}"

if [[ -z "${EXP:-}" ]]; then
  echo "SMOKE_BASELINE_FNV1A64=$CUR" > scripts/expected.txt
  echo "[SMOKE] BASELINED expected.txt (run again to verify)"
  exit 0
fi

if [[ "$CUR" == "$EXP" ]]; then
  echo "[SMOKE] PASS ($CUR)"
  exit 0
else
  echo "[SMOKE] FAIL"
  echo " expected=$EXP"
  echo " got     =$CUR"
  exit 1
fi