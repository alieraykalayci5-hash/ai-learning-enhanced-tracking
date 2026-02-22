#!/usr/bin/env bash
set -euo pipefail

ROOT="/c/Users/AliEray/Desktop/Staj-Proje/learning-enhanced-tracking"
cd "$ROOT"

cmake -S . -B build -G Ninja
cmake --build build -j

./build/let_track --seed 123 --steps 200 --out out_smoke --hash 1
./build/let_track --seed 123 --steps 200 --out out_smoke2 --hash 1

echo "[SMOKE] DONE (hashes should match)"