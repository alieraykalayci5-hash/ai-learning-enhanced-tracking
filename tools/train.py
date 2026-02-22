"""
train.py (v2)
- A1 online tuning: C++ içinde deploy edilecek (asıl iş src tarafında)
- A2 supervised: residual/features -> (q_scale, r_scale) küçük MLP

Bu dosyayı bir sonraki adımda gerçek training pipeline'a çevireceğiz.
Şimdilik dataset var mı kontrol eden bir sanity-check koyuyoruz.
"""

import argparse
from pathlib import Path
import pandas as pd


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--data_dir", default="data")
  args = ap.parse_args()

  data_dir = Path(args.data_dir)
  train = data_dir / "train.csv"
  val = data_dir / "val.csv"
  test = data_dir / "test.csv"

  for p in [train, val, test]:
    if not p.exists():
      raise SystemExit(f"Missing dataset file: {p}. Run: python tools/make_dataset.py")

  df = pd.read_csv(train)
  print("Dataset OK")
  print("train rows:", len(df))
  print("columns:", list(df.columns)[:15], "...")


if __name__ == "__main__":
  main()