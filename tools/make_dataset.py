import argparse
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd


def run_cmd(cmd, cwd: Path) -> None:
  r = subprocess.run(cmd, cwd=str(cwd), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
  if r.returncode != 0:
    print(r.stdout)
    raise RuntimeError(f"Command failed: {' '.join(cmd)}")
  # keep output (useful for debugging determinism)
  # print(r.stdout)


def exe_path(root: Path) -> Path:
  exe = root / "build" / "let_track.exe"
  if exe.exists():
    return exe
  exe2 = root / "build" / "let_track"
  if exe2.exists():
    return exe2
  raise FileNotFoundError("Executable not found. Build first: cmake -S . -B build -G Ninja && cmake --build build")


def make_one_run(root: Path, out_dir: Path, scenario: str, seed: int, steps: int,
                 sigma_z: float, p_detect: float, clutter_prob: float, clutter_range: float,
                 q: float, r: float) -> None:
  out_dir.mkdir(parents=True, exist_ok=True)
  cmd = [
    str(exe_path(root)),
    "--scenario", scenario,
    "--seed", str(seed),
    "--steps", str(steps),
    "--sigma_z", str(sigma_z),
    "--p_detect", str(p_detect),
    "--clutter_prob", str(clutter_prob),
    "--clutter_range", str(clutter_range),
    "--q", str(q),
    "--r", str(r),
    "--out", str(out_dir),
    "--hash", "1",
  ]
  run_cmd(cmd, cwd=root)


def load_run(out_dir: Path) -> pd.DataFrame:
  truth = pd.read_csv(out_dir / "truth.csv")
  meas = pd.read_csv(out_dir / "meas.csv")
  est = pd.read_csv(out_dir / "est.csv")
  diag = pd.read_csv(out_dir / "diag.csv")
  meta = pd.read_csv(out_dir / "meta.csv").iloc[0].to_dict()

  df = truth.merge(meas, on="k").merge(est, on="k", suffixes=("_truth", "_est")).merge(diag, on="k")
  for k, v in meta.items():
    df[k] = v
  return df


def build_dataset(df: pd.DataFrame) -> pd.DataFrame:
  # Features (inputs) for learning module
  # - innovation/residual y, S, NIS
  # - speed magnitude (truth) as a proxy for dynamics regime
  # - scenario params
  df = df.copy()
  df["speed"] = np.sqrt(df["vx_truth"] ** 2 + df["vy_truth"] ** 2)

  # Labels for supervised (A2) v2:
  # - measurement variance true = sigma_z^2
  df["r_true"] = (df["sigma_z"].astype(float) ** 2)

  # simple proxy for process noise label: maneuver -> higher
  # (for maneuver scenario, label high after midpoint)
  steps = int(df["steps"].iloc[0])
  if str(df["scenario"].iloc[0]) == "maneuver":
    df["q_true"] = np.where(df["k"] >= (steps // 2), 8.0, 1.5)
  else:
    df["q_true"] = 1.5

  # Keep dataset columns compact but informative
  keep = [
    "k",
    "x_truth", "y_truth", "vx_truth", "vy_truth",
    "zx", "zy", "valid",
    "x_est", "y_est", "vx_est", "vy_est",
    "yx", "yy", "Sx", "Sy", "NIS",
    "q", "r",
    "scenario", "dt", "seed", "steps", "sigma_z", "p_detect", "clutter_prob", "clutter_range",
    "speed",
    "q_true", "r_true",
  ]
  # Some columns may not exist if pandas suffix differs; fix names from merge:
  ren = {}
  for c in list(df.columns):
    if c.endswith("_truth") and c.replace("_truth", "_truth") not in df.columns:
      pass
  # After merge above, truth columns are x,y,vx,vy; est are x,y,vx,vy with suffixes already applied:
  # Actually we used suffixes=("_truth","_est") on truth vs est merge => truth: x_truth..., est: x_est...
  # Ensure that exact naming exists; if not, derive.
  if "x_truth" not in df.columns and "x" in df.columns:
    ren["x"] = "x_truth"
  if "x_est" not in df.columns and "x_est" not in df.columns and "x_est" in df.columns:
    pass
  df = df.rename(columns=ren)

  # Final sanity: filter keep columns that exist
  keep2 = [c for c in keep if c in df.columns]
  return df[keep2]


def split_write(dataset: pd.DataFrame, out_data_dir: Path, seed: int) -> None:
  out_data_dir.mkdir(parents=True, exist_ok=True)

  # deterministic split by (scenario, seed) groups
  grp_keys = dataset[["scenario", "seed"]].drop_duplicates().sort_values(["scenario", "seed"]).values.tolist()
  n = len(grp_keys)
  if n < 3:
    raise RuntimeError("Need at least 3 (scenario,seed) groups for train/val/test split.")

  # 70/15/15 by groups
  n_train = max(1, int(round(0.70 * n)))
  n_val = max(1, int(round(0.15 * n)))
  n_test = max(1, n - n_train - n_val)

  train_keys = set(tuple(x) for x in grp_keys[:n_train])
  val_keys = set(tuple(x) for x in grp_keys[n_train:n_train + n_val])
  test_keys = set(tuple(x) for x in grp_keys[n_train + n_val:])

  def mask(keys):
    return dataset.apply(lambda r: (r["scenario"], int(r["seed"])) in keys, axis=1)

  train = dataset[mask(train_keys)]
  val = dataset[mask(val_keys)]
  test = dataset[mask(test_keys)]

  train.to_csv(out_data_dir / "train.csv", index=False)
  val.to_csv(out_data_dir / "val.csv", index=False)
  test.to_csv(out_data_dir / "test.csv", index=False)

  summary = {
    "groups_total": n,
    "groups_train": len(train_keys),
    "groups_val": len(val_keys),
    "groups_test": len(test_keys),
    "rows_train": int(len(train)),
    "rows_val": int(len(val)),
    "rows_test": int(len(test)),
  }
  (out_data_dir / "dataset_summary.txt").write_text("\n".join([f"{k}={v}" for k, v in summary.items()]) + "\n")


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--root", default=".", help="repo root (default: .)")
  ap.add_argument("--out_data", default="data", help="data output dir (default: data/)")
  ap.add_argument("--runs_dir", default="data/runs", help="where to store sim runs (default: data/runs)")
  ap.add_argument("--steps", type=int, default=600)
  ap.add_argument("--seeds", type=int, default=12, help="number of seeds per scenario")
  ap.add_argument("--base_seed", type=int, default=100)
  ap.add_argument("--q", type=float, default=1.0)
  ap.add_argument("--r", type=float, default=4.0)
  args = ap.parse_args()

  root = Path(args.root).resolve()
  runs_dir = (root / args.runs_dir).resolve()
  out_data_dir = (root / args.out_data).resolve()

  scenarios = [
    ("cv", dict(sigma_z=2.0, p_detect=1.0, clutter_prob=0.0)),
    ("maneuver", dict(sigma_z=2.0, p_detect=1.0, clutter_prob=0.0)),
    ("high_noise", dict(sigma_z=6.0, p_detect=1.0, clutter_prob=0.0)),
    ("clutter", dict(sigma_z=2.0, p_detect=0.9, clutter_prob=0.25)),
  ]

  all_rows = []
  for scenario, cfg in scenarios:
    for i in range(args.seeds):
      seed = args.base_seed + 1000 * i + (hash(scenario) & 0xFF)
      out_dir = runs_dir / f"{scenario}_seed{seed}"
      make_one_run(
        root=root,
        out_dir=out_dir,
        scenario=scenario,
        seed=seed,
        steps=args.steps,
        sigma_z=float(cfg["sigma_z"]),
        p_detect=float(cfg["p_detect"]),
        clutter_prob=float(cfg["clutter_prob"]),
        clutter_range=80.0,
        q=float(args.q),
        r=float(args.r),
      )
      df_run = load_run(out_dir)
      ds = build_dataset(df_run)
      all_rows.append(ds)

  dataset = pd.concat(all_rows, ignore_index=True)
  split_write(dataset, out_data_dir, seed=args.base_seed)

  print(f"Wrote dataset to: {out_data_dir}")
  print(f"  train.csv rows={len(pd.read_csv(out_data_dir / 'train.csv'))}")
  print(f"  val.csv rows={len(pd.read_csv(out_data_dir / 'val.csv'))}")
  print(f"  test.csv rows={len(pd.read_csv(out_data_dir / 'test.csv'))}")


if __name__ == "__main__":
  main()