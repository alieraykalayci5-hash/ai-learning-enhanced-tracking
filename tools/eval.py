import argparse
import json
import subprocess
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def run_cmd(cmd, cwd: Path) -> str:
  r = subprocess.run(cmd, cwd=str(cwd), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
  if r.returncode != 0:
    print(r.stdout)
    raise RuntimeError(f"Command failed: {' '.join(cmd)}")
  return r.stdout


def exe_path(root: Path) -> Path:
  exe = root / "build" / "let_track.exe"
  if exe.exists():
    return exe
  exe2 = root / "build" / "let_track"
  if exe2.exists():
    return exe2
  raise FileNotFoundError("Executable not found. Build first.")


def run_one(root: Path, out_dir: Path, scenario: str, seed: int, steps: int,
            sigma_z: float, p_detect: float, clutter_prob: float,
            q: float, r: float) -> str:
  out_dir.mkdir(parents=True, exist_ok=True)
  cmd = [
    str(exe_path(root)),
    "--scenario", scenario,
    "--seed", str(seed),
    "--steps", str(steps),
    "--sigma_z", str(sigma_z),
    "--p_detect", str(p_detect),
    "--clutter_prob", str(clutter_prob),
    "--clutter_range", "80.0",
    "--q", str(q),
    "--r", str(r),
    "--out", str(out_dir),
    "--hash", "1",
  ]
  return run_cmd(cmd, cwd=root)


def rmse(a: np.ndarray) -> float:
  return float(np.sqrt(np.mean(a * a))) if a.size else float("nan")


def eval_run(out_dir: Path) -> dict:
  truth = pd.read_csv(out_dir / "truth.csv")
  est = pd.read_csv(out_dir / "est.csv")
  diag = pd.read_csv(out_dir / "diag.csv")
  meta = pd.read_csv(out_dir / "meta.csv").iloc[0].to_dict()

  # Align by k
  df = truth.merge(est, on="k", suffixes=("_truth", "_est")).merge(diag, on="k")

  ex = (df["x_est"] - df["x_truth"]).to_numpy()
  ey = (df["y_est"] - df["y_truth"]).to_numpy()
  evx = (df["vx_est"] - df["vx_truth"]).to_numpy()
  evy = (df["vy_est"] - df["vy_truth"]).to_numpy()

  pos_rmse = rmse(np.sqrt(ex * ex + ey * ey))
  vel_rmse = rmse(np.sqrt(evx * evx + evy * evy))

  nis = df["NIS"].to_numpy()
  nis_mean = float(np.mean(nis))
  nis_p50 = float(np.percentile(nis, 50))
  nis_p95 = float(np.percentile(nis, 95))

  # Expected mean NIS ~ measurement dimension (=2) when model/noise consistent
  nis_mean_target = 2.0
  nis_mean_error = float(nis_mean - nis_mean_target)

  out = {
    "meta": meta,
    "metrics": {
      "pos_rmse": pos_rmse,
      "vel_rmse": vel_rmse,
      "nis_mean": nis_mean,
      "nis_p50": nis_p50,
      "nis_p95": nis_p95,
      "nis_mean_error_vs_2": nis_mean_error,
    },
  }
  return out


def plot_rmse_timeseries(out_dir: Path, plot_path: Path) -> None:
  truth = pd.read_csv(out_dir / "truth.csv")
  est = pd.read_csv(out_dir / "est.csv")
  df = truth.merge(est, on="k", suffixes=("_truth", "_est"))
  e = np.sqrt((df["x_est"] - df["x_truth"]) ** 2 + (df["y_est"] - df["y_truth"]) ** 2).to_numpy()

  plt.figure()
  plt.plot(e)
  plt.title("Position Error Over Time")
  plt.xlabel("k")
  plt.ylabel("pos_error")
  plot_path.parent.mkdir(parents=True, exist_ok=True)
  plt.tight_layout()
  plt.savefig(plot_path, dpi=150)
  plt.close()


def plot_nis_hist(out_dir: Path, plot_path: Path) -> None:
  diag = pd.read_csv(out_dir / "diag.csv")
  nis = diag["NIS"].to_numpy()

  plt.figure()
  plt.hist(nis, bins=60)
  plt.title("NIS Histogram")
  plt.xlabel("NIS")
  plt.ylabel("count")
  plot_path.parent.mkdir(parents=True, exist_ok=True)
  plt.tight_layout()
  plt.savefig(plot_path, dpi=150)
  plt.close()


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--root", default=".", help="repo root (default: .)")
  ap.add_argument("--out_root", default="out_eval", help="where scenario runs will be written")
  ap.add_argument("--reports_dir", default="reports")
  ap.add_argument("--plots_dir", default="plots")
  ap.add_argument("--steps", type=int, default=800)
  ap.add_argument("--seed", type=int, default=123)
  ap.add_argument("--q", type=float, default=1.0)
  ap.add_argument("--r", type=float, default=4.0)
  ap.add_argument("--smoke", action="store_true", help="small deterministic eval for smoke test")
  args = ap.parse_args()

  root = Path(args.root).resolve()
  out_root = (root / args.out_root).resolve()
  reports_dir = (root / args.reports_dir).resolve()
  plots_dir = (root / args.plots_dir).resolve()

  if args.smoke:
    args.steps = 250
    args.seed = 123
    out_root = (root / "out_smoke_eval").resolve()

  scenarios = [
    ("cv", dict(sigma_z=2.0, p_detect=1.0, clutter_prob=0.0)),
    ("maneuver", dict(sigma_z=2.0, p_detect=1.0, clutter_prob=0.0)),
    ("high_noise", dict(sigma_z=6.0, p_detect=1.0, clutter_prob=0.0)),
    ("clutter", dict(sigma_z=2.0, p_detect=0.9, clutter_prob=0.25)),
  ]

  report = {
    "type": "baseline",
    "scenarios": {},
  }

  for name, cfg in scenarios:
    out_dir = out_root / f"{name}"
    run_one(root, out_dir, name, args.seed, args.steps,
            sigma_z=float(cfg["sigma_z"]),
            p_detect=float(cfg["p_detect"]),
            clutter_prob=float(cfg["clutter_prob"]),
            q=float(args.q), r=float(args.r))

    rep = eval_run(out_dir)
    report["scenarios"][name] = rep

    # per-scenario plots
    plot_rmse_timeseries(out_dir, plots_dir / f"baseline_{name}_rmse.png")
    plot_nis_hist(out_dir, plots_dir / f"baseline_{name}_nis.png")

  reports_dir.mkdir(parents=True, exist_ok=True)
  report_path = reports_dir / ("smoke_baseline_metrics.json" if args.smoke else "baseline_metrics.json")
  report_path.write_text(json.dumps(report, indent=2) + "\n")

  # Also write a compact top-level plot (aggregate bar)
  names = list(report["scenarios"].keys())
  pos = [report["scenarios"][n]["metrics"]["pos_rmse"] for n in names]
  vel = [report["scenarios"][n]["metrics"]["vel_rmse"] for n in names]
  plt.figure()
  x = np.arange(len(names))
  plt.bar(x - 0.2, pos, width=0.4, label="pos_rmse")
  plt.bar(x + 0.2, vel, width=0.4, label="vel_rmse")
  plt.xticks(x, names, rotation=20)
  plt.title("Baseline RMSE by Scenario")
  plt.ylabel("rmse")
  plt.legend()
  plt.tight_layout()
  plt.savefig(plots_dir / "baseline_rmse.png", dpi=150)
  plt.close()

  print(f"Wrote report: {report_path}")
  print(f"Wrote plots under: {plots_dir}")


if __name__ == "__main__":
  main()