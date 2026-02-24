# AI Learning-Enhanced Tracking

![C++17](https://img.shields.io/badge/C%2B%2B-17-blue)
![Deterministic](https://img.shields.io/badge/Simulation-Deterministic-success)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow)

Deterministic 2D target tracking simulation with adaptive Kalman noise tuning based on innovation statistics (NIS).

This project demonstrates a systems-level implementation of learning-enhanced state estimation where measurement noise covariance is adapted online under controlled, deterministic rules.

It is designed as an engineering-focused, reproducible tracking module aligned with defense-oriented estimation systems.

---

## Quick Start

Minimal build and run example:

```bash
cmake -S . -B build -G Ninja
cmake --build build -j
./build/let_track --mode baseline --scenario cv --seed 123 --steps 800 --hash 1
```

---

## System Overview

The pipeline consists of:

```text
Simulator
→ Measurement Generator
→ Kalman Filter (CV Model)
→ A1 Adaptive R Tuning (NIS-based)
→ Metrics + CSV Logging
→ FNV-1a Golden Hash
```

Two execution modes are supported:

* **Baseline** – Fixed Q and R
* **A1 Adaptive** – Online measurement noise tuning

All components operate under deterministic constraints.

---

## Deterministic Simulation Core

* Fixed timestep integration
* Seeded RNG (`--seed`)
* No system-time dependence
* Ordered CSV writes
* FNV-1a 64-bit regression hashing

Running identical commands with identical seeds produces identical outputs and hashes.

---

## Motion & Measurement Model

### Motion Model

* 2D Constant Velocity (CV)
* State vector:

```text
[x, y, vx, vy]^T
```

* Discrete linear state transition
* Configurable process noise covariance (Q)

### Measurement Model

* Position-only Gaussian measurements
* Configurable measurement covariance (R)
* Innovation and NIS logging

Diagnostics exported to CSV for analysis.

---

## A1 Adaptive Noise Tuning (Learning Module)

The A1 module adapts measurement noise covariance using innovation statistics.

Mechanism:

* NIS (Normalized Innovation Squared)
* Exponential moving average smoothing
* Mismatch-only activation threshold
* Conservative inflation-only policy (never decreases baseline R)
* Spike-based inflation for clutter robustness

### Design Constraints

* No degradation in nominal conditions
* Improved robustness under clutter
* Restored statistical consistency under noise mismatch
* Deterministic adaptation logic

No stochastic or ML-based inference is used in A1.

---

## Supported Scenarios

| Scenario   | Description                   |
| ---------- | ----------------------------- |
| cv         | Constant velocity             |
| maneuver   | Velocity change events        |
| high_noise | Elevated measurement noise    |
| clutter    | False measurements / dropouts |

---

## Baseline vs Adaptive Results

| Scenario   | Pos RMSE Δ | Vel RMSE Δ | NIS abs error Δ |
| ---------- | ---------- | ---------- | --------------- |
| cv         | 0.0%       | 0.0%       | 0.0%            |
| maneuver   | 0.0%       | 0.0%       | 0.0%            |
| high_noise | -13.6%     | +0.2%      | +99.0%          |
| clutter    | +4.2%      | +2.9%      | +95.5%          |

Observations:

* Nominal scenarios remain unchanged (A1 inactive).
* Clutter robustness improves.
* High-noise statistical consistency is restored.
* Deterministic behavior preserved.

---

## Run Examples

### Baseline

```bash
./build/let_track \
  --mode baseline \
  --scenario cv \
  --seed 123 \
  --steps 800 \
  --out out_baseline \
  --hash 1
```

---

### Adaptive A1

```bash
./build/let_track \
  --mode a1 \
  --scenario clutter \
  --seed 123 \
  --steps 800 \
  --out out_a1 \
  --hash 1
```

---

## Evaluation Pipeline (Python Tools)

```bash
python tools/eval.py --mode both
```

Generates:

* `reports/comparison_metrics.json`
* RMSE plots
* NIS distribution plots

Dataset generation utilities:

* `tools/make_dataset.py`
* `tools/train.py` (reserved for A2)

---

## Determinism & Smoke Test

```bash
bash scripts/smoke.sh
```

Ensures:

* Identical metrics
* Identical output CSVs
* Stable golden hash values

If hashes change, tracking logic or output formatting changed.

---

## Repository Structure

```text
ai-learning-enhanced-tracking/
├── src/
│   ├── sim/
│   ├── tracking/
│   ├── util/
│   └── main.cpp
├── tools/
│   ├── make_dataset.py
│   ├── eval.py
│   └── train.py
├── scripts/
│   └── smoke.sh
├── reports/
├── plots/
├── data/
├── CMakeLists.txt
└── README.md
```

---

## Engineering Highlights

* Deterministic tracking core
* Online adaptive covariance tuning
* Innovation-based statistical validation (NIS)
* Conservative safety-first adaptation policy
* Reproducible benchmarking framework
* Golden-hash regression detection
* Clean CMake + Ninja build system

---

## Technologies

* C++17
* CMake
* Ninja
* Python (evaluation & plotting)
* Custom FNV-1a 64-bit hashing

---

## Limitations

* Linear Kalman Filter (no EKF/UKF)
* Single-target tracking
* Scalar R adaptation only
* No ML-based covariance prediction (reserved for A2)

---

## Future Work

* A2: ML-based residual-to-noise scaling (MLP + ONNX)
* Adaptive Q tuning
* Multi-target tracking + data association
* Continuous benchmarking pipeline
* CI integration with automatic smoke verification

---

## License

MIT

---

## Author

Ali Eray Kalaycı
Computer Engineering
Focus: Real-Time Systems, Tracking & Estimation, Autonomous Systems
