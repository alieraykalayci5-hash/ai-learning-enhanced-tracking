# AI Learning-Enhanced Tracking

Deterministic 2D target tracking simulation with AI-based adaptive Kalman noise tuning.

This project demonstrates a systems-level implementation of learning-enhanced state estimation, where measurement noise covariance is adapted online using innovation statistics (NIS).

It is designed as an engineering-focused, reproducible autonomy module suitable for defense-oriented tracking systems.

---

## Overview

The system consists of:

- Deterministic 2D simulator
- Linear Kalman Filter (constant velocity model)
- A1 Adaptive Noise Tuning (NIS-based)
- Dataset generation pipeline
- Baseline vs Adaptive evaluation
- Deterministic regression testing (FNV-1a hash)

Architecture:


Simulator → Measurement Generator → Kalman Filter
↓
A1 Adaptive R Tuning (NIS-based)
↓
Improved Robustness & Statistical Consistency


---

## Key Features

### Deterministic Simulation Core
- Fixed timestep integration
- Seeded RNG
- No system-time dependence
- Bit-level reproducibility
- FNV-1a 64-bit regression hashing

### Tracking
- 2D Constant Velocity Kalman Filter
- Configurable Q and R
- Innovation and NIS logging
- CSV diagnostics

### A1 Adaptive Noise Tuning (AI Module)

Adaptive measurement noise covariance tuning using:

- Innovation squared (NIS)
- Exponential moving average
- Mismatch-only activation threshold
- Never-decrease baseline R (safe mode)
- Spike-based inflation for clutter robustness

Design goals:
- No degradation in nominal conditions
- Improved robustness under clutter
- Restored statistical consistency under noise mismatch

---

## Scenarios

The system supports:

- `cv` — Constant velocity
- `maneuver` — Velocity changes
- `high_noise` — Elevated measurement noise
- `clutter` — False measurements / detection drops

---

## Results (Baseline vs A1 Adaptive)

A1 policy:
- Activates only when NIS indicates strong mismatch
- Never reduces baseline R
- Increases R conservatively
- Spike handling for clutter

| Scenario    | Pos RMSE Δ | Vel RMSE Δ | NIS abs error Δ |
|------------|------------|------------|-----------------|
| cv         | 0.0%       | 0.0%       | 0.0%            |
| maneuver   | 0.0%       | 0.0%       | 0.0%            |
| high_noise | -13.6%     | +0.2%      | +99.0%          |
| clutter    | +4.2%      | +2.9%      | +95.5%          |

Observations:

- Nominal scenarios remain unchanged (A1 inactive).
- Clutter robustness improves.
- High-noise statistical consistency is restored.
- Deterministic behavior preserved.

---

## Project Structure


ai-learning-enhanced-tracking/
├── src/
│ ├── sim/
│ ├── tracking/
│ ├── util/
│ └── main.cpp
├── tools/
│ ├── make_dataset.py
│ ├── eval.py
│ └── train.py (future A2)
├── scripts/
│ └── smoke.sh
├── reports/
├── plots/
├── data/
├── CMakeLists.txt
└── README.md


---

## Build

MSYS2 UCRT64 + Ninja:


cd /c/Users/AliEray/Desktop/Staj-Proje/ai-learning-enhanced-tracking
cmake -S . -B build -G Ninja
cmake --build build -j


Executable:


build/let_track.exe


---

## Run Examples

Baseline:


./build/let_track --mode baseline --scenario cv --seed 123 --steps 800 --out out_baseline --hash 1


Adaptive A1:


./build/let_track --mode a1 --scenario clutter --seed 123 --steps 800 --out out_a1 --hash 1


---

## Evaluation


./.venv/Scripts/python.exe tools/eval.py --mode both


Generates:

- `reports/comparison_metrics.json`
- Scenario RMSE plots
- NIS distribution plots

---

## Determinism & Regression Testing

Golden-hash smoke test:


bash scripts/smoke.sh


Ensures:
- Identical metrics
- Identical outputs
- Deterministic pipeline integrity

---

## Engineering Design Principles

- Reproducibility first
- Deterministic regression safety
- Conservative adaptive logic
- Separation of simulation and evaluation
- Measurable statistical validation (NIS consistency)

---

## Limitations

- Linear KF (no EKF/UKF)
- Single-target tracking
- Scalar R adaptation (no full covariance learning)
- No ML-based noise prediction (reserved for A2)

---

## Future Work

- A2: ML-based residual-to-noise scaling (MLP + ONNX)
- Adaptive Q tuning
- Multi-target tracking
- Data association
- CI integration
- Continuous benchmarking

---

## Author

Ali Eray Kalaycı  
Computer Engineering  
Focus: Real-Time Systems, Tracking & Estimation, Autonomous Systems

MIT License