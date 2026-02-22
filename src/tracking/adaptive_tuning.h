#pragma once
#include <algorithm>
#include <cmath>

struct A1TunerConfig {
  // Expected mean NIS for 2D measurement
  double target_nis = 2.0;

  // EMA on NIS (higher => slower)
  double nis_ema_alpha = 0.98;

  // Main gain for multiplicative growth (small => conservative)
  double gain = 0.02;

  // Activation ratio: only adapt if nis_ema > target * activate_ratio
  // This prevents harming nominal well-tuned cases.
  double activate_ratio = 2.0;

  // Clamp for measurement noise variance r
  double r_min = 0.2;
  double r_max = 100.0;

  // Spike inflate (for clutter/outliers): if instantaneous nis is huge, bump R quickly
  double spike_nis = 50.0;
  double spike_bump = 0.15; // +15% bump
};

class A1AdaptiveRTuner {
public:
  explicit A1AdaptiveRTuner(const A1TunerConfig& cfg) : cfg_(cfg) {}

  void reset() {
    has_ema_ = false;
    nis_ema_ = 0.0;
    has_base_r_ = false;
    base_r_ = 0.0;
  }

  // Conservative rule:
  // - Keep a "base_r" (baseline) from the first call
  // - Never return r < base_r (prevents overconfidence / RMSE regressions)
  // - Only increase r when nis_ema indicates strong mismatch
  double step(double nis, double current_r);

  double nis_ema() const { return nis_ema_; }
  bool has_ema() const { return has_ema_; }
  double base_r() const { return base_r_; }

private:
  A1TunerConfig cfg_;

  bool has_ema_ = false;
  double nis_ema_ = 0.0;

  bool has_base_r_ = false;
  double base_r_ = 0.0;

  static double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
  }
};