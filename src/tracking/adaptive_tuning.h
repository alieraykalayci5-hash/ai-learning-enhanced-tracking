#pragma once
#include <algorithm>
#include <cmath>

struct A1TunerConfig {
  // Expected mean NIS for 2D measurement
  double target_nis = 2.0;

  // EMA of NIS (0..1). Higher => slower adaptation.
  double nis_ema_alpha = 0.97;

  // Main adaptation gain (0..1). Higher => faster but can oscillate.
  // We keep it modest for stability.
  double gain = 0.03;

  // DEAD-BAND around target:
  // If |nis_ema - target| < deadband, keep R unchanged.
  // This prevents harming already well-tuned nominal cases (cv/maneuver).
  double deadband = 0.25;

  // Clamp for measurement noise variance r
  double r_min = 0.2;    // variance
  double r_max = 100.0;  // variance

  // Spike / outlier handling (clutter robustness):
  // If instantaneous NIS is huge, inflate R quickly.
  double spike_nis = 50.0;     // threshold for "outlier-like" innovation
  double spike_gain = 0.20;    // multiplicative bump per spike (e.g., +20%)
  double spike_cap_ratio = 50.0; // cap on nis/target ratio used in spike logic
};

class A1AdaptiveRTuner {
public:
  explicit A1AdaptiveRTuner(const A1TunerConfig& cfg) : cfg_(cfg) {}

  void reset() {
    has_ema_ = false;
    nis_ema_ = 0.0;
  }

  // Update internal NIS EMA and return tuned r (variance).
  // Call only when you have a valid measurement update.
  double step(double nis, double current_r);

  double nis_ema() const { return nis_ema_; }
  bool has_ema() const { return has_ema_; }

private:
  A1TunerConfig cfg_;
  bool has_ema_ = false;
  double nis_ema_ = 0.0;

  static double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
  }
};