#include "tracking/adaptive_tuning.h"

double A1AdaptiveRTuner::step(double nis, double current_r) {
  // Defensive
  if (!(nis >= 0.0) || !std::isfinite(nis)) nis = 0.0;
  if (!(current_r > 0.0) || !std::isfinite(current_r)) current_r = cfg_.r_min;

  // Latch baseline r from first call (acts like "do no harm" floor)
  if (!has_base_r_) {
    base_r_ = current_r;
    has_base_r_ = true;
  }

  // Spike bump (clutter/outlier robustness): quick upward nudge only
  if (nis > cfg_.spike_nis) {
    current_r = clamp(current_r * (1.0 + std::max(0.0, cfg_.spike_bump)), cfg_.r_min, cfg_.r_max);
  }

  // EMA on NIS
  const double a = clamp(cfg_.nis_ema_alpha, 0.0, 0.9999);
  if (!has_ema_) {
    nis_ema_ = nis;
    has_ema_ = true;
  } else {
    nis_ema_ = a * nis_ema_ + (1.0 - a) * nis;
  }

  const double target = std::max(1e-9, cfg_.target_nis);
  const double activate = std::max(1.0, cfg_.activate_ratio);

  // If we're not strongly inconsistent, keep r at least baseline (no decrease below baseline)
  if (nis_ema_ <= target * activate) {
    return clamp(std::max(current_r, base_r_), cfg_.r_min, cfg_.r_max);
  }

  // Strong mismatch: increase r conservatively
  double ratio = nis_ema_ / target;
  ratio = clamp(ratio, 1.0, 50.0);

  // Multiplicative growth with small exponent
  const double expo = clamp(cfg_.gain, 0.0, 0.25);
  const double mult = std::pow(ratio, expo);

  double new_r = current_r * mult;

  // Never go below baseline r
  new_r = std::max(new_r, base_r_);

  // Clamp
  new_r = clamp(new_r, cfg_.r_min, cfg_.r_max);
  return new_r;
}