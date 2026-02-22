#include "tracking/adaptive_tuning.h"

double A1AdaptiveRTuner::step(double nis, double current_r) {
  // Defensive
  if (!(nis >= 0.0) || !std::isfinite(nis)) nis = 0.0;
  if (!(current_r > 0.0) || !std::isfinite(current_r)) current_r = cfg_.r_min;

  const double target = std::max(1e-9, cfg_.target_nis);

  // --- 1) Spike / outlier inflation (instantaneous) ---
  // If a single update has extremely large NIS, we treat it as likely clutter/outlier.
  // Inflate R quickly so next updates don't get yanked by false measurements.
  if (nis > cfg_.spike_nis) {
    double ratio = nis / target;
    ratio = clamp(ratio, 1.0, cfg_.spike_cap_ratio);

    // simple bump: r <- r * (1 + spike_gain), repeated effect from repeated spikes
    double bumped = current_r * (1.0 + cfg_.spike_gain);

    // also allow a slight extra bump with ratio (gentle, bounded)
    // e.g., ratio=50 => factor ~1.0..2.0
    double extra = 1.0 + 0.02 * (ratio - 1.0);
    extra = clamp(extra, 1.0, 2.0);

    current_r = clamp(bumped * extra, cfg_.r_min, cfg_.r_max);
  }

  // --- 2) EMA on NIS (for smooth adaptation) ---
  if (!has_ema_) {
    nis_ema_ = nis;
    has_ema_ = true;
  } else {
    const double a = clamp(cfg_.nis_ema_alpha, 0.0, 0.9999);
    nis_ema_ = a * nis_ema_ + (1.0 - a) * nis;
  }

  // --- 3) Deadband: do nothing if we're already close to target ---
  if (std::abs(nis_ema_ - target) < std::max(0.0, cfg_.deadband)) {
    return clamp(current_r, cfg_.r_min, cfg_.r_max);
  }

  // --- 4) Main multiplicative adaptation using EMA ratio ---
  // ratio > 1 => increase R, ratio < 1 => decrease R
  double ratio = nis_ema_ / target;
  ratio = clamp(ratio, 0.05, 20.0);

  // Tempered exponent update
  const double expo = clamp(cfg_.gain, 0.0, 1.0);
  const double mult = std::pow(ratio, expo);

  double new_r = current_r * mult;
  new_r = clamp(new_r, cfg_.r_min, cfg_.r_max);
  return new_r;
}