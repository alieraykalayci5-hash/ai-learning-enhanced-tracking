#include "tracking/adaptive_tuning.h"

double A1AdaptiveRTuner::step(double nis, double current_r) {
  // Defensive
  if (!(nis >= 0.0) || !std::isfinite(nis)) nis = 0.0;
  if (!(current_r > 0.0) || !std::isfinite(current_r)) current_r = cfg_.r_min;

  // EMA on NIS
  if (!has_ema_) {
    nis_ema_ = nis;
    has_ema_ = true;
  } else {
    nis_ema_ = cfg_.nis_ema_alpha * nis_ema_ + (1.0 - cfg_.nis_ema_alpha) * nis;
  }

  const double target = std::max(1e-9, cfg_.target_nis);

  // If nis_ema > target => we are overconfident => increase R (r).
  // If nis_ema < target => decrease R (r).
  //
  // Use a multiplicative update on r with a tempered ratio.
  // ratio = nis_ema/target ; ratio>1 => increase r ; ratio<1 => decrease r
  double ratio = nis_ema_ / target;
  ratio = clamp(ratio, 0.05, 20.0);

  // Temper the ratio to avoid aggressive jumps.
  // exponent in (0..1): smaller => gentler
  const double expo = clamp(cfg_.gain, 0.0, 1.0);
  const double mult = std::pow(ratio, expo);

  double new_r = current_r * mult;
  new_r = clamp(new_r, cfg_.r_min, cfg_.r_max);
  return new_r;
}