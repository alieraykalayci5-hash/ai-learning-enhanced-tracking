#pragma once
#include <cstdint>

struct KFConfig {
  // baseline (fixed) noises
  double q = 1.0;          // process noise scale
  double r = 4.0;          // measurement noise variance (per-axis)
};

struct KFState {
  // [x, y, vx, vy]
  double x = 0, y = 0, vx = 0, vy = 0;

  // diag covariance only for skeleton (keeps it simple now)
  double Pxx = 10, Pyy = 10, Pvxvx = 10, Pvyvy = 10;
};

struct KFStepOut {
  KFState est;
  // innovation y = z - H x_pred
  double yx = 0, yy = 0;
  // innovation covariance diag for skeleton
  double Sx = 0, Sy = 0;
  // NIS (2D): yx^2/Sx + yy^2/Sy
  double nis = 0;
};

class KF2D {
public:
  KF2D(double dt, const KFConfig& cfg);

  KFStepOut step(double zx, double zy, bool has_meas);

  const KFConfig& cfg() const { return cfg_; }
  KFConfig& cfg_mut() { return cfg_; } // later: adaptive tuning updates this

private:
  double dt_;
  KFConfig cfg_;
  KFState st_;

  void predict();
  void update(double zx, double zy);
};