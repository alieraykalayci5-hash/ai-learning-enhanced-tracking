#pragma once
#include <algorithm>

// Constant-Velocity (CV) Kalman Filter in 2D
// State: [x, y, vx, vy]
// Measurement: [zx, zy] = [x, y] + noise

struct KFConfig {
  double q = 1.0; // process noise spectral density (acceleration noise)
  double r = 4.0; // measurement noise variance (per-axis)
};

struct KFState {
  double x = 0, y = 0, vx = 0, vy = 0;
};

struct KFDiag {
  // innovation y = z - H x_pred
  double yx = 0, yy = 0;
  // innovation covariance diag (S = HPH^T + R)
  double Sx = 0, Sy = 0;
  // NIS = y^T S^{-1} y  (2D)
  double nis = 0;
};

class KF2D {
public:
  KF2D(double dt, const KFConfig& cfg);

  // step returns diag (innovation + NIS) and updates internal state
  KFDiag step(double zx, double zy, bool has_meas);

  const KFState& state() const { return st_; }
  const KFConfig& cfg() const { return cfg_; }
  KFConfig& cfg_mut() { return cfg_; }

private:
  double dt_;
  KFConfig cfg_;
  KFState st_;

  // full 4x4 covariance
  double P_[4][4]{};

  void predict_();
  void update_(double zx, double zy, KFDiag& d);

  static void mat4_mul(const double A[4][4], const double B[4][4], double C[4][4]);
  static void mat4_transpose(const double A[4][4], double AT[4][4]);
};