#include "tracking/kalman.h"
#include <algorithm>

KF2D::KF2D(double dt, const KFConfig& cfg) : dt_(dt), cfg_(cfg) {}

void KF2D::predict() {
  // x = x + vx*dt, vx constant
  st_.x += st_.vx * dt_;
  st_.y += st_.vy * dt_;

  // crude diagonal P predict
  // Pxx += dt^2 * Pvxvx + q
  const double dt2 = dt_ * dt_;
  st_.Pxx += dt2 * st_.Pvxvx + cfg_.q;
  st_.Pyy += dt2 * st_.Pvyvy + cfg_.q;

  // velocity uncertainty increases slightly with q too
  st_.Pvxvx += cfg_.q * 0.1;
  st_.Pvyvy += cfg_.q * 0.1;
}

void KF2D::update(double zx, double zy) {
  // H picks x,y so S = Pxx + r, Pyy + r
  const double Sx = st_.Pxx + cfg_.r;
  const double Sy = st_.Pyy + cfg_.r;

  // Kalman gains (diag)
  const double Kx = st_.Pxx / Sx;
  const double Ky = st_.Pyy / Sy;

  const double yx = zx - st_.x;
  const double yy = zy - st_.y;

  // state update (position only in skeleton)
  st_.x += Kx * yx;
  st_.y += Ky * yy;

  // covariance update
  st_.Pxx = (1.0 - Kx) * st_.Pxx;
  st_.Pyy = (1.0 - Ky) * st_.Pyy;

  // NOTE: vx/vy not updated in skeleton yet; we’ll extend soon.
}

KFStepOut KF2D::step(double zx, double zy, bool has_meas) {
  predict();

  KFStepOut out;
  out.est = st_;

  // innovation + NIS computed pre-update (standard)
  out.yx = zx - st_.x;
  out.yy = zy - st_.y;
  out.Sx = st_.Pxx + cfg_.r;
  out.Sy = st_.Pyy + cfg_.r;
  out.nis = (out.yx * out.yx) / std::max(1e-12, out.Sx)
          + (out.yy * out.yy) / std::max(1e-12, out.Sy);

  if (has_meas) {
    update(zx, zy);
  }

  out.est = st_;
  return out;
}