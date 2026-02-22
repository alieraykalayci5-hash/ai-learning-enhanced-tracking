#include "tracking/kalman.h"
#include <cmath>
#include <cstring>

static inline double clamp_pos(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

KF2D::KF2D(double dt, const KFConfig& cfg) : dt_(dt), cfg_(cfg) {
  // Initialize covariance (some uncertainty)
  std::memset(P_, 0, sizeof(P_));
  P_[0][0] = 25.0;
  P_[1][1] = 25.0;
  P_[2][2] = 10.0;
  P_[3][3] = 10.0;
}

void KF2D::mat4_mul(const double A[4][4], const double B[4][4], double C[4][4]) {
  double tmp[4][4]{};
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      double s = 0.0;
      for (int k = 0; k < 4; ++k) s += A[i][k] * B[k][j];
      tmp[i][j] = s;
    }
  }
  std::memcpy(C, tmp, sizeof(tmp));
}

void KF2D::mat4_transpose(const double A[4][4], double AT[4][4]) {
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      AT[j][i] = A[i][j];
}

void KF2D::predict_() {
  // State prediction (CV)
  st_.x += st_.vx * dt_;
  st_.y += st_.vy * dt_;

  // F matrix
  double F[4][4] = {
    {1, 0, dt_, 0},
    {0, 1, 0, dt_},
    {0, 0, 1, 0},
    {0, 0, 0, 1},
  };

  // Process noise Q (discretized accel noise, per axis)
  // q * [ dt^4/4  dt^3/2 ; dt^3/2 dt^2 ] for each axis
  const double dt2 = dt_ * dt_;
  const double dt3 = dt2 * dt_;
  const double dt4 = dt2 * dt2;
  const double q = std::max(1e-12, cfg_.q);

  double Q[4][4]{};
  Q[0][0] = q * (dt4 / 4.0);
  Q[0][2] = q * (dt3 / 2.0);
  Q[2][0] = q * (dt3 / 2.0);
  Q[2][2] = q * (dt2);

  Q[1][1] = q * (dt4 / 4.0);
  Q[1][3] = q * (dt3 / 2.0);
  Q[3][1] = q * (dt3 / 2.0);
  Q[3][3] = q * (dt2);

  // P = F P F^T + Q
  double FP[4][4]{}, FT[4][4]{}, FPFt[4][4]{};
  mat4_mul(F, P_, FP);
  mat4_transpose(F, FT);
  mat4_mul(FP, FT, FPFt);

  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      P_[i][j] = FPFt[i][j] + Q[i][j];
}

void KF2D::update_(double zx, double zy, KFDiag& d) {
  // H picks x,y:
  // y = z - Hx
  d.yx = zx - st_.x;
  d.yy = zy - st_.y;

  const double r = std::max(1e-12, cfg_.r);

  // S = HPH^T + R => diag: Pxx + r, Pyy + r (since H selects state components)
  d.Sx = P_[0][0] + r;
  d.Sy = P_[1][1] + r;

  const double invSx = 1.0 / std::max(1e-12, d.Sx);
  const double invSy = 1.0 / std::max(1e-12, d.Sy);

  d.nis = d.yx * d.yx * invSx + d.yy * d.yy * invSy;

  // Kalman gain K = P H^T S^{-1}
  // Since H^T selects columns 0 and 1, K is 4x2:
  // K[:,0] = P[:,0]/Sx ; K[:,1] = P[:,1]/Sy
  double K0[4], K1[4];
  for (int i = 0; i < 4; ++i) {
    K0[i] = P_[i][0] * invSx;
    K1[i] = P_[i][1] * invSy;
  }

  // State update: x = x + K y
  st_.x  += K0[0] * d.yx + K1[0] * d.yy;
  st_.y  += K0[1] * d.yx + K1[1] * d.yy;
  st_.vx += K0[2] * d.yx + K1[2] * d.yy;
  st_.vy += K0[3] * d.yx + K1[3] * d.yy;

  // Cov update: P = (I - K H) P
  // With H selecting x,y:
  // (I - K H) = I - [K0 K1] * [[1,0,0,0],[0,1,0,0]]
  // => affects columns 0 and 1 only:
  double I_KH[4][4]{};
  for (int i = 0; i < 4; ++i) I_KH[i][i] = 1.0;
  for (int i = 0; i < 4; ++i) {
    I_KH[i][0] -= K0[i];
    I_KH[i][1] -= K1[i];
  }

  double newP[4][4]{};
  mat4_mul(I_KH, P_, newP);
  std::memcpy(P_, newP, sizeof(newP));

  // keep symmetry & non-negative diagonal (numerical hygiene)
  for (int i = 0; i < 4; ++i) {
    P_[i][i] = std::max(1e-12, P_[i][i]);
    for (int j = i + 1; j < 4; ++j) {
      double s = 0.5 * (P_[i][j] + P_[j][i]);
      P_[i][j] = s;
      P_[j][i] = s;
    }
  }
}

KFDiag KF2D::step(double zx, double zy, bool has_meas) {
  predict_();

  KFDiag d{};
  // If no measurement, still compute a stable "diag" based on predicted covariance.
  if (!has_meas) {
    d.yx = 0.0;
    d.yy = 0.0;
    d.Sx = P_[0][0] + std::max(1e-12, cfg_.r);
    d.Sy = P_[1][1] + std::max(1e-12, cfg_.r);
    d.nis = 0.0;
    return d;
  }

  update_(zx, zy, d);
  return d;
}