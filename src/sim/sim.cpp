#include "sim/sim.h"
#include <cmath>

static inline uint64_t splitmix64(uint64_t& x) {
  uint64_t z = (x += 0x9e3779b97f4a7c15ull);
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
  return z ^ (z >> 31);
}

Sim2D::Sim2D(const SimConfig& cfg) : cfg_(cfg), rng_(cfg.seed), k_(0) {}

uint32_t Sim2D::next_u32() {
  // deterministic 32-bit stream derived from splitmix64
  uint64_t z = splitmix64(rng_);
  return static_cast<uint32_t>(z & 0xFFFFFFFFu);
}

double Sim2D::rand01() {
  // [0,1)
  uint32_t u = next_u32();
  return (double)u / (double)0x100000000ull;
}

double Sim2D::randn() {
  // Box-Muller
  double u1 = rand01();
  double u2 = rand01();
  if (u1 < 1e-12) u1 = 1e-12;
  double r = std::sqrt(-2.0 * std::log(u1));
  double th = 2.0 * M_PI * u2;
  return r * std::cos(th);
}

SimOut Sim2D::step() {
  // simple CV truth propagation
  s_.x += s_.vx * cfg_.dt;
  s_.y += s_.vy * cfg_.dt;

  // deterministic mild maneuver after some time
  if (k_ == cfg_.steps / 2) {
    s_.vx *= 0.6;
    s_.vy *= 1.4;
  }

  SimOut out;
  out.truth = s_;

  // measurement with noise
  out.meas.valid = true;
  out.meas.zx = s_.x + cfg_.sigma_z * randn();
  out.meas.zy = s_.y + cfg_.sigma_z * randn();

  ++k_;
  return out;
}