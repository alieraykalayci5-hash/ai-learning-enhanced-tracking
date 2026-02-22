#include "sim/sim.h"
#include <cmath>

static inline uint64_t splitmix64(uint64_t& x) {
  uint64_t z = (x += 0x9e3779b97f4a7c15ull);
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
  return z ^ (z >> 31);
}

Sim2D::Sim2D(const SimConfig& cfg) : cfg_(cfg), rng_(cfg.seed), k_(0) {
  // scenario presets (still overrideable by CLI args in main)
  if (cfg_.scenario == "high_noise") {
    cfg_.sigma_z = std::max(cfg_.sigma_z, 6.0);
  } else if (cfg_.scenario == "clutter") {
    cfg_.clutter_prob = std::max(cfg_.clutter_prob, 0.25);
    cfg_.p_detect = std::min(cfg_.p_detect, 0.9);
  }
}

uint32_t Sim2D::next_u32() {
  uint64_t z = splitmix64(rng_);
  return static_cast<uint32_t>(z & 0xFFFFFFFFu);
}

double Sim2D::rand01() {
  uint32_t u = next_u32();
  return (double)u / (double)0x100000000ull; // [0,1)
}

double Sim2D::randu(double a, double b) {
  return a + (b - a) * rand01();
}

double Sim2D::randn() {
  double u1 = rand01();
  double u2 = rand01();
  if (u1 < 1e-12) u1 = 1e-12;
  double r = std::sqrt(-2.0 * std::log(u1));
  double th = 2.0 * M_PI * u2;
  return r * std::cos(th);
}

SimOut Sim2D::step() {
  // Truth propagation (CV)
  s_.x += s_.vx * cfg_.dt;
  s_.y += s_.vy * cfg_.dt;

  // Maneuver injection (deterministic change)
  if (cfg_.scenario == "maneuver") {
    if (k_ == cfg_.steps / 2) {
      s_.vx *= 0.55;
      s_.vy *= 1.65;
    }
  }

  SimOut out;
  out.k = k_;
  out.truth = s_;

  // Detection
  bool detected = (rand01() < cfg_.p_detect);
  out.meas.valid = detected;

  // Measurement
  if (detected) {
    double zx = s_.x + cfg_.sigma_z * randn();
    double zy = s_.y + cfg_.sigma_z * randn();

    // Clutter replaces measurement with uniform false return
    if (cfg_.clutter_prob > 0.0 && (rand01() < cfg_.clutter_prob)) {
      zx = randu(-cfg_.clutter_range, cfg_.clutter_range);
      zy = randu(-cfg_.clutter_range, cfg_.clutter_range);
    }

    out.meas.zx = zx;
    out.meas.zy = zy;
  } else {
    // keep deterministic values even if invalid
    out.meas.zx = 0.0;
    out.meas.zy = 0.0;
  }

  ++k_;
  return out;
}