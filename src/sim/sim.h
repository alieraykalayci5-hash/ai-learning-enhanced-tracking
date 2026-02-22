#pragma once
#include <cstdint>

struct SimConfig {
  double dt = 0.02;
  uint64_t seed = 123;
  int steps = 500;
  double sigma_z = 2.0; // measurement stddev
};

struct TruthState {
  double x = 0, y = 0, vx = 1, vy = 0.5;
};

struct Meas2 {
  double zx = 0, zy = 0;
  bool valid = true;
};

struct SimOut {
  TruthState truth;
  Meas2 meas;
};

class Sim2D {
public:
  explicit Sim2D(const SimConfig& cfg);
  SimOut step();

private:
  SimConfig cfg_;
  TruthState s_;
  uint64_t rng_;
  int k_ = 0;

  uint32_t next_u32();
  double rand01();
  double randn(); // Box-Muller
};