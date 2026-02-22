#pragma once
#include <cstdint>
#include <string>

struct SimConfig {
  double dt = 0.02;
  uint64_t seed = 123;
  int steps = 500;

  // measurement model
  double sigma_z = 2.0;   // measurement stddev
  double p_detect = 1.0;  // probability measurement is valid

  // clutter model
  double clutter_prob = 0.0;   // probability of clutter replacing measurement
  double clutter_range = 80.0; // uniform clutter range around origin

  // scenario
  std::string scenario = "cv"; // cv | maneuver | high_noise | clutter
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
  int k = 0;
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
  double randu(double a, double b);
};