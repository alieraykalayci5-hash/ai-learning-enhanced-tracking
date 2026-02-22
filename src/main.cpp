#include "sim/sim.h"
#include "tracking/kalman.h"
#include "tracking/adaptive_tuning.h"
#include "util/csv.h"
#include "util/fnv1a.h"

#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>

static std::string arg_str(int argc, char** argv, const std::string& key, const std::string& def) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == key) return argv[i + 1];
  }
  return def;
}

static int arg_int(int argc, char** argv, const std::string& key, int def) {
  return std::stoi(arg_str(argc, argv, key, std::to_string(def)));
}

static double arg_double(int argc, char** argv, const std::string& key, double def) {
  return std::stod(arg_str(argc, argv, key, std::to_string(def)));
}

static uint64_t arg_u64(int argc, char** argv, const std::string& key, uint64_t def) {
  return static_cast<uint64_t>(std::stoull(arg_str(argc, argv, key, std::to_string(def))));
}

int main(int argc, char** argv) {
  const std::string out_dir = arg_str(argc, argv, "--out", "out_run");

  // Mode
  const std::string mode = arg_str(argc, argv, "--mode", "baseline"); // baseline | a1

  // Sim config
  SimConfig scfg;
  scfg.dt = arg_double(argc, argv, "--dt", 0.02);
  scfg.seed = arg_u64(argc, argv, "--seed", 123);
  scfg.steps = arg_int(argc, argv, "--steps", 500);
  scfg.sigma_z = arg_double(argc, argv, "--sigma_z", 2.0);
  scfg.p_detect = arg_double(argc, argv, "--p_detect", 1.0);
  scfg.clutter_prob = arg_double(argc, argv, "--clutter_prob", 0.0);
  scfg.clutter_range = arg_double(argc, argv, "--clutter_range", 80.0);
  scfg.scenario = arg_str(argc, argv, "--scenario", "cv"); // cv|maneuver|high_noise|clutter

  // KF config
  KFConfig kcfg;
  kcfg.q = arg_double(argc, argv, "--q", 1.0);
  kcfg.r = arg_double(argc, argv, "--r", 4.0);

  // A1 config (safe adaptive R tuning)
  A1TunerConfig a1cfg;
  a1cfg.target_nis = arg_double(argc, argv, "--a1_target_nis", 2.0);
  a1cfg.nis_ema_alpha = arg_double(argc, argv, "--a1_ema", 0.98);
  a1cfg.gain = arg_double(argc, argv, "--a1_gain", 0.02);
  a1cfg.activate_ratio = arg_double(argc, argv, "--a1_activate_ratio", 2.0);
  a1cfg.r_min = arg_double(argc, argv, "--a1_rmin", 0.2);
  a1cfg.r_max = arg_double(argc, argv, "--a1_rmax", 100.0);
  a1cfg.spike_nis = arg_double(argc, argv, "--a1_spike_nis", 50.0);
  a1cfg.spike_bump = arg_double(argc, argv, "--a1_spike_bump", 0.15);

  const int do_hash = arg_int(argc, argv, "--hash", 1);

  std::filesystem::create_directories(out_dir);

  CsvWriter truth(out_dir + "/truth.csv");
  CsvWriter meas(out_dir + "/meas.csv");
  CsvWriter est(out_dir + "/est.csv");
  CsvWriter diag(out_dir + "/diag.csv");
  CsvWriter meta(out_dir + "/meta.csv");

  truth.write_line("k,x,y,vx,vy");
  meas.write_line("k,zx,zy,valid");
  est.write_line("k,x,y,vx,vy");
  diag.write_line("k,yx,yy,Sx,Sy,NIS,q,r,nis_ema,base_r");

  meta.write_line(
    "mode,scenario,dt,seed,steps,sigma_z,p_detect,clutter_prob,clutter_range,q,r,"
    "a1_target_nis,a1_ema,a1_gain,a1_activate_ratio,a1_rmin,a1_rmax,a1_spike_nis,a1_spike_bump"
  );

  {
    char buf[768];
    std::snprintf(
      buf, sizeof(buf),
      "%s,%s,%.6f,%llu,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
      mode.c_str(),
      scfg.scenario.c_str(),
      scfg.dt,
      (unsigned long long)scfg.seed,
      scfg.steps,
      scfg.sigma_z,
      scfg.p_detect,
      scfg.clutter_prob,
      scfg.clutter_range,
      kcfg.q,
      kcfg.r,
      a1cfg.target_nis,
      a1cfg.nis_ema_alpha,
      a1cfg.gain,
      a1cfg.activate_ratio,
      a1cfg.r_min,
      a1cfg.r_max,
      a1cfg.spike_nis,
      a1cfg.spike_bump
    );
    meta.write_line(buf);
  }

  Sim2D sim(scfg);
  KF2D kf(scfg.dt, kcfg);
  A1AdaptiveRTuner tuner(a1cfg);

  uint64_t h = 0;

  for (int k = 0; k < scfg.steps; ++k) {
    SimOut o = sim.step();

    // KF step (uses current cfg_.r)
    KFDiag d = kf.step(o.meas.zx, o.meas.zy, o.meas.valid);

    // A1: update R only after valid measurement update
    if (mode == "a1" && o.meas.valid) {
      double new_r = tuner.step(d.nis, kf.cfg().r);
      kf.cfg_mut().r = new_r;
    }

    const KFState& s = kf.state();
    const double nis_ema = tuner.has_ema() ? tuner.nis_ema() : 0.0;
    const double base_r = tuner.base_r();

    // log
    {
      char buf[256];
      std::snprintf(buf, sizeof(buf), "%d,%.6f,%.6f,%.6f,%.6f",
                    k, o.truth.x, o.truth.y, o.truth.vx, o.truth.vy);
      truth.write_line(buf);
      if (do_hash) h ^= fnv1a64(buf, std::strlen(buf));
    }
    {
      char buf[256];
      std::snprintf(buf, sizeof(buf), "%d,%.6f,%.6f,%d",
                    k, o.meas.zx, o.meas.zy, o.meas.valid ? 1 : 0);
      meas.write_line(buf);
      if (do_hash) h ^= fnv1a64(buf, std::strlen(buf));
    }
    {
      char buf[256];
      std::snprintf(buf, sizeof(buf), "%d,%.6f,%.6f,%.6f,%.6f",
                    k, s.x, s.y, s.vx, s.vy);
      est.write_line(buf);
      if (do_hash) h ^= fnv1a64(buf, std::strlen(buf));
    }
    {
      char buf[384];
      std::snprintf(buf, sizeof(buf), "%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                    k, d.yx, d.yy, d.Sx, d.Sy, d.nis, kf.cfg().q, kf.cfg().r, nis_ema, base_r);
      diag.write_line(buf);
      if (do_hash) h ^= fnv1a64(buf, std::strlen(buf));
    }
  }

  if (do_hash) {
    std::cout << "FNV1A64_XOR=" << std::hex << h << std::dec << "\n";
  }
  std::cout << "Wrote outputs to: " << out_dir << "\n";
  return 0;
}