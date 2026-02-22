#include "sim/sim.h"
#include "tracking/kalman.h"
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
  // Paths
  const std::string out_dir = arg_str(argc, argv, "--out", "out_run");

  // Sim config
  SimConfig scfg;
  scfg.dt = arg_double(argc, argv, "--dt", 0.02);
  scfg.seed = arg_u64(argc, argv, "--seed", 123);
  scfg.steps = arg_int(argc, argv, "--steps", 500);
  scfg.sigma_z = arg_double(argc, argv, "--sigma_z", 2.0);

  // KF baseline config
  KFConfig kcfg;
  kcfg.q = arg_double(argc, argv, "--q", 1.0);
  kcfg.r = arg_double(argc, argv, "--r", 4.0);

  const int do_hash = arg_int(argc, argv, "--hash", 1);

  std::filesystem::create_directories(out_dir);

  CsvWriter truth(out_dir + "/truth.csv");
  CsvWriter meas(out_dir + "/meas.csv");
  CsvWriter est(out_dir + "/est.csv");
  CsvWriter diag(out_dir + "/diag.csv");

  truth.write_line("k,x,y,vx,vy");
  meas.write_line("k,zx,zy,valid");
  est.write_line("k,x,y,vx,vy");
  diag.write_line("k,yx,yy,Sx,Sy,NIS,q,r");

  Sim2D sim(scfg);
  KF2D kf(scfg.dt, kcfg);

  uint64_t h = 0;

  for (int k = 0; k < scfg.steps; ++k) {
    SimOut o = sim.step();

    // tracker step
    KFStepOut ko = kf.step(o.meas.zx, o.meas.zy, o.meas.valid);

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
                    k, ko.est.x, ko.est.y, ko.est.vx, ko.est.vy);
      est.write_line(buf);
      if (do_hash) h ^= fnv1a64(buf, std::strlen(buf));
    }
    {
      char buf[256];
      std::snprintf(buf, sizeof(buf), "%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                    k, ko.yx, ko.yy, ko.Sx, ko.Sy, ko.nis, kf.cfg().q, kf.cfg().r);
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