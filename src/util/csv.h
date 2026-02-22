#pragma once
#include <cstdio>
#include <string>
#include <vector>

struct CsvWriter {
  std::FILE* f = nullptr;

  explicit CsvWriter(const std::string& path) {
    f = std::fopen(path.c_str(), "wb");
  }
  ~CsvWriter() {
    if (f) std::fclose(f);
  }

  CsvWriter(const CsvWriter&) = delete;
  CsvWriter& operator=(const CsvWriter&) = delete;

  bool ok() const { return f != nullptr; }

  void write_line(const std::string& s) {
    if (!f) return;
    std::fwrite(s.data(), 1, s.size(), f);
    std::fwrite("\n", 1, 1, f);
  }
};