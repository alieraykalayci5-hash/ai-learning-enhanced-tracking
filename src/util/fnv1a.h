#pragma once
#include <cstdint>
#include <cstddef>

static inline uint64_t fnv1a64(const void* data, size_t n) {
  const uint8_t* p = static_cast<const uint8_t*>(data);
  uint64_t h = 1469598103934665603ull; // offset basis
  for (size_t i = 0; i < n; ++i) {
    h ^= uint64_t(p[i]);
    h *= 1099511628211ull; // FNV prime
  }
  return h;
}