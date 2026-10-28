#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace flyarm_hw {
using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;  // w,x,y,z throughout
inline Vec3 ned_to_enu(const Vec3 &v) { return {v[1], v[0], -v[2]}; }
inline Vec3 frd_to_flu(const Vec3 &v) { return {v[0], -v[1], -v[2]}; }
template <std::size_t N> bool finite(const std::array<double, N> &v) {
  return std::all_of(v.begin(), v.end(), [](double a) { return std::isfinite(a); });
}
inline Quat multiply(const Quat &a, const Quat &b) {
  return {a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
          a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
          a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
          a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};
}
inline double norm(const Quat &q) {
  return std::sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
}
inline Quat enu_flu(const Quat &ned_frd) {
  const double n = norm(ned_frd);
  if (!finite(ned_frd) || std::abs(n-1.0) > 0.05) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    return {nan,nan,nan,nan};
  }
  const double s = std::sqrt(0.5);
  Quat q = multiply(multiply({0,s,s,0}, ned_frd), {0,1,0,0});
  const double sign = q[0] < 0 ? -1.0 : 1.0;
  for (auto &v : q) v *= sign/n;
  return q;
}
inline std::string topic(const char *base, unsigned version) {
  return std::string("/fmu/out/") + base + (version ? "_v" + std::to_string(version) : "");
}
struct Stats {
  std::uint64_t count=0, regressions=0, repeats=0, zero_stamps=0, stamp=0;
  double first=0, last=0, max_gap_ms=0;
  void add(double receipt, std::uint64_t source_stamp) {
    if (count) {
      max_gap_ms = std::max(max_gap_ms, 1000*(receipt-last));
      regressions += source_stamp < stamp;
      repeats += source_stamp == stamp;
    } else first = receipt;
    ++count;
    zero_stamps += source_stamp == 0;
    last = receipt;
    stamp = source_stamp;
  }
  double hz() const { return count > 1 && last > first ? (count-1)/(last-first) : 0; }
  bool fresh(double now, double threshold) const {
    return count && now >= last && now-last <= threshold;
  }
};
}  // namespace flyarm_hw
