#ifndef ZAKERIMANESH_TEMPLATE_CLAMP_HPP
#define ZAKERIMANESH_TEMPLATE_CLAMP_HPP

#include <algorithm>

namespace zakerimanesh {
template <typename T>
inline T templateClamp(const T& v, const T& lo, const T& hi) noexcept {
  return std::max(lo, std::min(v, hi));
}
}  // namespace zakerimanesh

#endif  // ZAKERIMANESH_TEMPLATE_CLAMP_HPP