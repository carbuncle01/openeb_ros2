#ifndef OPENEB_ROS2__EVENT_PALETTE_HPP_
#define OPENEB_ROS2__EVENT_PALETTE_HPP_
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
namespace openeb_ros2 {
// BGR: positive red, negative blue, empty white; ties select positive.
inline std::array<std::uint8_t, 3> red_blue_white(float positive, float negative) {
  positive = std::clamp(positive, 0.0F, 1.0F);
  negative = std::clamp(negative, 0.0F, 1.0F);
  const auto pale = static_cast<std::uint8_t>(std::lround(255.0F * (1.0F - std::max(positive, negative))));
  return positive >= negative ? std::array<std::uint8_t,3>{pale,pale,255} : std::array<std::uint8_t,3>{255,pale,pale};
}
}
#endif
