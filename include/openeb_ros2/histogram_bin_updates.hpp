#ifndef OPENEB_ROS2__HISTOGRAM_BIN_UPDATES_HPP_
#define OPENEB_ROS2__HISTOGRAM_BIN_UPDATES_HPP_
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>
namespace openeb_ros2 {
// Owned, absolute bin replacements; NOT per-event deltas. Values are packed
// slot-major, then ON/OFF plane, then H,W, independent of GPU ring layout.
struct HistogramBinUpdates {
  std::size_t width{0},height{0},bins{0};
  bool separate{true},polarity_major{true};
  std::uint64_t epoch{0};
  std::int64_t origin_us{0},latest_us{0},newest_bin{-1},bin_width_us{0};
  std::vector<std::size_t> slots;
  std::vector<float> values;
  void validate(std::size_t w,std::size_t h,std::size_t b,bool s,bool major,
    std::int64_t bin_us) const {
    if(width!=w || height!=h || bins!=b || separate!=s || polarity_major!=major ||
      bin_width_us!=bin_us || bin_us<=0 || origin_us<0 || latest_us<origin_us ||
      newest_bin<0 || (latest_us-origin_us)/bin_us!=newest_bin ||
      slots.size()>2*b || values.size()!=slots.size()*w*h*(s?2u:1u))
      throw std::invalid_argument("Invalid histogram bin update shape/clock");
    for(std::size_t i=0;i<slots.size();++i)
      if(slots[i]>=2*b || (i && slots[i-1]>=slots[i]))
        throw std::invalid_argument("Histogram slots must be unique and sorted");
  }
};
} // namespace openeb_ros2
#endif
