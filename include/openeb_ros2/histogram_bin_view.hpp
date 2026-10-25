#ifndef OPENEB_ROS2__HISTOGRAM_BIN_VIEW_HPP_
#define OPENEB_ROS2__HISTOGRAM_BIN_VIEW_HPP_
#include "openeb_ros2/histogram_bin_updates.hpp"
#include <cstring>
namespace openeb_ros2 {
// Borrowed CPU ring planes, slot-major then ON/OFF. No pixel payload copy.
// Caller must prevent any histogram mutation through update_bins_direct().
// The backend copies into its own pinned storage BEFORE returning; async H2D
// never refers to these borrowed CPU pointers.
struct HistogramBinView : HistogramBinMetadata {
  std::vector<const float*> planes;
  std::size_t payload_bytes() const {return slots.size()*(separate?2u:1u)*width*height*sizeof(float);}
  void validate(std::size_t w,std::size_t h,std::size_t b,bool s,bool major,
    std::int64_t bin_us) const {
    HistogramBinMetadata::validate(w,h,b,s,major,bin_us);
    if(planes.size()!=slots.size()*(s?2u:1u))throw std::invalid_argument("Invalid histogram bin view planes");
    for(auto p:planes)if(!p)throw std::invalid_argument("Null histogram bin view plane");
  }
  void copy_slot_to(std::size_t index,float* destination) const {
    const std::size_t count=separate?2u:1u;
    if(index>=slots.size()||!destination||planes.size()!=slots.size()*count)
      throw std::invalid_argument("Invalid histogram slot copy");
    for(std::size_t p=0;p<count;++p) {
      const auto source=planes[index*count+p];
      if(!source)throw std::invalid_argument("Null histogram source plane");
      std::memcpy(destination+p*width*height,source,width*height*sizeof(float));
    }
  }
};
} // namespace openeb_ros2
#endif
