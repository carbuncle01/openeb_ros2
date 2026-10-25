#ifndef OPENEB_ROS2__EVT3_RUN_BATCH_HPP_
#define OPENEB_ROS2__EVT3_RUN_BATCH_HPP_
#include "openeb_ros2/histogram_bin_updates.hpp"
#include <limits>
namespace openeb_ros2 {
// One descriptor per contiguous ADDR_X run, or per encoded vector word.
// The original little-endian 16-bit words remain compressed; no CudaEvent list.
struct Evt3Run {
  std::int64_t bin;
  std::uint32_t word_offset,word_count;
  std::uint16_t y,base_x;
  std::uint8_t kind,polarity;
  std::uint16_t padding{0};
};
static_assert(sizeof(Evt3Run)==24,"Unexpected GPU run descriptor layout");
struct Evt3RunBatch {
  HistogramBinMetadata metadata;
  unsigned input_width{0},input_height{0};
  std::size_t raw_bytes{0};
  bool ready{false};
  std::uint64_t accepted_events{0};
  std::vector<Evt3Run> runs;
  std::size_t payload_bytes() const {return raw_bytes+runs.size()*sizeof(Evt3Run);}
  void validate(std::size_t w,std::size_t h,std::size_t bins,bool separate,bool major,std::int64_t bin_us) const {
    if(!ready||!raw_bytes||raw_bytes%2||raw_bytes/2>std::numeric_limits<std::uint32_t>::max()-32u||
       !input_width||!input_height||input_width>2048||input_height>2048)
      throw std::invalid_argument("Invalid grouped EVT3 RAW shape");
    metadata.validate(w,h,bins,separate,major,bin_us);
    if(!metadata.slots.empty())throw std::invalid_argument("Grouped batch cannot contain histogram slots");
    for(const auto& run:runs) {
      if(run.y>=h||!run.word_count||run.word_offset>=raw_bytes/2||run.word_count>raw_bytes/2-run.word_offset||
         run.bin<0||run.bin>metadata.newest_bin||
         (run.kind!=2&&run.kind!=4&&run.kind!=5)||
         (run.kind!=2&&run.word_count!=1)||run.polarity>1)
        throw std::invalid_argument("Invalid grouped EVT3 run descriptor");
    }
  }
};
} // namespace openeb_ros2
#endif
