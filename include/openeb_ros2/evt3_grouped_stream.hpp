#ifndef OPENEB_ROS2__EVT3_GROUPED_STREAM_HPP_
#define OPENEB_ROS2__EVT3_GROUPED_STREAM_HPP_
#include "openeb_ros2/evt3_run_scanner.hpp"
namespace openeb_ros2 {
// One private worker owns this object. RAW input remains caller-owned through
// update_grouped_raw(); both RAW and descriptors can be reused after it returns.
class Evt3GroupedStream {
public:
  Evt3GroupedStream(std::size_t w,std::size_t h,std::size_t bins,std::int64_t window,bool separate,bool major,
      std::int64_t tolerance,std::size_t max_bytes,std::size_t max_runs)
    :scanner_(w,h,bins,window,separate,major,tolerance),max_bytes_(max_bytes) {
    if(max_bytes<2||max_bytes%2||!max_runs||max_runs>max_bytes/2)
      throw std::invalid_argument("Invalid grouped packet/run capacity");
    scanner_.set_run_limit(max_runs);scanner_.reserve_runs(max_runs);
  }
  const Evt3RunBatch& ingest(const std::uint8_t* raw,std::size_t size,unsigned w,unsigned h,std::uint64_t sequence) {
    reset_before_=false;geometry_changed_=false;
    try {
      if(!size||size>max_bytes_)throw std::length_error("Grouped RAW packet exceeds configured capacity");
      if(w!=width_||h!=height_) {
        scanner_.set_geometry(w,h);width_=w;height_=h;has_sequence_=false;geometry_changed_=true;
      } else if(has_sequence_&&sequence!=sequence_+1)scanner_.reset_stream();
      sequence_=sequence;has_sequence_=true;
      scanner_.begin_batch(size);scanner_.feed(raw,size,0);
      const auto& batch=scanner_.batch();
      reset_before_=batch.metadata.epoch!=last_epoch_;last_epoch_=batch.metadata.epoch;
      return batch;
    } catch(...) {reset();throw;}
  }
  void reset(){scanner_.reset_stream();has_sequence_=false;last_epoch_=0;reset_before_=true;}
  bool reset_before()const{return reset_before_;}
  bool geometry_changed()const{return geometry_changed_;}
  const Evt3RunScanner::Statistics& statistics()const{return scanner_.statistics();}
private:
  Evt3RunScanner scanner_;std::size_t max_bytes_;unsigned width_{0},height_{0};
  bool has_sequence_{false},reset_before_{false},geometry_changed_{false};std::uint64_t sequence_{0},last_epoch_{0};
};
} // namespace openeb_ros2
#endif
