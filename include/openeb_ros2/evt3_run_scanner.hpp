// EVT3 clock rules match Evt3RunsHistogram/event_camera_codecs. No pixel histogram.
#ifndef OPENEB_ROS2__EVT3_RUN_SCANNER_HPP_
#define OPENEB_ROS2__EVT3_RUN_SCANNER_HPP_
#include "openeb_ros2/evt3_run_batch.hpp"
#include <algorithm>
namespace openeb_ros2 {
class Evt3RunScanner {
public:
  struct Statistics {std::uint64_t accepted{0},bounds{0},backward{0},dropped{0},resets{0},triggers{0},backward_max_us{0};};
  Evt3RunScanner(std::size_t w,std::size_t h,std::size_t bins,std::int64_t window,bool separate,bool major,
      std::int64_t tolerance=4000):height_(h),tolerance_(tolerance) {
    if(!w||!h||w>65535||h>65535||!bins||bins>1024||window<=0||window%bins||tolerance<0)
      throw std::invalid_argument("Invalid run scanner configuration");
    auto& m=batch_.metadata;m.width=w;m.height=h;m.bins=bins;m.separate=separate;m.polarity_major=major;m.bin_width_us=window/bins;
  }
  void set_geometry(unsigned w,unsigned h) {
    if(!w||!h||w>2048||h>2048)throw std::invalid_argument("Invalid EVT3 geometry");
    batch_.input_width=w;batch_.input_height=h;reset_stream();
  }
  void reset_stream() {
    valid_time_=false;high_=0;low_=0;y_=0;base_=0;polarity_=0;
    batch_.raw_bytes=0;next_offset_=0;clear_epoch();
  }
  void begin_batch(std::size_t raw_bytes) {
    if(!batch_.input_width||!raw_bytes||raw_bytes%2||raw_bytes/2>std::numeric_limits<std::uint32_t>::max()-32u)
      throw std::invalid_argument("Invalid RAW batch size");
    batch_.runs.clear();batch_.accepted_events=0;batch_.raw_bytes=raw_bytes;next_offset_=0;
  }
  void set_run_limit(std::size_t count){if(!count)throw std::invalid_argument("run limit must be positive");run_limit_=count;}
  void reserve_runs(std::size_t count){batch_.runs.reserve(count);}
  void feed(const std::uint8_t* bytes,std::size_t size,std::size_t offset) {
    if((size&&!bytes)||size%2||offset!=next_offset_||offset>batch_.raw_bytes||size>batch_.raw_bytes-offset)
      throw std::invalid_argument("RAW packets must cover the batch in order with whole words");
    next_offset_+=size;bool acquisition_high=false;
    for(std::size_t i=0;i<size;i+=2) {
      const auto word=load(bytes+i);const unsigned code=word>>12,value=word&4095;
      if(!valid_time_) {
        if(code==8){update_high(value);acquisition_high=true;}
        else if(code==6){low_=value;if(acquisition_high)valid_time_=true;}
        continue;
      }
      switch(code) {
        case 0:y_=value&2047;break;
        case 2: {
          std::size_t end=i+2;while(end<size&&(bytes[end+1]>>4)==2)end+=2;
          const auto count=(end-i)/2;std::size_t valid=0;
          if(y_>=batch_.input_height)stats_.bounds+=count;
          else for(auto p=i;p<end;p+=2) {if((load(bytes+p)&2047)<batch_.input_width)++valid;else ++stats_.bounds;}
          if(valid&&prepare(time(),valid))append((offset+i)/2,count,2,0,0);
          i=end-2;break;
        }
        case 3:base_=value&2047;polarity_=(value>>11)&1;break;
        case 4:case 5: {
          const auto count=code==4?12u:8u;std::size_t valid=0;
          for(unsigned b=0;b<count;++b)if(value&(1u<<b)) {
            const auto x=static_cast<std::uint16_t>(base_+b);
            if(x<batch_.input_width&&y_<batch_.input_height)++valid;else ++stats_.bounds;
          }
          if(valid&&prepare(time(),valid))append((offset+i)/2,1,code,base_,polarity_);
          base_=static_cast<std::uint16_t>(base_+count);break;
        }
        case 6:low_=value;break;
        case 8:update_high(value);break;
        case 10:++stats_.triggers;break;
        case 7:case 9:case 14:case 15:break;
        default:throw std::runtime_error("Unsupported EVT3 word code");
      }
    }
  }
  const Evt3RunBatch& batch() const {
    if(next_offset_!=batch_.raw_bytes)throw std::logic_error("Incomplete RAW batch");return batch_;
  }
  const Statistics& statistics() const{return stats_;}
private:
  static std::uint16_t load(const std::uint8_t* p){return p[0]|(std::uint16_t(p[1])<<8);}
  std::int64_t time() const{return static_cast<std::int64_t>(high_|low_);}
  void update_high(unsigned value){const auto old=(high_>>12)&4095;if(value<old&&old-value>10)high_+=1ULL<<24;high_=(high_&~((1ULL<<24)-1))|(std::uint64_t(value)<<12);}
  void clear_epoch(){auto& m=batch_.metadata;++m.epoch;m.origin_us=0;m.latest_us=0;m.newest_bin=-1;batch_.ready=false;batch_.accepted_events=0;batch_.runs.clear();}
  bool prepare(std::int64_t t,std::size_t count) {
    auto& m=batch_.metadata;
    if(batch_.ready&&t<m.latest_us) {
      stats_.backward+=count;stats_.backward_max_us=std::max(stats_.backward_max_us,static_cast<std::uint64_t>(m.latest_us-t));
      if(m.latest_us-t<=tolerance_){stats_.dropped+=count;return false;}
      ++stats_.resets;clear_epoch();
    }
    if(!batch_.ready){batch_.ready=true;m.origin_us=t;}
    m.latest_us=t;m.newest_bin=(t-m.origin_us)/m.bin_width_us;stats_.accepted+=count;batch_.accepted_events+=count;return true;
  }
  void append(std::size_t word_offset,std::size_t count,unsigned kind,unsigned base,unsigned polarity) {
    if(batch_.runs.size()>=run_limit_)throw std::length_error("EVT3 run capacity exceeded");
    batch_.runs.push_back(Evt3Run{batch_.metadata.newest_bin,static_cast<std::uint32_t>(word_offset),
      static_cast<std::uint32_t>(count),static_cast<std::uint16_t>(y_*height_/batch_.input_height),
      static_cast<std::uint16_t>(base),static_cast<std::uint8_t>(kind),static_cast<std::uint8_t>(polarity),0});
  }
  std::size_t height_,next_offset_{0},run_limit_{std::numeric_limits<std::size_t>::max()};std::int64_t tolerance_;
  std::uint64_t high_{0};unsigned low_{0},y_{0},polarity_{0};std::uint16_t base_{0};bool valid_time_{false};
  Evt3RunBatch batch_;Statistics stats_;
};
} // namespace openeb_ros2
#endif
