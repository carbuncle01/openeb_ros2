// Shared, ROS-free RAW replay input. Official decoder is an external dependency.
#pragma once
#include "event_camera_codecs/evt3_decoder.h"
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>
namespace raw_bench {
struct Packet {std::vector<std::uint8_t> events;std::uint64_t time_base{0};};
struct Raw {
  unsigned width{0},height{0};std::uint64_t file_payload_bytes{0};
  std::vector<std::uint8_t> bytes;
};
inline Raw load(const std::string& path,std::uint64_t limit) {
  std::ifstream in(path,std::ios::binary);if(!in)throw std::runtime_error("Cannot open RAW: "+path);
  Raw raw;bool evt3=false;std::string line;
  while(in.peek()=='%') {
    std::getline(in,line);if(line.size()>65536)throw std::runtime_error("Invalid RAW header");
    if(!line.empty()&&line.back()=='\r')line.pop_back();
    if(line.rfind("% format ",0)==0) {
      evt3=line.rfind("% format EVT3;",0)==0;
      const auto x=line.find("width="),y=line.find("height=");
      if(x!=std::string::npos)raw.width=std::stoul(line.substr(x+6));
      if(y!=std::string::npos)raw.height=std::stoul(line.substr(y+7));
    }
    if(line=="% end")break;
  }
  if(!evt3||!raw.width||!raw.height||raw.width>2048||raw.height>2048)
    throw std::runtime_error("Require EVT3 RAW with width/height in % format header");
  const auto offset=in.tellg();if(offset<0)throw std::runtime_error("Invalid RAW offset");
  in.seekg(0,std::ios::end);const auto end=in.tellg();
  if(end<offset)throw std::runtime_error("Invalid RAW length");
  raw.file_payload_bytes=static_cast<std::uint64_t>(end-offset);
  if(raw.file_payload_bytes%2)throw std::runtime_error("Truncated EVT3 word");
  const auto count=limit?std::min(limit,raw.file_payload_bytes)&~std::uint64_t(1):raw.file_payload_bytes;
  if(!count)throw std::runtime_error("Empty RAW payload");
  raw.bytes.resize(count);in.seekg(offset);in.read(reinterpret_cast<char*>(raw.bytes.data()),count);
  if(static_cast<std::uint64_t>(in.gcount())!=count)throw std::runtime_error("RAW read failed");
  return raw;
}
// Same bounds, 4ms backward-drop and LUT scaling rules as the existing node.
// A larger reversal is rejected explicitly: this benchmark measures one epoch.
// Event is templated so the timed path writes directly into vector<CudaEvent>.
struct Event {std::int64_t timestamp_us;std::uint16_t x,y;std::uint8_t polarity,padding[3];};
template<class E=Event> struct Collector {
  unsigned width,height;std::vector<std::uint16_t> xs,ys;std::vector<E> events;
  bool capture{true},ready{false};std::int64_t origin{0},latest{0};
  std::uint64_t accepted{0},bounds{0},dropped{0};
  Collector(unsigned w,unsigned h,bool store=true):width(w),height(h),xs(w),ys(h),capture(store) {
    for(unsigned x=0;x<w;++x)xs[x]=x*160/w;
    for(unsigned y=0;y<h;++y)ys[y]=y*120/h;
  }
  void eventCD(std::uint64_t ns,std::uint16_t x,std::uint16_t y,std::uint8_t p) {
    if(x>=width||y>=height){++bounds;return;}
    const auto us=static_cast<std::int64_t>(ns/1000);
    if(ready&&us<latest) {
      if(latest-us<=4000){++dropped;return;}
      throw std::runtime_error("RAW has >4ms backward jump; benchmark requires a single clock epoch");
    }
    if(!ready){origin=us;ready=true;}latest=us;++accepted;
    if(capture)events.push_back(E{us,xs[x],ys[y],static_cast<std::uint8_t>(p!=0),{0,0,0}});
  }
  bool eventExtTrigger(std::uint64_t,std::uint8_t,std::uint8_t){return true;}
  void finished(){}void rawData(const char*,std::size_t){}
};
struct Frame {
  std::size_t begin,end;std::int64_t window_end,latest,origin;
  std::uint64_t accepted,bounds,dropped;
};
inline std::vector<Frame> index(const Raw& raw,std::size_t packet_bytes,std::size_t max_frames) {
  if(packet_bytes<2||packet_bytes%2)throw std::invalid_argument("packet-bytes must be positive and even");
  Collector<> clock(raw.width,raw.height,false);
  event_camera_codecs::evt3::Decoder<Packet,Collector<>> decoder;decoder.setGeometry(raw.width,raw.height);
  std::vector<Frame> frames;std::size_t begin=0;std::int64_t next=0;
  for(std::size_t pos=0;pos<raw.bytes.size();) {
    const auto n=std::min(packet_bytes,raw.bytes.size()-pos);
    decoder.decode(raw.bytes.data()+pos,n,&clock);pos+=n;
    if(!clock.ready)continue;
    if(!next)next=clock.origin+40000; // Fill first window as mandatory warmup.
    if(clock.latest>=next) {
      const auto end=clock.origin+(clock.latest-clock.origin)/4000*4000;
      frames.push_back({begin,pos,end,clock.latest,clock.origin,clock.accepted,clock.bounds,clock.dropped});
      begin=pos;next=end+4000;
      if(max_frames&&frames.size()>=max_frames)break;
    }
  }
  if(frames.empty())throw std::runtime_error("Not enough valid RAW time for a 40ms window");
  return frames; // A trailing partial window remains explicitly unprocessed.
}
template<class Fn> void packets(const Raw& raw,const Frame& frame,std::size_t size,Fn fn) {
  for(auto pos=frame.begin;pos<frame.end;pos+=size)
    fn(raw.bytes.data()+pos,std::min(size,frame.end-pos));
}
} // namespace raw_bench
