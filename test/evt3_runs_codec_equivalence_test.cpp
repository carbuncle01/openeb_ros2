// Runs against the installed event_camera_codecs, also usable without ROS.
#include "openeb_ros2/evt3_runs_histogram.hpp"
#include "event_camera_codecs/evt3_decoder.h"
#include "bin_update_mirror.hpp"
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>
struct Packet {std::vector<std::uint8_t> events;std::uint64_t time_base{0};};
struct Reference {
  openeb_ros2::Evt3RunsHistogram histogram;
  Reference(unsigned w,unsigned h,bool separate,bool major):histogram(160,120,10,40000,separate,major){histogram.set_geometry(w,h);}
  void eventCD(std::uint64_t t,std::uint16_t x,std::uint16_t y,std::uint8_t p){histogram.event(t/1000,x,y,p);}
  bool eventExtTrigger(std::uint64_t,std::uint8_t,std::uint8_t){return true;}
  void finished(){}
  void rawData(const char*,std::size_t){}
};
struct Comparison {
  Reference reference;
  event_camera_codecs::evt3::Decoder<Packet,Reference> decoder;
  openeb_ros2::Evt3RunsHistogram direct;
  std::vector<float> expected,actual,mirrored;
  BinUpdateMirror mirror;
  openeb_ros2::HistogramBinUpdates updates;
  std::uint64_t uploaded_bytes{0};
  std::uint64_t snapshots{0},digest{1469598103934665603ULL};
  std::int64_t next{0};
  Comparison(unsigned w,unsigned h,bool separate=true,bool major=true)
  :reference(w,h,separate,major),direct(160,120,10,40000,separate,major),expected(direct.elements()),actual(direct.elements()),mirror(160,120,10,4000,separate,major) {
    decoder.setGeometry(w,h);direct.set_geometry(w,h);
  }
  void compare(std::int64_t end) {
    reference.histogram.snapshot(expected.data(),end);direct.snapshot(actual.data(),end);
    direct.take_dirty_bins(updates);uploaded_bytes+=updates.values.size()*sizeof(float);
    mirror.apply(updates);mirror.snapshot(mirrored,end);
    if(mirrored!=actual)throw std::runtime_error("Dirty-bin mirror differs");
    if(std::memcmp(expected.data(),actual.data(),actual.size()*sizeof(float)))throw std::runtime_error("Tensor differs at window end "+std::to_string(end));
    for(float f:actual){std::uint32_t b;std::memcpy(&b,&f,4);digest=(digest^b)*1099511628211ULL;}
    ++snapshots;
  }
  void feed(const std::uint8_t* data,std::size_t size) {
    decoder.decode(data,size,&reference);direct.feed(data,size);
    const auto& a=direct.statistics();const auto& b=reference.histogram.statistics();
    if(a.accepted!=b.accepted || a.backward!=b.backward || a.resets!=b.resets || direct.ready()!=reference.histogram.ready())
      throw std::runtime_error("Decoder state/count differs");
    if(!direct.ready())return;
    if(direct.origin()!=reference.histogram.origin() || direct.latest()!=reference.histogram.latest())throw std::runtime_error("Sensor clock differs");
    if(next==0)next=direct.origin()+40000;
    while(next<=direct.latest()){compare(next);next+=4000;}
  }
};
std::vector<std::uint8_t> bytes(const std::vector<std::uint16_t>& words){std::vector<std::uint8_t> b;for(auto w:words){b.push_back(w&255);b.push_back(w>>8);}return b;}
int main(int argc,char**argv)try {
  if(argc==3 && std::string(argv[1])=="--raw") {
    std::ifstream in(argv[2],std::ios::binary);if(!in)throw std::runtime_error("Cannot open RAW");
    unsigned w=0,h=0;std::string line;
    while(in.peek()=='%'){std::getline(in,line);if(!line.empty()&&line.back()=='\r')line.pop_back();if(line.rfind("% format ",0)==0){auto a=line.find("width="),b=line.find("height=");if(a!=std::string::npos)w=std::stoul(line.substr(a+6));if(b!=std::string::npos)h=std::stoul(line.substr(b+7));}if(line=="% end")break;}
    if(!w||!h)throw std::runtime_error("Invalid RAW geometry");
    Comparison c(w,h);std::vector<std::uint8_t> payload(65536);
    while(in){in.read(reinterpret_cast<char*>(payload.data()),payload.size());auto n=in.gcount();if(n%2)throw std::runtime_error("Truncated word");if(n)c.feed(payload.data(),n);}
    if(!in.eof())throw std::runtime_error("RAW read failed");
    std::cout<<"{\"events\":"<<c.direct.statistics().accepted<<",\"tensors\":"<<c.snapshots<<",\"bin_upload_bytes\":"<<c.uploaded_bytes<<",\"full_tensor_upload_bytes\":"<<c.snapshots*c.direct.elements()*sizeof(float)<<",\"digest\":\""<<c.digest<<"\",\"identical\":true}\n";return 0;
  }
  for(bool separate:{false,true})for(bool major:{false,true})for(unsigned chunk:{2u,14u,65536u}) {
    Comparison c(641,481,separate,major);
    auto first=bytes({0x8fff});c.feed(first.data(),first.size()); // HIGH/LOW split before acquisition.
    auto second=bytes({0x6000,0x2801});c.feed(second.data(),second.size());
    std::vector<std::uint16_t> words{0x8fff,0x6ff0,0x0002,0x2801,0x2002};
    std::mt19937 random(42);
    for(unsigned t=0;t<200000;t+=713) {
      const auto time=0xfffff0ULL+t;
      words.push_back(0x8000|((time>>12)&4095));words.push_back(0x6000|(time&4095));
      words.push_back(random()%482);
      for(unsigned n=0;n<15;++n)words.push_back(0x2000|(random()%2048)|((random()%2)<<11));
      words.push_back(0x3800|(random()%650));words.push_back(0x4000|(random()%4096));words.push_back(0x5000|(random()%4096));
      words.push_back(0xa001);words.push_back(0x9000);words.push_back(0xe000);words.push_back(0xf000);
    }
    auto data=bytes(words);
    // Acquire valid clock in one packet; then stress every subsequent boundary.
    c.feed(data.data(),8);
    for(std::size_t i=8;i<data.size();i+=chunk)c.feed(data.data()+i,std::min<std::size_t>(chunk,data.size()-i));
    c.compare(c.next+40000);c.compare(c.next+100000);
  }
  std::cout<<"evt3_runs_codec_equivalence_test passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
