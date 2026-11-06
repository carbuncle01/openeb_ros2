#include "openeb_ros2/evt3_runs_histogram.hpp"
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>
using openeb_ros2::Evt3RunsHistogram;
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
std::vector<std::uint8_t> bytes(std::initializer_list<std::uint16_t> words){std::vector<std::uint8_t> b;for(auto w:words){b.push_back(w&255);b.push_back(w>>8);}return b;}
int main()try {
  for(bool separate:{false,true})for(bool major:{false,true}) {
    Evt3RunsHistogram h(4,3,2,2000,separate,major);h.set_geometry(8,6);
    auto a=bytes({0x8000,0x6000,0x0002,0x2802,0x2003,0x63e8,0x2804,0x67d0,0x2006});
    h.feed(a.data(),a.size());
    std::vector<float> out(h.elements());h.snapshot(out.data(),2000);
    std::vector<float> expected(h.elements(),0.f);
    if(separate){expected[(major?0:0)*12+5]=1;expected[(major?2:1)*12+5]=1;expected[(major?1:2)*12+6]=1;}
    else {expected[12+6]=1;}
    require(out==expected,"half-open window/polarity/resize mismatch");
    h.snapshot(out.data(),2000,{1.0},{2.0});
    for(std::size_t i=0;i<out.size();++i)require(out[i]==(expected[i]-1.f)/2.f,"normalization mismatch");
    h.snapshot(out.data(),4000);std::fill(expected.begin(),expected.end(),0.f);
    expected[(separate?(major?2:1):0)*12+7]=separate?1.f:-1.f;
    require(out==expected,"future empty-bin decay mismatch");
    h.snapshot(out.data(),100000);for(float x:out)require(x==0,"expired ring replayed");
  }
  Evt3RunsHistogram h(4,3,2,2000,true,true);h.set_geometry(8,6);
  auto a=bytes({0x8000,0x6000,0x0002,0x3802,0x4007,0x0007,0x2801});h.feed(a.data(),a.size());
  require(h.statistics().accepted==3 && h.statistics().bounds==1,"vector/bounds mismatch");
  auto b=bytes({0x8000,0x6fff,0x0000,0x2800});h.feed(b.data(),b.size());
  auto c=bytes({0x6064,0x2800,0x2801});h.feed(c.data(),c.size());
  require(h.statistics().backward==2,"backward-run diagnostic mismatch");
  const auto epoch=h.epoch();h.reset_stream();require(!h.ready() && h.epoch()>epoch,"reset must invalidate ring");
  bool rejected=false;try{h.feed(a.data(),1);}catch(const std::invalid_argument&){rejected=true;}require(rejected,"odd packet accepted");
  {
    Evt3RunsHistogram clock(4,3,2,2000,true,true);clock.set_geometry(8,6);
    auto feed=[&](std::initializer_list<std::uint16_t> words){auto data=bytes(words);clock.feed(data.data(),data.size());};
    // Accumulate two 24-bit wraps, then simulate a forward packet loss.
    feed({0x8fff,0x6000,0x0000,0x2800});
    feed({0x8000,0x6000,0x2800});
    feed({0x8fff,0x6000,0x2800});
    feed({0x8000,0x6000,0x2800});
    feed({0x8c6e,0x6000,0x2800});
    const auto before=clock.latest();const auto old_epoch=clock.epoch();
    clock.reset_stream(true);
    require(!clock.ready() && clock.epoch()>old_epoch,"gap must clear histogram");
    feed({0x6001,0x0000,0x2800});
    require(!clock.ready(),"gap must reacquire HIGH and LOW");
    feed({0x8c6e,0x6064,0x0000,0x2800});
    require(clock.latest()==before+100,"gap lost accumulated 33554432 us");
    require(clock.origin()==before+100,"gap retained old histogram origin");
    // A wrap during the missing interval must also be counted.
    feed({0x8fff,0x6000,0x2800});
    clock.reset_stream(true);
    feed({0x8000,0x6064,0x0000,0x2800});
    require(clock.latest()==3*(1LL<<24)+100,"wrap inside gap was lost");
    // Explicit new-stream reset still starts a fresh clock.
    clock.reset_stream();feed({0x8000,0x6064,0x0000,0x2800});
    require(clock.latest()==100,"full reset retained previous clock");
  }
  std::cout<<"evt3_runs_histogram_test passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
