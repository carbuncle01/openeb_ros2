#include "openeb_ros2/evt3_grouped_stream.hpp"
#include "evt3_run_mirror.hpp"
#include <iostream>
void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
std::vector<std::uint8_t> bytes(std::initializer_list<unsigned> words){std::vector<std::uint8_t>b;for(auto w:words){b.push_back(w&255);b.push_back(w>>8);}return b;}
int main()try {
  using openeb_ros2::Evt3GroupedStream;
  Evt3GroupedStream stream(160,120,10,40000,true,true,4000,1048576,65536);
  Evt3RunMirror mirror(160,120,10,40000,true,true);
  openeb_ros2::Evt3RunsHistogram reference(160,120,10,40000,true,true);reference.set_geometry(640,480);
  std::vector<float>a(reference.elements()),b(a.size());
  auto data=bytes({0x8000,0x63e8,0x0001,0x2801,0x2002});
  const auto& first=stream.ingest(data.data(),data.size(),640,480,0);
  require(stream.reset_before()&&stream.geometry_changed()&&first.ready,"first packet acquisition/reset");
  reference.feed(data.data(),data.size());mirror.apply(first,data.data());
  reference.snapshot(a.data(),41000);mirror.snapshot(b.data(),41000);require(a==b,"first tensor differs");
  auto epoch=first.metadata.epoch;
  data=bytes({0x63e7,0x2801});const auto& small=stream.ingest(data.data(),data.size(),640,480,1);
  require(!stream.reset_before()&&small.metadata.epoch==epoch&&small.runs.empty()&&stream.statistics().dropped==1,"small reversal reset stream");
  data=bytes({0x2804});const auto& gap=stream.ingest(data.data(),data.size(),640,480,3);
  require(stream.reset_before()&&!gap.ready,"packet gap did not invalidate parser");
  data=bytes({0x8000,0x63e8,0x0001,0x2805});const auto& reacquired=stream.ingest(data.data(),data.size(),640,480,4);
  require(reacquired.ready&&reacquired.metadata.origin_us==1000,"gap recovery failed");
  const auto& reordered=stream.ingest(data.data(),data.size(),640,480,2);require(stream.reset_before()&&reordered.ready,"reorder reset missing");
  stream.ingest(data.data(),data.size(),641,481,3);require(stream.geometry_changed()&&stream.reset_before(),"geometry reset missing");
  stream.reset();data=bytes({0x2806});require(!stream.ingest(data.data(),data.size(),641,481,4).ready,"explicit loss reset missing");
  Evt3GroupedStream limited(160,120,10,40000,true,true,4000,32,1);
  data=bytes({0x8000,0x63e8,0x0001,0x2801,0x0002,0x2802});bool rejected=false;
  try{limited.ingest(data.data(),data.size(),640,480,0);}catch(const std::length_error&){rejected=true;}require(rejected,"run cap ignored");
  data=bytes({0x2801});require(!limited.ingest(data.data(),data.size(),640,480,1).ready,"run cap failure left valid parser");
  std::vector<std::uint8_t> oversized(34);rejected=false;
  try{limited.ingest(oversized.data(),oversized.size(),640,480,2);}catch(const std::length_error&){rejected=true;}require(rejected,"packet cap ignored");
  data=bytes({0x8000,0x63e8,0x0001,0x2801});require(limited.ingest(data.data(),data.size(),640,480,3).ready,"limit recovery failed");
  data=bytes({0xb000});rejected=false;try{limited.ingest(data.data(),data.size(),640,480,4);}catch(const std::runtime_error&){rejected=true;}require(rejected,"bad code accepted");
  std::cout<<"evt3_grouped_stream_test passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
