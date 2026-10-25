#include "openeb_ros2/event_tensor_cuda_backend.hpp"
#include "openeb_ros2/evt3_run_scanner.hpp"
#include "openeb_ros2/evt3_runs_histogram.hpp"
#include <cstring>
#include <iostream>
void check(cudaError_t e){if(e!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(e));}
void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
std::vector<std::uint8_t> bytes(const std::vector<unsigned>& words){std::vector<std::uint8_t>b;for(auto w:words){b.push_back(w&255);b.push_back(w>>8);}return b;}
int main()try {
  cudaStream_t stream;check(cudaStreamCreate(&stream));float* device=nullptr;check(cudaMalloc(reinterpret_cast<void**>(&device),48*sizeof(float)));
  for(bool separate:{false,true})for(bool major:{false,true}) {
    auto selected=major?stream:nullptr;
    openeb_ros2::EventTensorCudaBackend gpu(4,3,2,separate,major,2000,16,{1.0},{2.0},false);
    gpu.reserve_grouped_raw(65536,32768);
    openeb_ros2::Evt3RunScanner scanner(4,3,2,2000,separate,major);scanner.set_geometry(8,6);
    openeb_ros2::Evt3RunsHistogram reference(4,3,2,2000,separate,major);reference.set_geometry(8,6);
    std::vector<float> expected(reference.elements()),actual(expected.size());
    auto compare=[&](){const auto& m=scanner.batch().metadata;auto end=m.origin_us+std::max<std::int64_t>(2,m.newest_bin)*1000;
      reference.snapshot(expected.data(),end,{1.0},{2.0});gpu.snapshot(device,end,selected);check(cudaStreamSynchronize(selected));
      check(cudaMemcpy(actual.data(),device,actual.size()*sizeof(float),cudaMemcpyDeviceToHost));
      require(!std::memcmp(actual.data(),expected.data(),actual.size()*sizeof(float)),"Grouped GPU Tensor differs");};
    for(unsigned frame=0;frame<80;++frame) {
      const auto t=frame<20?20000+frame*100:frame<40?200000+(frame-20)*100:frame<60?196000+(frame-40)*100:2000000+(frame-60)*100;
      std::vector<unsigned> words{0x8000|((t>>12)&4095),0x6000|(t&4095),frame%9==8?9u:1u};
      for(unsigned i=0;i<80;++i)words.push_back(0x2000|(i%10)|((i%2)<<11));
      words.push_back(0x3807);words.push_back(0x400f);words.push_back(0x5003);
      auto data=bytes(words);scanner.begin_batch(data.size());
      // Acquire HIGH/LOW together, then alternate packet lengths across frames.
      reference.feed(data.data(),4);scanner.feed(data.data(),4,0);
      for(std::size_t pos=4;pos<data.size();){const auto n=std::min<std::size_t>(frame%2?14:64,data.size()-pos);
        reference.feed(data.data()+pos,n);scanner.feed(data.data()+pos,n,pos);pos+=n;}
      require(reference.statistics().accepted==scanner.statistics().accepted,"Grouped event count differs");
      gpu.update_grouped_raw(scanner.batch(),data.data(),data.size(),selected);
      std::fill(data.begin(),data.end(),0); // H2D must use owned pinned data.
      // More than two uploads run without global synchronization, forcing
      // safe reuse of both grouped staging slots and preserving RAW ownership.
      if(frame%11==10||frame==79)compare();
    }
    compare();
    auto data=bytes({0xe000,0xf000});scanner.begin_batch(data.size());scanner.feed(data.data(),data.size(),0);
    gpu.update_grouped_raw(scanner.batch(),data.data(),data.size(),selected);compare();
    auto invalid=scanner.batch();invalid.runs.push_back({0,100,1,0,0,2,1,0});bool rejected=false;
    try{gpu.update_grouped_raw(invalid,data.data(),data.size(),selected);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"Invalid grouped range accepted");compare();
    gpu.reset(selected);gpu.update_grouped_raw(scanner.batch(),data.data(),data.size(),selected);
    // reset drops all old pixels; a metadata-only batch stays empty.
    gpu.snapshot(device,scanner.batch().metadata.origin_us+2000,selected);check(cudaStreamSynchronize(selected));
    check(cudaMemcpy(actual.data(),device,actual.size()*sizeof(float),cudaMemcpyDeviceToHost));
    for(auto value:actual)require(value==-0.5f,"Reset grouped histogram is not empty");
  }
  check(cudaFree(device));check(cudaStreamDestroy(stream));
  std::cout<<"evt3_runs_grouped_cuda_test passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
