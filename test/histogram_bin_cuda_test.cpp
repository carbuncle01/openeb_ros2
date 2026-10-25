#include "openeb_ros2/event_tensor_cuda_backend.hpp"
#include "openeb_ros2/evt3_runs_histogram.hpp"
#include <cuda_runtime_api.h>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
void check(cudaError_t result){if(result!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(result));}
int main()try {
  cudaStream_t stream;check(cudaStreamCreate(&stream));
  float* device=nullptr;check(cudaMalloc(reinterpret_cast<void**>(&device),48*sizeof(float)));
  for(bool separate:{false,true})for(bool major:{false,true}) {
    const auto selected_stream = major ? stream : nullptr;
    // Non-identity normalization checks that it is performed by the GPU gather.
    openeb_ros2::EventTensorCudaBackend gpu(4,3,2,separate,major,2000,16,{1.0},{2.0},false);
    openeb_ros2::Evt3RunsHistogram cpu(4,3,2,2000,separate,major);cpu.set_geometry(8,6);
    openeb_ros2::HistogramBinUpdates update;
    std::vector<float> expected(cpu.elements()),actual(cpu.elements());
    auto upload=[&](){cpu.take_dirty_bins(update);gpu.update_bins(update,selected_stream);};
    auto compare=[&](std::int64_t end){
      cpu.snapshot(expected.data(),end,{1.0},{2.0});gpu.snapshot(device,end,selected_stream);
      check(cudaStreamSynchronize(selected_stream));
      check(cudaMemcpy(actual.data(),device,actual.size()*sizeof(float),cudaMemcpyDeviceToHost));
      for(std::size_t i=0;i<actual.size();++i)
        if(std::fabs(actual[i]-expected[i])>1e-6f)throw std::runtime_error("GPU dirty-bin tensor differs");
    };
    cpu.event(1000,2,2,1);upload();compare(3000);
    cpu.event(1100,3,2,0);upload();gpu.update_bins(update,selected_stream);compare(3000); // absolute resend
    // Multiple enqueued uploads reuse four pinned slots without a global sync.
    for(unsigned i=0;i<40;++i){cpu.event(1200+i,2,2,i%2);upload();}
    compare(3000);
    for(auto t:{2000,3000,4000,5000,6000,18000,20000}){cpu.event(t,3,3,t%2);upload();compare(t+2000);compare(t+20000);}
    upload();compare(22000); // metadata-only update
    cpu.reset_stream();cpu.event(10,1,1,1);upload();compare(2010);
    cpu.event(11,1,2,0);cpu.mark_all_dirty();upload();compare(2010);
    auto invalid=update;invalid.newest_bin=-1;bool rejected=false;
    try{gpu.update_bins(invalid,selected_stream);}catch(const std::invalid_argument&){rejected=true;}
    if(!rejected)throw std::runtime_error("Invalid GPU update accepted");
    compare(2010);
    cpu.mark_all_dirty();cpu.take_dirty_bins(update);
    gpu.reset(selected_stream);gpu.update_bins(update,selected_stream);compare(2010);
  }
  check(cudaFree(device));check(cudaStreamDestroy(stream));
  std::cout<<"histogram_bin_cuda_test passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
