// Decoded-event aggregation ablation. RAW decode, event-array construction,
// USB, ROS, inference and D2H validation are OUTSIDE timing. CPU aggregation,
// staging, H2D, snapshot/normalization and completion wait are INSIDE timing.
// Scalar CPU event() is used; this is not the optimized ADDR_X decoder path.
#include "openeb_ros2/event_tensor_cuda_backend.hpp"
#include "openeb_ros2/evt3_runs_histogram.hpp"
#include <cuda_runtime_api.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
using Clock=std::chrono::steady_clock;
void check(cudaError_t r){if(r!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(r));}
struct Result {std::vector<double> ms;std::uint64_t bytes{0},over_budget{0};};
void report(const char* name,Result result){std::sort(result.ms.begin(),result.ms.end());double sum=0;for(auto ms:result.ms)sum+=ms;
  std::cout<<std::quoted(name)<<":{\"mean_ms\":"<<sum/result.ms.size()<<",\"p50_ms\":"<<result.ms[result.ms.size()/2]<<",\"p95_ms\":"<<result.ms[(result.ms.size()-1)*95/100]<<",\"p99_ms\":"<<result.ms[(result.ms.size()-1)*99/100]<<",\"h2d_payload_bytes\":"<<result.bytes<<",\"over_4ms\":"<<result.over_budget<<"}";}
int main(int argc,char**argv)try {
  unsigned iterations=300,events=100000,warmup=30;std::string pattern="uniform";
  for(int i=1;i<argc;++i){if(i+1==argc)throw std::invalid_argument("Missing value");std::string key=argv[i],v=argv[++i];
    if(key=="--iterations")iterations=std::stoul(v);else if(key=="--events")events=std::stoul(v);else if(key=="--warmup")warmup=std::stoul(v);else if(key=="--pattern")pattern=v;else throw std::invalid_argument("Unknown option");}
  if(iterations<10||iterations%3||warmup<10||!events||events>10000000||(pattern!="uniform"&&pattern!="hotspot"))throw std::invalid_argument("iterations>=10 divisible by 3, warmup>=10, events=1..10000000, pattern=uniform|hotspot required");
  int count=0,device_id=0,driver=0,runtime=0;check(cudaGetDeviceCount(&count));if(!count)throw std::runtime_error("No CUDA GPU");
  check(cudaGetDevice(&device_id));cudaDeviceProp properties{};check(cudaGetDeviceProperties(&properties,device_id));
  check(cudaDriverGetVersion(&driver));check(cudaRuntimeGetVersion(&runtime));
  cudaStream_t stream;check(cudaStreamCreate(&stream));float* device=nullptr;
  constexpr std::size_t elements=20*160*120,bytes=elements*sizeof(float);check(cudaMalloc(reinterpret_cast<void**>(&device),bytes));
  std::array<Result,3> results;
  {
    openeb_ros2::Evt3RunsHistogram full(160,120,10,40000,true,true),dirty(160,120,10,40000,true,true);
    full.set_geometry(160,120);dirty.set_geometry(160,120);
    openeb_ros2::EventTensorCudaBackend bins_gpu(160,120,10,true,true,40000,8192,{0.0},{1.0},false);
    openeb_ros2::EventTensorCudaBackend events_gpu(160,120,10,true,true,40000,8192,{0.0},{1.0});
    openeb_ros2::HistogramBinUpdates updates;std::vector<openeb_ros2::CudaEvent> input(events);
    std::vector<float> host(elements),reference(elements),actual(elements);std::uint32_t seed=42;
    for(unsigned frame=0;frame<iterations+warmup;++frame){
      const auto base=1000+static_cast<std::int64_t>(frame)*4000;
      for(unsigned i=0;i<events;++i){seed=1664525u*seed+1013904223u;
        input[i]={base+static_cast<std::int64_t>(std::uint64_t(i)*4000/events),
          static_cast<std::uint16_t>(pattern=="uniform"?seed%160:32+(seed&7)),
          static_cast<std::uint16_t>(pattern=="uniform"?(seed>>8)%120:32+((seed>>3)&7)),
          static_cast<std::uint8_t>(seed>>31),{0,0,0}};
      }
      const auto end=1000+static_cast<std::int64_t>(std::max(10u,frame+1))*4000;
      // Rotation balances order and cache effects over three frames.
      for(unsigned order=0;order<3;++order){const unsigned mode=(order+frame)%3;
        const auto start=Clock::now();std::size_t sent=0;
        if(mode==0){for(const auto& e:input)full.event(e.timestamp_us,e.x,e.y,e.polarity);full.snapshot(host.data(),end);check(cudaMemcpyAsync(device,host.data(),bytes,cudaMemcpyHostToDevice,stream));sent=bytes;}
        else if(mode==1){for(const auto& e:input)dirty.event(e.timestamp_us,e.x,e.y,e.polarity);dirty.take_dirty_bins(updates);sent=updates.values.size()*sizeof(float);bins_gpu.update_bins(updates,stream);bins_gpu.snapshot(device,end,stream);}
        else {events_gpu.update(input,stream);events_gpu.snapshot(device,end,stream);sent=input.size()*sizeof(openeb_ros2::CudaEvent);}
        check(cudaStreamSynchronize(stream));const double ms=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
        if(frame>=warmup){results[mode].ms.push_back(ms);results[mode].bytes+=sent;if(ms>4.0)++results[mode].over_budget;}
        check(cudaMemcpy(actual.data(),device,bytes,cudaMemcpyDeviceToHost));
        // full has not necessarily processed this frame yet. Keep each result
        // until all three strategies reach the same event-time window.
        if(order==0)reference=actual;else if(actual!=reference)throw std::runtime_error("Strategies produce different tensors");
      }
    }
  }
  check(cudaFree(device));check(cudaStreamDestroy(stream));
  std::cout<<std::setprecision(10)<<"{\"gpu\":"<<std::quoted(properties.name)<<",\"sm\":"<<properties.major*10+properties.minor<<",\"driver\":"<<driver<<",\"runtime\":"<<runtime<<",\"scope\":\"decoded_event_aggregation\",\"iterations\":"<<iterations<<",\"warmup\":"<<warmup<<",\"events_per_4ms\":"<<events<<",\"events_per_second\":"<<std::uint64_t(events)*250<<",\"pattern\":"<<std::quoted(pattern)<<",\"all_outputs_identical\":true,";
  report("cpu_full_tensor",results[0]);std::cout<<',';report("cpu_bins_gpu_tensor",results[1]);std::cout<<',';report("gpu_event_aggregation",results[2]);std::cout<<"}\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
