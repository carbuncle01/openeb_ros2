// Target-only microbenchmark: same CPU histogram, full tensor upload versus
// dirty-bin upload + GPU gather. Excludes decode/USB/inference, synchronizes
// each measured operation, so it does not predict the asynchronous pipeline.
#include "openeb_ros2/event_tensor_cuda_backend.hpp"
#include "openeb_ros2/evt3_runs_histogram.hpp"
#include <cuda_runtime_api.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
void check(cudaError_t r){if(r!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(r));}
struct Measurements {std::vector<double> ms;std::uint64_t bytes{0};};
void report(const char* name,Measurements m){std::sort(m.ms.begin(),m.ms.end());double sum=0;for(double x:m.ms)sum+=x;
  std::cout<<"\""<<name<<"\":{\"mean_ms\":"<<sum/m.ms.size()<<",\"p50_ms\":"<<m.ms[m.ms.size()/2]<<",\"p95_ms\":"<<m.ms[(m.ms.size()-1)*95/100]<<",\"p99_ms\":"<<m.ms[(m.ms.size()-1)*99/100]<<",\"h2d_payload_bytes\":"<<m.bytes<<"}";}
int main(int argc,char**argv)try {
  unsigned iterations=1000,events=100000;
  for(int i=1;i<argc;++i){if(i+1==argc)throw std::invalid_argument("Missing benchmark value");std::string key=argv[i];unsigned value=std::stoul(argv[++i]);if(key=="--iterations")iterations=value;else if(key=="--events")events=value;else throw std::invalid_argument("Unknown benchmark option");}
  if(iterations<10 || !events)throw std::invalid_argument("Need iterations>=10 and events>0");
  cudaStream_t stream;check(cudaStreamCreate(&stream));float* device=nullptr;
  constexpr std::size_t elements=20*160*120,bytes=elements*sizeof(float);
  check(cudaMalloc(reinterpret_cast<void**>(&device),bytes));
  {
    openeb_ros2::Evt3RunsHistogram cpu(160,120,10,40000,true,true);cpu.set_geometry(160,120);
    openeb_ros2::EventTensorCudaBackend gpu(160,120,10,true,true,40000,8192,{0.0},{1.0},false);
    openeb_ros2::HistogramBinUpdates updates;
    std::vector<float> expected(elements),host(elements),actual(elements);Measurements full,dirty;
    constexpr unsigned warmup=100;
    for(unsigned frame=0;frame<iterations+warmup;++frame){
      const std::int64_t time=1000+static_cast<std::int64_t>(frame)*4000;
      for(unsigned i=0;i<events;++i)cpu.event(time,i%160,(i/160)%120,i%2);
      const auto end=1000+static_cast<std::int64_t>(std::max(10u,frame+1))*4000;
      cpu.snapshot(expected.data(),end); // independent readout check, outside timing
      for(unsigned order=0;order<2;++order){
        const bool use_bins=(order^(frame%2))!=0;
        const auto start=std::chrono::steady_clock::now();std::size_t sent=bytes;
        if(use_bins){cpu.take_dirty_bins(updates);sent=updates.values.size()*sizeof(float);gpu.update_bins(updates,stream);gpu.snapshot(device,end,stream);}
        else {cpu.snapshot(host.data(),end);check(cudaMemcpyAsync(device,host.data(),bytes,cudaMemcpyHostToDevice,stream));}
        check(cudaStreamSynchronize(stream));
        const auto ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        if(frame>=warmup){auto& result=use_bins?dirty:full;result.ms.push_back(ms);result.bytes+=sent;}
        check(cudaMemcpy(actual.data(),device,bytes,cudaMemcpyDeviceToHost)); // outside timing
        if(actual!=expected)throw std::runtime_error("Benchmark output differs");
      }
    }
    std::cout<<"{\"iterations\":"<<iterations<<",\"events_per_4ms\":"<<events<<",\"all_outputs_identical\":true,";
    report("full_cpu_snapshot",full);std::cout<<',';report("dirty_bins_gpu_snapshot",dirty);std::cout<<"}\n";
  }
  check(cudaFree(device));check(cudaStreamDestroy(stream));
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
