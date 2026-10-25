# NVIDIAノートPCでの単独CUDA比較

Jetsonは必須ではない。CUDA対応NVIDIA GPU、対応driver/CUDA Toolkit、C++17 compiler、
CMake 3.24以上、sweep用Python 3を使う。LinuxまたはWindowsのWSL2を想定。
ROS、OpenEB SDK、カメラ、TensorRT、Jetsonライブラリは不要。
このdirectoryのCMakeはpackage本体のament entry pointと独立しており、
本番と同じsrc/event_tensor_cuda_backend.cuを直接ビルドする。
AMD/Intel GPU・Apple GPUではこのCUDAコードをそのまま実行できない。

- [NVIDIA CUDA on WSL](https://docs.nvidia.com/cuda/wsl-user-guide/)
- [CMake CUDA_ARCHITECTURES](https://cmake.org/cmake/help/latest/prop_tgt/CUDA_ARCHITECTURES.html)

## 実行

openeb_ros2のcheckout rootで実行する。kart全体のビルドやROS環境のsourceは不要。
WSL2のGPU設定は上の公式手順に従い、CUDA Toolkitと対応host compilerを用意する。

```bash
nvidia-smi
nvcc --version
cmake --version
cmake -S tools/cuda_benchmark -B build/cuda_benchmark -DCMAKE_BUILD_TYPE=Release
cmake --build build/cuda_benchmark -j 2
ctest --test-dir build/cuda_benchmark --output-on-failure
python3 tools/cuda_benchmark/sweep.py \
  --binary build/cuda_benchmark/event_tensor_cuda_load_benchmark \
  --output results/cuda_sweep_01 --iterations 300 --repeats 3
```

resultsは未作成directoryを指定する。GPU名・SM・driver/runtime version、source hash、
全runのJSON、stdout/stderr、summary.csvを保存する。
status=completeと全runのall_outputs_identical=trueを確認する。
途中失敗はstatus=failed、実行中断はrunningで残り、完了と区別する。
CMakeは既定で接続GPUのarchitectureをnativeで選ぶ。
別GPU向けやGPU非表示のbuildでは、適切なSMを明示する:
`-DCMAKE_CUDA_ARCHITECTURES=<対象GPUのSM番号>`。
Windows nativeのmulti-config buildは別の実行ファイル位置になるため、この手順はWSL2を基準とする。

## 比較内容

### 1. 転送・Tensor構成のみ

```bash
./build/cuda_benchmark/histogram_bin_cuda_benchmark --iterations 1000 --events 100000
```

共通のCPU histogramを先に作り、完成Tensor転送と変更bin転送＋GPU構成を比較する。
CPU decode/集計を計測に含めないので、この結果だけで高イベント時の全体性能を判断しない。

### 2. 集計も含むイベント量別比較

```bash
./build/cuda_benchmark/event_tensor_cuda_load_benchmark \
  --iterations 300 --events 100000 --pattern uniform
```

同じdecode済みイベント列から次の3方式を比較する。

|方式|計測内の処理|
|---|---|
|cpu_full_tensor|CPU集計→CPU Tensor生成→全Tensor H2D|
|cpu_bins_gpu_tensor|CPU集計→変更bin H2D→GPU Tensor生成|
|gpu_event_aggregation|イベント列H2D→GPU集計→GPU Tensor生成|

1 frame=4ms相当、shape=[1,20,120,160]、40ms窓。直接実行は既定30回、sweepは既定100回のwarmup後、方式順を3 frameごとに
循環させて比較する。iterationsは3の倍数で10以上。
イベント数は1000/10000/100000/200000 per 4ms、つまり0.25/2.5/25/50 M events/s相当。
uniformと8×8 pixelへ集中するhotspotをsweepする。

計測にはCPU集計、staging、H2D、Tensor生成、GPU完了待ちを含む。
RAW decode、入力配列の生成・確保、USB、ROS、D2H照合、TensorRT推論は含まない。
CPU集計はscalar event()で実行し、ADDR_X runをまとめたdecode経路の計測ではない。
GPU方式もdecode済みイベント配列を入力とするため、本番の配列生成コストを含まない。
今回の比較は集計段階の特性を見るもので、本番3 profileの完全な再現ではない。
各処理を同期して比較するため、非同期overlapの効果やlive latencyは評価しない。
4msを超えた回数は集計stageのbudget超過であり、車両制御のdeadlineを意味しない。
全出力はframeごとにD2Hして全要素照合する。このD2H時間は計測から除外する。

summary.csvのp50/p95/p99とover_4ms_fractionを比較する。
高イベント時には、GPUへのイベント転送が増える一方、bin転送は出力寸法に主に依存する。
ただしCPU集計もイベント数に比例するため、GPUイベント集計が勝つ可能性もある。
イベントが集中した場合のGPU atomicAddの影響も分けて見る。
ノートPCでの勝敗をOrinへ直接外挿しない。最後に実RAW・ROS・推論併走で測定する。
電源状態、power profile、他のGPU workloadを揃え、複数回のrunを残す。

## GPUのないMacで確認できる部分

```bash
cmake -S tools/cuda_benchmark -B build/cpu_checks -DOPENEB_BENCH_CPU_ONLY=ON
cmake --build build/cpu_checks -j 2
ctest --test-dir build/cpu_checks --output-on-failure
python3 tools/cuda_benchmark/sweep.py --binary /not/built --dry-run --repeats 1
```

CPU-onlyではCUDA compiler/driverを探索せず、CPU core/更新binとPython runnerを検証する。
これはCUDA build成功やGPU動作確認を意味しない。

## 3. 実RAWのdecodeからGPU Tensor完成まで比較する（今回の主比較）

このテストでは公式event_camera_codecsのEVT3 decoderと本番のEvt3RunsHistogram、
本番のCUDA backendを直接使う。ROS/OpenEB SDKは不要だが、公式codecのheaderが必要。
本番と同じ版を比較したい場合は、そのcheckoutのincludeを指定する。
以下のclone例は取得時の版を使い、decoder headerのSHA256を結果へ記録する。

```bash
# openeb_ros2のrootで。clone先は未作成directoryを指定する。
git clone https://github.com/ros-event-camera/event_camera_codecs.git /tmp/event_camera_codecs
cmake -S tools/cuda_benchmark -B build/raw_cuda \
  -DCMAKE_BUILD_TYPE=Release -DOPENEB_BENCH_RAW=ON \
  -DEVENT_CAMERA_CODECS_INCLUDE_DIR=/tmp/event_camera_codecs/include
cmake --build build/raw_cuda -j 2
ctest --test-dir build/raw_cuda --output-on-failure

python3 tools/cuda_benchmark/raw_sweep.py \
  --binary build/raw_cuda/event_tensor_raw_cuda_benchmark \
  --raw /absolute/path/to/recording.raw \
  --output results/raw_cuda_01 --repeats 3
```

**このcheckoutの最新ファイルとRAWをLinux PCへコピーしてから実行する。**
Mac上にある変更が別PCへ自動反映されることはない。
既存のdecoded-event用sweep.pyとは実行ファイル・計測範囲が異なる。

|mode|計測する処理|
|---|---|
|events（既存cuda_async）|公式EVT3 decode → 座標LUT/時刻検査 → CudaEvent配列生成 → pinned staging/H2D → GPU集計 → GPU Tensor生成・正規化 → 完了待ち|
|evt3_runs|新EVT3 decode＋CPU直接集計 → CPU Tensor生成・正規化 → 全Tensor H2D → 完了待ち|
|evt3_runs_gpu|新EVT3 decode＋CPU直接集計 → dirty bin抽出/pinned staging/H2D → GPU Tensor生成・正規化 → 完了待ち|

shape=[1,20,120,160]、40ms窓、4ms stride、ON先のpolarity-major、mean=0/std=1。
元解像度はRAWの`% format EVT3;width=...;height=...` headerから読む。
既存方式のvectorは再利用し、必要時の容量増大も計測内。本番と同じ小逆転<=4msはdropする。
大きな逆転はepoch再現の曖昧さを避けて明示的に失敗する。複数epochの性能試験ではない。
既存方式のGPU updateは本番既定の8192イベント単位、vector初期reserveは16384に揃える。
元のUSB packet境界はRAWから復元できないため、共通の偶数byte packetへ分割する。
既定64KiB、`--packet-bytes`で変更できる。frame境界でもpacketを分割し直さない。

RAW読込みと比較窓の事前index作成は計測外で、同じ圧縮byte列を各方式へ入力する。
最初に40ms以上蓄積した窓を作り、その後はpacket終端が4ms境界を越えるたびに
1つの比較窓を作る。窓終端はセンサoriginへ整列し、3方式の時刻は同じ。
packetによる量子化や無イベント区間では処理対象のsensor spanは4msより長くなる。
CSVには実際のspanと入力イベントレートを残す。これは厳密な250Hz timer再現ではない。
末尾の未完成窓は処理せず、loaded/processed_payload_bytesで範囲を確認する。

各frameで方式順を循環させる。各方式の計測末尾でGPU完了を待ち、その後にD2Hして
**全Tensor要素の完全一致・decode採用数・drop数・センサ時刻を照合**する。
D2Hと照合は計測外。Tensor不一致・CUDA errorがあれば失敗し、成功結果を出さない。
GPU命令をenqueueしただけの時間を完了時間と取り違えない。

ただし本番の複数workerのキュー待ち、ROS publisher、USB受信、推論、frame間overlapは除外する。
既存方式では同一frame内で前packetのGPU処理と次packetのCPU decodeが重なる場合がある。
CPU/GPUの初期確保は計測外、イベントvectorのclear/生成とpinnedコピーは計測内。
タイマ計測と公平性確保のための同期・方式切替の影響は含まれる。
`cpu_decode_mean_ms`は既存方式ではdecode＋イベント列生成、新方式ではdecode＋CPU集計。
GPU実行時間のみの内訳ではなく、total_msが主指標。

### 保存結果と負荷別比較

- `report.json`: status、GPU/driver/runtime、codec header hash、入力RAW全体とbinary/source hash、全runの結果。
- `summary.csv`: 各方式のp50/p95/p99のrun間中央値、既存events比、転送量、4ms超過率。
- `windows-0.csv`など: frameごとの入力byte数、イベント数、sensor span、イベントレート、全体時間、decode時間、転送量。warmup行も区別して保存。
- `rate_summary.csv`: <10 / 10..30 / >=30 Mevents/sに分けた全体時間のp50/p95/p99。繰返しrunのsampleをpoolした値で、独立したデータ数ではない。
- stdout/stderr: 失敗も保存。status=completeとall_outputs_identical=trueを確認する。

既定はRAW payload先頭256MiBまでRAMへ読み、最大1000frame、最初30frameをwarmupとする。
`--max-bytes 0 --max-frames 0`で全RAWを扱う（約3GBのRAWならそれ以上の空きRAMが必要）。
wrapperの入力同定hashは常にファイル全体を計算し、この時間も計測外。
全RAWを3回再生する例:

```bash
python3 tools/cuda_benchmark/raw_sweep.py \
  --binary build/raw_cuda/event_tensor_raw_cuda_benchmark \
  --raw /absolute/path/to/recording.raw \
  --max-bytes 0 --max-frames 0 --repeats 3 \
  --output results/raw_cuda_full_01
```

このRAWが高イベントの区間を含まなければ、高負荷での勝敗は判定できない。
syntheticなイベント量別試験は前節のsweep.pyで補い、実RAW結果と区別する。
ノートPCとOrinではGPU、メモリ構成、電力枠が異なるので、Orinでの性能保証にはならない。
Macでは`OPENEB_BENCH_CPU_ONLY=ON`と`OPENEB_BENCH_RAW=ON`を併用すれば、
同じcodec headerに対するRAW index／直接decode／CPU Tensor／変更bin模擬照合を実行できる。
これはCUDAコンパイル・GPU動作の検証ではない。
