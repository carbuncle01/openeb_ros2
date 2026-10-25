# kart branch検証記録

2026-10-09、ローカルmacOS。ROS/OpenEB/CUDAを実行できる環境ではない。

## 確認済み

- packet_router_test: clang++ C++17、-Wall -Wextra -Werrorでbuild・実行成功。
  direct有無×配信ON/OFFの4条件で、direct処理の継続、packet内容、所有権の移動、
  監視時だけのpacketコピーを確認。5 packet中で配信を切り替え、direct入力順序保持も確認。
- test/check_configuration.py: 5設定ファイルのparameter記載漏れ・余分なキーなし。
  cpu/cudaの選択、direct入力設定、既定20ch shape、launch AST、package依存を確認。
- 全launchのPython構文検証。
- git diff --check（実repoへ反映後に実行）。

AddressSanitizer/UBSan付きrouter実行は、このMacで応答せずテストプロセスを停止。
通常buildのテスト成功とは区別し、sanitizer検証成功とは扱わない。

## 未確認

- ROS 2 Lyrical / Isaac ROS 5.0 / OpenEB / CUDAでのcolcon build。
- 移植したevent_tensor_cuda_backend_testのGPU実行。
- 3方式の実カメラ入力、CPU/CUDA同値性、driver停止時の実SDK callback join。
- 実行中packet_publish_enabled変更のROS parameterサービス結合。
- TensorRTのcomponent load・shape/input binding接続、CUDA buffer共有・CPU fallback。
- latency、p95/p99、欠落、deadline、温度・負荷変動下での性能。

## 対象環境での確認順

1. OpenEB対応カメラ・Lyrical・Isaac ROS 5.0・CUDA buffer依存を確認してcolcon build。
2. colcon test --packages-select openeb_ros2 と colcon test-result --verbose。
3. tensor.launch.pyの各backendを別runで起動し、tensor_diagnosticsの出力・エラー・timestampを確認。
4. packet配信OFF時にtopic echo/events_raw購読を付けても配信されず、tensorが更新されることを確認。
   ON時に配信を再開し、OFFへ戻してtensor生成が続くことを確認。
5. TensorRTをtensor_containerへloadし、入力名・shape・時間表現・正規化をモデルに合わせる。
   GPU pointer/buffer backend・転送traceを確認し、実環境のzero-copy成立を判定する。
6. 同じイベント条件で3方式を比較する。内部診断とCUDA Eventを使い、
   packet配信ON/OFF、RAW保存ON/OFFを別条件にして中央値・p95/p99、queue age、dropを記録する。

ros2 topic hz/echoやbag保存は監視負荷・CPU fallbackを増やす可能性があるため、
GPU tensor payloadへの追加購読なしのrunも比較する。


## RAW連携・画像の追加検証

- event_palette_test: 白、ON赤、OFF青、同値ON優先、半強度、clampをC++で確認。
- parameter/config整合検査をdriver_bag_linked.yaml / visualization.yamlへ拡張。
- kart側RawRelayの12テストを含む21テストが通過。保存先設定後のSTART、処理中STOP、
  directory拒否時の手動RAW保護、サービス利用不可・応答timeout、split、次sessionを確認。
- RAWサービス実結合、同一directoryでのMCAP/RAW生成、画像workerの実入力・周期・headerは未確認。

Tensor timing: portable ROS-stub serialization/JSON/integer mapping/epoch/sequence checks passed.
check_configuration.py passed. No actual ROS publication, SDK RAW shift extraction, GPU or sync validation.
Run test/check_timing.py to repeat the portable check; stubs do not establish ROS integration.

## evt3_runs opt-in追加（2026-10-25）

- Evt3RunsHistogram単体: 境界、sum pooling、signed/separate、polarity/time layout、
  正規化、空窓、保持期限、範囲外、局所逆行、reset、奇数byte拒否を確認。
- upstream event_camera_codecsの公式ヘッダーを一時取得して実際にコンパイルした
  evt3_runs_codec_equivalence_testが成功。初期HIGH/LOW分割、ベクトル、trigger、
  padding、24bit rollover、2/14/65536 byte packet境界、4つのpolarity/layout条件を照合。
- 全RAW（53.125443秒、1,345,943,605 CD）の13,272窓を、公式decoder→scalar histogramと
  evt3_runsで照合。全FP32 Tensorをmemcmpで比較し一致。digest=16274568228255728515。
  scalar histogram側は同じring/snapshot coreを使うため、この照合はdecoder/run集計の
  差分を検出する試験。ringの独立した期待値は単体試験、GPU kernelとの一致は下記試験。
- 6 profileのparameter網羅、launch構文、既存TensorTiming stub試験が成功。
- 既存event_tensor_cuda_backend_testにCPU直接集計との全要素比較を追加。
  CUDAがないMacでは実行していない。targetでcolcon testを実施すること。

取得したreferenceはupstream master。対象Jetsonに導入済みcodecのversionは未確認。
colcon testはその環境のinstalled event_camera_codecsを使って互換性を再確認する。
MacにはROS/colcon/nvccがなく、node全体のROS/SDK/CUDA build、live USB、
TensorRT結合、Orin throughput/deadline、実際の出版header/epochは未検証。
算法の一致は同一sensor windowの条件であり、wall timerで出版する回数や
queue欠落・resetの有無が異なる実行同士で同じTensor系列になるという保証ではない。

対象環境での確認:
1. 通常通りbuild/sourceしてcolcon test --packages-select openeb_ros2。
2. tensor_cuda_async_runs.yamlを指定して起動し、診断event_aggregation=evt3_runs、
   tensor_upload=full_cpu_snapshotを確認。通常profileはeventsのまま。
3. 同じRAWをbuild/openeb_ros2/evt3_runs_codec_equivalence_test --rawで再照合。
4. event stream/epoch/windowで比較し、TensorRT併走時のqueue drop、decode busy、
   snapshot期限、CPU→GPU転送を測定する。試験は制御出力を伴わない条件で開始する。

## evt3_runs_gpu（2026-10-25）

Macでhistogram_bin_updates_test、evt3_runs_histogram_test、公式codecとの合成比較が成功。
全RAWのCPU参照・直接集計・dirty-bin再構成CPU模擬を比較し、13,272 Tensorがmemcmp/全要素一致。
有効event 1,345,943,605、digest 16274568228255728515。bin転送対象4,065,177,600 byteに対して
全Tensor転送対象20,385,792,000 byte。USB/実GPU転送帯域の測定ではない。
7 profileの設定網羅、launch構文、TensorTimingのstub試験を確認。
CUDA用histogram_bin_cuda_testとhistogram_bin_cuda_benchmarkを追加したが、Macでは未実行。
GPU版のROS/SDK/CUDA build、USB camera、GPU一致、推論併走・Orin性能は未確認。
CPU模擬の成功をCUDA kernel・転送寿命の実GPU検証と混同しない。

## ノートPC用独立ベンチマーク（2026-10-25）

tools/cuda_benchmarkからROS/SDK依存なしに本番CUDA backendをbuildできる構成を追加。
CPU-only build、CPU histogram/更新bin、runnerのschema/失敗値・dry-run検証をMacで確認。
イベント量0.25/2.5/25/50 M events/s相当とuniform/hotspotを比較するrunnerを追加。
集計stage内のCPU集計・転送・GPU完了までを比較し、RAW decodeやevent配列生成は除外する。
CUDA compiler、CUDA build、GPUでの一致・速度はこのMacでは未検証。

## RAW-to-CUDA比較の準備

`tools/cuda_benchmark`にopt-in `OPENEB_BENCH_RAW`を追加。
公式codecの外部headerを明示し、production CUDA backendと直接集計coreを使用。
RAW読込み・indexは計測外、同一packetを既存events/new full/new binsへ入力。
decode採用数・drop数・clock・GPU Tensor全要素を比較し、GPU completion込みのwall timeを保存。
異なるdecoderのOOB統計は直接比較しない（公式decoderはcallback前にOOBを除外）。
CPU testで14/64/65536byte境界、vector encoding、OOB、小時刻逆転、ringとdirty mirrorを検証。
手元RAW冒頭16MiBもCPU一致。portable CTestとrunner検査、host C++構文検査を実施。
このMacではnvcc/GPUを使っていない。CUDA compile、GPU一致と高速化は未確認。
通常async nodeのworker scheduling/ROS/USB/実車の性能保証とは分離する。
