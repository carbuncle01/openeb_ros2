# kart branchの責務と転送契約

2026-10-09。sensor driverとイベント表現までをopeneb_ros2が所有する。
学習モデル、TensorRT engine、車両制御・モード管理はこのrepoに含めない。

- OpenEB取得 -> 所有するRAW byte buffer -> decode -> representationを同一プロセスで接続する。
  EventPacket構造体は内部のencoding/geometry/timestamp契約にも使うが、ROS packet topicを処理の入力にしない。
- SDK bufferの寿命を越えて参照しない。取得byte列は所有するbufferへ1回コピーする。
  direct経路はSDK callbackごとに渡し、topic用packet_duration_usによる追加集約はしない。
- packet_publish_enabledは利用者が負荷に応じて動的に切り替える。自動負荷判定は導入しない。
  falseでもdirect入力・tensor生成・native RAW記録は継続する。
  trueかつ購読者がいる場合だけ監視packetをコピーしてevents_rawへpublishする。
- cpu / cuda / cuda_asyncはrepresentation方式。共通のevent_camera_codecsでEVT3をデコードする。
  asyncだけがdecodeとGPU workerを分離し、bounded queue、古いbatchの破棄、decode buffer再利用を持つ。
  cpu/従来cudaはSDK callback内で処理するため、遅い処理が取得callbackを占有し得る。
- 出力はIsaac ROS 5.0 / ROS 2 Lyricalのisaac_ros_tensor_msgs/msg/TensorList。
  tensor_msgs/msg/ExperimentalTensorのdataをCUDA buffer backendで確保する。
  CUDA snapshotは出力bufferに直接書く。CPU表現はH2Dを伴う。
  転送zero-copyは取得から全工程でコピーがないという意味ではない。
- NCHW FP32、N=1、既定[1,20,120,160]。channel/bin順・正規化・window/strideは学習契約と一致させる。
- WriteHandleの寿命内でCUDA処理をenqueueし、破棄による完了event記録後にpublishする。
  subscriberはfrom_input_bufferを使い、handle/messageの寿命を守る。
  GPU共有条件が満たされない場合のCPU fallbackと実測の転送挙動を区別する。
- 同一process用にtensor_containerのcomponent managerを提供する。
  TensorRTはそのmanagerへloadできる。別process共有もCUDA backendの対応環境では可能。
- 設定の基準値はconfig/、実運用の正本は利用側bringup/configに置く。
  backend変更は再起動。packet配信と停止中のRAW保存先を実行中に変更可。
- JetPilot固有BagRequestを依存から外す。RAW保存は標準Triggerサービスと自動分割設定を使う。
  recording sessionとの連携は利用側adapterの責務とする。kart bag managerは
  保存先parameterの設定成功後にTrigger STARTを呼び、同じdirectoryへnative RAWを保存する。

## 出典と移植元

JetPilot commit 456c0bfc83455a92a6811e302e4a0522f992c08bの
jetpilot_e2e_inference/{EventTensorEncoderNode,AsyncEventTensorPreprocessorNode,EventTensorCudaBackend}
を移植し、名前空間、direct入力、GPU buffer出力、並行実行の保護を変更した。
旧NitrosMemoryPoolの出力pool設定は引き継がず、CUDA buffer backendのallocatorへ任せる。
CPU側のstaging_buffer_countとasyncのdecode buffer poolは独立して維持する。

公式API確認日: 2026-10-09。ローカルapt導入version・GPU実行は未確認。

- https://nvidia-isaac-ros.github.io/concepts/rosidl_buffer/cuda_buffer_backend.html
- https://nvidia-isaac-ros.github.io/concepts/rosidl_buffer/nitros_migration.html
- https://github.com/ros2/rosidl_buffer_backends/tree/main/cuda_buffer_backend
- https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/isaac_ros_tensor_msgs/msg/TensorList.msg


## 可視化と記録（2026-10-09追加）

- direct入力から可視化専用workerへ分岐する。購読者なしでは追加decodeを行わない。
  packet配信の許可とは独立し、可視化nodeからpacketを再publishしない。
- 白背景、ON赤、OFF青のbgr8画像を25 Hzで生成。windowからイベントが抜ければ白へ戻る。
  同pixelの極性が重なる場合は正規化強度の大きい方、同値はON赤。
- 可視化queueにpacket数・byte数・年齢の上限を持たせ、破棄後にdecoderをresetする。
  画像headerはpacket receiptへの推定対応であり、hardware synchronizationの主張には使わない。
- RAW保存先変更は停止中だけ。START/STOP/SPLITの失敗をサービス応答へ返す。
  splitには独立した連番を付け、driver timerなしのmanager splitでもファイル名を重複させない。

## 学習用の時刻メタデータ

3方式ともTensor出版後に`/event_camera/tensor_timing`（std_msgs/msg/String、JSON v1）を
出版する。reliable/volatile/depth 512、Tensorと同周期（既定250 Hz）、購読者あり時のみ。
追加parameterなし。GPU Tensor本体は記録不要。OpenEB用bag profileはこのtopicを記録する。
JSONはstream_id、sequence、sensor_epoch、tensor_stamp_ros_ns、generated_ros_ns、
window_start/end_sensor_us、sensor_to_ros_offset_ns、window_start/end_ros_ns、
latest_event_sensor_us、tensor_name、frame_id、shapeを含む。
窓は半開区間[start,end)。header stampと窓終端は同一とは限らない。
sequenceは成功したTensor出版ごとに増加、リセット時sensor_epochが増える。
メタデータとTensorの出版は別メッセージなのでatomicではなく、欠落はsequenceで確認する。

offsetは毎snapshotのpacket受信ROS時刻−最新イベントセンサ時刻から得る推定値。
USB遅延/queue遅延とclock変動が入り、hardware sync精度を保証しない。
RAWの0始まりはreaderのtime shiftingに依存する。recording_start_ros_timeを
RAWの0秒へ直接足さない。HALのget_timestamp_shift()でファイルごとのshiftを取得し、
`ros_ns=(raw_relative_us+raw_sensor_shift_us)*1000+sensor_to_ros_offset_ns`で対応付ける。
RAW splitごとにshiftを取得し、timestamp reset/rolloverはepoch別に扱う。
SDK実装: https://github.com/prophesee-ai/openeb/blob/5.2.0/hal/cpp/include/metavision/hal/facilities/i_events_stream_decoder.h

## cuda_async opt-in CPU直接集計（2026-10-25）

`event_aggregation=events`を既定として維持。`evt3_runs`では所有RAW packetをdecode workerで
直接CPU binへ集計し、snapshot workerが完成TensorをH2Dする。イベント配列を介さない。
両workerは同じringをmutexで保護し、転送用snapshotの寿命はCUDA完了まで維持する。
seq不連続ではdecoder/ringをresetし、異なる時刻epochのbinを混在させない。
更新binだけのGPU転送や取得SDK bufferのzero-copyはこの方式の契約に含めない。
時刻処理はevent_camera_codecs互換を試験し、同じsensor windowでTensorを比較する。

## evt3_runs_gpu（2026-10-25）

直接集計の追加選択肢。CPUの2-window ringからdirty slotの絶対値を所有bufferへ取得し、
固定pinned staging経由でGPU ringへ上書きする。epoch/reset/clearと再送でcountを重複させない。
GPUは既存snapshot kernelを使う。既定eventsとCPU版evt3_runsの意味は維持する。
更新buffer、pinned staging、GPU ringはそれぞれの寿命を持ち、CUDA eventで再利用を保護する。
失敗時は現在の全CPU slotを再送する。小さいCD時刻逆行の破棄閾値は既存parameterを使う。

## Borrowed histogram viewによるCPUコピー削減の評価

新APIはCPU ringのdirty planeを借用し、owned pinned stagingへ直接退避する。
evt3_runs_gpu_directは独立benchmarkで比較後、ROSのstartup選択にも追加。既定eventsは維持する。
借用期間はpeekからupdate_bins_direct完了まで。外部serializationでhistogramの変更を禁止する。
GPU転送中のCPU ring参照は認めず、CUDA eventでpinned slotを保護し、成功時のみdirtyをackする。
GPU転送量・Tensor順序・dtype・正規化・窓定義は従来と同一。障害後は全dirty再送で復旧する。
ROSではpeekからupdate_bins_directまで同じmutexを保持し、成功後にackする。liveのmutex保持時間は実機で評価する。

## 圧縮runをGPUで直接histogramへ加算する評価経路

CPUはEVT3状態と座標範囲・時刻逆転を検査し、同時刻・YのADDR_X runまたはvector wordを記述する。
pixel histogramは作らず、圧縮RAW＋24byte descriptorをowned pinned/device stagingへコピーする。
GPUは展開済みイベント配列を作らずrolling ringへ直接atomic加算し、既存snapshot kernelを使う。
stagingは2 slot・事前確保・CUDA eventで保護する。入力CPU bufferの寿命はupdate呼出しの終了まで。
GPU上のring保持範囲、half-open窓、座標floor、極性順、normalizationは現方式と同一。
新旧mode混在には明示resetが必要。大きな逆転はscannerのepochを更新し、GPU ringを初期化する。
独立benchmarkのevt3_runs_grouped_gpuで一致と性能を確認後、ROSのstartup選択にも追加。


## ROS grouped workerと選択override

受信callbackは有界RAW queueへの所有権移譲のみとし、grouped専用workerがparser・CUDA・snapshot scheduleを所有する。
CPU run scannerはpacket/run上限を持ち、固定容量の2 pinned/device slotを起動時確保する。
packetはupdate_grouped_raw完了まで保持し、以後はbackendが所有するpinnedコピーとCUDA eventで寿命を守る。
sequence gap/reorder、queue overflow、stale、上限、geometry、decode/CUDA errorで履歴を破棄して再取得する。
古い履歴と新しいepochを混在させず、4ms以内の時刻逆転はevent dropとして継続する。
CLI/TUIのevs_aggregationはselectorのevent_aggregationへ渡し、backend=cuda_async時だけchildへ明示overrideする。
shapeやnormalizationをprofile選択で置換せず、モデル由来Tensor YAMLとの併用を保持する。
