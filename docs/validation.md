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
