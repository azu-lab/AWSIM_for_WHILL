# AWSIM for WHILL（日本語）

電動車椅子 **WHILL Model CR** を [AWSIM](https://github.com/autowarefoundation/AWSIM) 上で動かすシミュレーション環境です。
実機に合わせて物理パラメータを最適化した差動二輪の車両モデル、`geometry_msgs/Twist` による ROS 2（Humble）制御、真値オドメトリ、インフラ LiDAR、障害物のある廊下シーン、パラメータ最適化ツール（Optuna）、DreamerV3 による強化学習環境を含みます。

AWSIM（TIER IV / Autoware Foundation）のコミット `fa1dc65`（v2.0.0 と v2.0.1 の間）からの派生物です。
WHILL Model CR の元の 3D モデルは**含まれません**。車両の見た目は Unity のプリミティブで作った代替モデルで、物理は元のプレハブと同一です。

英語版 [README.md](README.md) が正本です。手順の詳細やコマンドの説明は英語版と `docs/` を参照してください。実際に何をどう確認したかは [docs/TEST_REPORT.md](docs/TEST_REPORT.md) にあります。

## 機能と状態

| 機能 | 状態 |
|---|---|
| 差動二輪（Twist → 逆運動学 → 左右輪 PID）、最適化済み物理パラメータ | 確認済み（前進・後退・旋回・停止） |
| 公開用代替 3D モデル（物理は元と同一） | 確認済み（差し替え前後のパラメータ書き出しで一致） |
| ROS 2：cmd_vel、ギア、`/ground_truth/odom`、車輪速度、カメラ、車載 LiDAR、`/clock` | 確認済み（周期を実測） |
| インフラ LiDAR（Ouster OS1-64、`config/infra_lidar.yaml`）と `/tf_static` | 確認済み（WHILL を捉える） |
| 障害物のある廊下、衝突検知、エピソードリセット、開始姿勢・ゴールの ROS 2 指定 | 確認済み |
| 車両パラメータ最適化（Optuna、実機 rosbag） | パイプラインは確認済み（実機データは同梱していない） |
| DreamerV3（カメラ）障害物回避の学習・チェックポイント・再開 | 短時間学習（400〜500 ステップ）のみ確認。学習が進むかは未検証 |
| ns-3 による通信シミュレーション | 設計のみ（[docs/NS3_INTEGRATION.md](docs/NS3_INTEGRATION.md)） |
| 複数台の WHILL | 非対応（[docs/ROS2_INTERFACE.md §5](docs/ROS2_INTERFACE.md)） |

## 動作環境（検証した構成）

Ubuntu 22.04.5 LTS／NVIDIA RTX 5090（ドライバ 580.178.04）／Unity **6000.0.34f1**（Unity Hub でサインイン済みのライセンスが必要）／ROS 2 Humble + CycloneDDS（Docker イメージで提供）／Docker 29.4 + Compose v5 + NVIDIA Container Toolkit。

## 導入から走行まで

```bash
git clone --recurse-submodules https://github.com/shoumexi-03/AWSIM_for_WHILL.git
cd AWSIM_for_WHILL
sudo sysctl -w net.core.rmem_max=2147483647          # 大きな点群用（推奨）
docker compose -f docker/compose.yaml build ros2
docker compose -f docker/compose.yaml build dreamer  # 強化学習を使う場合のみ（ros2 の後に）
scripts/build_player.sh                              # 初回 10〜30 分（Unity Editor は閉じておく）

scripts/run_awsim.sh                                          # 端末 1：AWSIM
scripts/ros2_shell.sh python3 ros2_examples/drive_test.py    # 端末 2：走行テスト
```

手で動かす場合はギアを一度 DRIVE にしてから `cmd_vel` を送ります。publisher の durability は TRANSIENT_LOCAL が必須です（`ros2 topic pub` は既定でそうなっているが、rclpy / rclcpp の既定は VOLATILE で届かない）。

```bash
scripts/ros2_shell.sh
ros2 topic pub --once --qos-reliability reliable --qos-durability transient_local \
  /control/command/gear_cmd autoware_vehicle_msgs/msg/GearCommand "{command: 2}"
ros2 topic pub -r 10 --qos-reliability reliable --qos-durability transient_local \
  /whill/controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.2}}"
```

Unity Editor で開く場合は、**ROS 2 を source していないシェル**から Unity Hub を起動してください（Ros2ForUnity は ROS 2 のライブラリを同梱しており、`/opt/ros/humble` と混ざると落ちる・トピックが出ない）。

## 主なドキュメント

| 内容 | 文書 |
|---|---|
| 上流 AWSIM との差分（ベースコミット、追加・変更・無効化） | [docs/UPSTREAM_DIFF.md](docs/UPSTREAM_DIFF.md) |
| ライセンスと除外したアセット | [docs/ASSETS_AND_LICENSES.md](docs/ASSETS_AND_LICENSES.md) |
| 車両モデル・物理パラメータ・代替モデル | [docs/VEHICLE_MODEL.md](docs/VEHICLE_MODEL.md) |
| ROS 2 インタフェース（トピック・QoS・TF・時刻） | [docs/ROS2_INTERFACE.md](docs/ROS2_INTERFACE.md)、[docs/ROS2_EXAMPLES.md](docs/ROS2_EXAMPLES.md) |
| パラメータ最適化 | [docs/PARAMETER_OPTIMIZATION.md](docs/PARAMETER_OPTIMIZATION.md) |
| DreamerV3 学習 | [docs/DREAMER_TRAINING.md](docs/DREAMER_TRAINING.md) |
| ns-3 連携案 | [docs/NS3_INTEGRATION.md](docs/NS3_INTEGRATION.md) |
| テスト結果 | [docs/TEST_REPORT.md](docs/TEST_REPORT.md) |

## 制約

- 1 つの AWSIM で WHILL は 1 台（トピック名が固定、最適化ブリッジのポートも固定）。
- キーボードでは WHILL は走らない（ROS 2 か最適化ブリッジ経由で操作）。
- 車両パラメータの既定値は最適化の最良値。卒業研究のシーン（DreamerV3 の学習を含む）とは横滑り係数 2 つが異なり、当時の状態は `config/vehicle/whill_model_cr_soturon_scene.yaml` で再現できる（[docs/VEHICLE_MODEL.md §4](docs/VEHICLE_MODEL.md)）。
- 実機 WHILL の rosbag と元研究の最適化記録は公開していない。最適化の再実行には自前の実機データが要る。
- DreamerV3 は短時間の学習のみ確認。compute capability 12 以上の GPU（RTX 50 系）では float16 ではなく float32 で計算する。
- 上流の新宿マップは別配布（CC BY-NC）で同梱していない。

## ライセンス

[AWSIM License](LICENSE) に従います。コード（`*.cs` など）は **Apache 2.0**、AWSIM のアセットは **CC BY-NC**（非商用）、AWSIM のドキュメントは再公開不可（同梱していない）。本プロジェクトで追加したコードとファイルは Apache 2.0、DreamerV3（submodule）は MIT です。詳細は [docs/ASSETS_AND_LICENSES.md](docs/ASSETS_AND_LICENSES.md) と [NOTICE](NOTICE)。

## 謝辞

TIER IV, Inc. と Autoware Foundation の [AWSIM](https://github.com/autowarefoundation/AWSIM)、Robotec.ai の [Robotec GPU Lidar](https://github.com/RobotecAI/RobotecGPULidar) と [ROS2 For Unity](https://github.com/RobotecAI/ros2-for-unity)、Danijar Hafner 氏の [DreamerV3](https://github.com/danijar/dreamerv3)、[Optuna](https://optuna.org/)、[ROS 2](https://docs.ros.org/en/humble/)、[Eclipse Cyclone DDS](https://cyclonedds.io/) に感謝します。
WHILL および Model CR は WHILL 株式会社の製品であり、本プロジェクトは同社とは関係がなく、同社の承認を受けたものでもありません。
