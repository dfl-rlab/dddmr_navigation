# DDDMR 初心者ガイド

このガイドは DDDMR ナビゲーションの入門チュートリアルです。Lite3 の Gazebo シミュレーションと実機への導入手順で、3D ナビゲーションを始められます。

## 目次

| # | セクション | 説明 |
|:-:|:--------|:------------|
| 1 | [Gazebo シミュレーションナビゲーション](#-dddmr-navigation-with-gazebo) | Lite3 RL 四足ロボットのシミュレーションデモ |
| 2 | [実機ロボットナビゲーション](#-start-dddmr-navigation-with-a-real-robot) | 実機ロボットへの展開（車輪型、四足、ヒューマノイドなど） |

## 🖥️ ソフトウェア要件
- **Ubuntu 22.04**（テスト済み。Ubuntu 24.04 ホストでも Humble コンテナを実行できます）。
- **Docker** — [Docker のインストール](https://docs.docker.com/engine/install/)
- GPU シミュレーションと TensorRT ナビゲーションデモ用の **NVIDIA GPU および NVIDIA Container Toolkit**。

## ✨ DDDMR Navigation with Gazebo

ドーム型 LiDAR を搭載した **Lite3 RL 四足ロボット**で、複数階の Gazebo ワールド内をナビゲーションします。このデモでは、次の 2 つの独立したイメージとコンテナを使います。

| ターミナル | 用途 | イメージ | コンテナ |
|:---------|:--------|:------|:----------|
| 1 | Lite3 Gazebo シミュレーション | `dddmr_simulation:rl_lite3` | `rl_lite3` |
| 2 | TensorRT を使用した自己位置推定と 3D ナビゲーション | `dddmr:humble-cuda` | `dddmr_humble_cuda_dev` |

以下のコマンドでは、ナビゲーションリポジトリを `~/dddmr_navigation`、シミュレータを `~/dfl_mobilerobot_simulator` に配置します。

### 1. Docker イメージの作成

まずナビゲーションリポジトリをクローンします。以下のどちらの方法でも、この手順が必要です。

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
```

次の**いずれか 1 つ**の方法でナビゲーションイメージを取得します。

#### 方法 A：ローカルでビルド

ビルドスクリプトを実行し、**`x64`** を選び、CUDA の確認に **`Y`** と回答します。CUDA と TensorRT の依存関係を含む `dddmr:humble-cuda` が作成されます。

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./build.bash
```

#### 方法 B：公開イメージを取得

ビルド済みイメージが公開されています。取得後、`run_x64_gpu.bash` が使用するローカルタグを付けます。

```bash
docker pull tsengapola/dddmr:humble-cuda
docker tag tsengapola/dddmr:humble-cuda dddmr:humble-cuda
```

どちらの方法でも、次に下記の地図をダウンロードします。ナビゲーションワークスペースはターミナル 2 のコンテナ内でコンパイルします。Lite3 シミュレーション用イメージは、下記の**ターミナル 1** の手順で別途ビルドします。

### 2. ナビゲーション用地図のダウンロード

Lite3 ワールド用の地図 `stair_2_map.zip` をダウンロードして展開します。確認には **`Y`** と回答してください。

```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide
./download_files.bash
```

スクリプトは `~/dddmr_bags` を作成し、地図を `~/dddmr_bags/stair_2_map` に展開します。起動スクリプトは、この地図用ディレクトリをナビゲーションコンテナ内の `/root/dddmr_bags` にマウントします。

### 3. デモ環境の準備

シミュレータ用とナビゲーション用に**別々のターミナルを 2 つ**開きます。両方のコンテナで **`ROS_DOMAIN_ID=22`** を使用してください。

#### 🖥️ ターミナル 1：Lite3 Gazebo シミュレーションの起動

[Gazebo ガイド — Lite3](../GAZEBO_GUIDE.md#lite3--rl-quadruped) に従い、`rl_lite3` ブランチのクローン、サブモジュールの初期化、イメージのビルドを行い、`./rl_lite3_gpu.bash` を実行します。

ロボットが地面に降りたら、シミュレータのターミナルにフォーカスを合わせ、**`r` → `0` → `1` → `n`** の順に押します。姿勢のリセット、起立、RL コントローラの開始、ナビゲーション速度指令の有効化を行います。**キー操作の間は 1 秒以上待ち、`0` の後は起立動作が完了するまで待ってください。**

| キー | 動作 |
|:----|:-------|
| `r` | ロボットをワールド内の初期姿勢に戻します。 |
| `0` | 起立して移動の準備をします。 |
| `1` | RL 歩行ポリシーを有効にします。 |
| `n` | ナビゲーションから制御できるよう、`/cmd_vel` 入力を有効にします。 |

シミュレーションのターミナルを開いたまま、ターミナル 2 の手順に進みます。

#### 🖥️ ターミナル 2：ナビゲーションコンテナの作成

##### 手順 1（ホスト側）：CUDA コンテナの起動

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./run_x64_gpu.bash
```

`ROS_DOMAIN_ID` を聞かれたら **`22`** と入力します。`dddmr:humble-cuda` から `dddmr_humble_cuda_dev` コンテナが作成され、リポジトリが `/root/dddmr_navigation` にマウントされます。

##### 手順 2（ナビゲーションコンテナ内）：TensorRT を有効にしてビルド

```bash
cd /root/dddmr_navigation
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=22
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DTRT_ENABLED=ON -DTEST_ENABLED=ON
```

`TRT_ENABLED=ON` はナビゲーションの推論コードを有効にします。新しくクローンした場合は、同梱モデルと変換スクリプトから TensorRT エンジンも生成してください。

```bash
cd /root/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/engine
bash onnx2trt.bash
cd /root/dddmr_navigation
```

##### 手順 3（ナビゲーションコンテナ内）：自己位置推定とナビゲーションの起動

```bash
source install/setup.bash
export ROS_DOMAIN_ID=22
ros2 launch p2p_move_base lite3_localization_launch.py
```

### 4. デモの実行

1. ターミナル 1 の Lite3 が RL ポリシーモードで、`/cmd_vel` 入力が有効であることを確認します。
2. RViz でライブ点群と地図が重なることを確認します。デモ設定にはこのワールド用の初期姿勢が含まれます。ロボットを別の場所に移動またはリセットした場合は、**3D Pose Estimate** で位置合わせをします。
3. RViz の **3D Goal Pose** を選び、到達可能な地面上の位置を指定します。Lite3 はナビゲーション指令に従って目標へ移動します。

<p align="center">
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/3d_nav_rl_quadruped_deep_learning.png?raw=true" alt="Gazebo と RViz で深層学習を使用する Lite3 RL 四足ロボットの 3D ナビゲーション" width="920" />
</p>

---

<p align="center">
  <b>━━━━━━━━━━ 🤖 実機ロボットチュートリアル 以下 🤖 ━━━━━━━━━━</b>
</p>

---

## ✨ Start DDDMR Navigation with a Real Robot

このガイドでは、実機ロボットでの **3D ナビゲーション**のセットアップ方法を説明します — マッピングから自律ナビゲーションまで。

DDDMR は車輪型ロボット、四足ロボット、ヒューマノイドなどに 3D ナビゲーション機能を提供します。ロボットを動かしましょう！

### 1. Docker イメージの作成

リポジトリをクローンし、./build.bash を実行します。プラットフォームに応じて **`x64`** または **`l4t`** を選択してください。

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

> **注意：** テストプラットフォームは Jetson Orin Nano を使用しているため、**`l4t`** を選択します。

### 2. テストプラットフォーム

このチュートリアルに素早く合わせるために、使用するハードウェア構成は以下の通りです：

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/hardware_quadruped.png" width="500" height="250"/>
</p>

| コンポーネント | モデル |
|:----------|:------|
| ロボット | 四足ロボット（Lite3） |
| LiDAR | ドーム型 LiDAR（RoboSense Airy）、45° 傾斜 |
| コンピュータ | Jetson Orin Nano |
| 電源 | LiDAR と Jetson 用外部バッテリー |
| 接続 | イーサネット（ロボット、Jetson、LiDAR がネットワークで通信） |

> **注意：** まったく同じハードウェアは必要ありません。システムが以下の要件を満たしていれば、DDDMR は動作します。

#### システム要件

| 項目 | トピック | メッセージタイプ | 説明 |
|:----:|:------|:-------------|:------------|
| 🎮 | `/cmd_vel` | `geometry_msgs/msg/Twist` | ロボットを制御する速度コマンド |
| 📡 | `/lidar_point_cloud` | `sensor_msgs/msg/PointCloud2` | 3D LiDAR ポイントクラウド（マルチレイヤー、ドーム型、ソリッドなど） |
| 📍 | `/odom` | `nav_msgs/msg/Odometry` | TF 付きオドメトリ（誤差 < 10% が望ましい） |
| 🌳 | `/tf` | `tf2_msgs/msg/TFMessage` | TF ツリー：`odom` → `base_link` → `lidar_link` |

> **注意：** 
> - DDDMR は上記の要件を満たす**車輪型ロボット、四足ロボット、ヒューマノイド**で使用できます。
> - `/cmd_vel` はロボットのモーション制御への**入力**です。
> - `/lidar_point_cloud`、`/odom`、`/tf` はロボットの**出力**で、DDDMR がマッピングと位置推定に使用します。

#### TF ツリー構造

LiDAR トピックの `frame_id` が TF ツリーと一致していることを確認してください：

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/tf_requirement.png" width="150" height="250"/>
</p>

#### TF 設定例

以下は四足ロボットを例にした TF 設定です。LiDAR は `base_link` から `x=0.2m`、`z=0.22m` の位置に取り付けられ、45° 下向きに傾斜しています：

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/setup_lidar_testing_quadruped.png" width="400" height="250"/>
</p>

```xml
<!--- TF: x y z yaw pitch roll -->
<node pkg="tf2_ros" exec="static_transform_publisher" name="sensor2baselink" 
      args="0.2 0.0 0.22 0.0 0.785 0.0 base_link lidar" />
```

> **注意：** 45° ≈ 0.785 ラジアン。下向きに傾斜。

#### 👉 上級（オプション）

| 機能 | 説明 |
|:--------|:------------|
| [3D オドメトリ](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_odom_3d) | 不整地でのより良い位置推定とマッピング |

---

### 3. 🚀 マッピング + ナビゲーションの実行

DDDMR は3つのナビゲーションワークフローをサポートしています：

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/dddmr_nav_workflows.png" width="600" height="450"/>
</p>

> **注意：** このチュートリアルは **② オンラインマッピング + ナビゲーション** ワークフローに従います。

---

### 3.1 🗺️ マッピング

開始する前に、ロボットが必要なトピックをパブリッシュしていることを確認してください：

- オドメトリトピック
- LiDAR ポイントクラウドトピック
- TF（`odom` → `base_link` → `lidar_link`）

次にマッピングを起動します：

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_mapping.launch
```

起動後、次のような RViz インターフェースが表示されます：

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/mapping_realrobot_rviz.png" width="800" height="500"/>
</p>

ロボットを周囲に動かすと、以下が確認できます：
- **キーフレーム** — ロボットの移動に伴って増加するはずです
- **特徴点** — 環境から抽出された特徴
- **地面点** — 検出された地面表面
- **2D 投影** — シーンと LiDAR FOV の理解に役立ちます

#### マップの保存

エリアのマッピングが完了したら、新しいターミナルを開いて実行します：

```bash
ros2 service call /save_mapped_point_cloud std_srvs/srv/Empty
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/map_save_.png" width="500" height="250"/>
</p>

ターミナルに `Create dir:` が表示されたら、マップが保存されたことを意味します。マッピングが完了したら、マッピングノードを終了できます。

マップは `/tmp` に保存されます。フォルダを `/root/dddmr_bags` に移動できます：

```bash
mv /tmp/2026_03_28_19_25_15/ /root/dddmr_bags/
```

> **注意：** フォルダ名 `2026_03_28_19_25_15` は例です。実際のフォルダ名はマッピング実行時刻によって異なります。

---

### 3.2 📍 位置推定 + ナビゲーション

まず、設定ファイルを開いてマップパスを設定します：

📄 `config/airy_tilt45_navigation.yaml`

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/robot_map_location.png" width="500" height="500"/>
</p>

`pose_graph_dir` をマップフォルダに変更します：

```yaml
pose_graph_dir: "/root/dddmr_bags/2026_03_28_19_25_15"
```

> **注意：** `2026_03_28_19_25_15` を実際のマップフォルダ名に置き換えてください。

次に位置推定とナビゲーションを起動します：

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_navigation.launch
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/LOC_NAV_realrobot_rviz.png" width="800" height="500"/>
</p>

##### ステップ 1：初期ポーズの設定

RViz ツールバーの **「3D Pose Estimate」** をクリックし、ロボットの実際の位置と一致する地面点を選択します。

初期ポーズが正しければ、観測されたポイントクラウドがマップの特徴とよく重なっているのが見えます。

##### ステップ 2：ゴールの送信

初期化が完了したら、**「3D Goal Pose」** をクリックし、地面点をゴールとして選択します。

ロボットがゴールに向かって移動を開始します。
