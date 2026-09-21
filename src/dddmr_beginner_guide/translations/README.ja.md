# DDDMR 初心者ガイド

このガイドは DDDMR ナビゲーションシステムの入門チュートリアルです。Gazebo 四足ロボットシミュレーション例と実機ロボット展開ガイドを含み、素早く始めて 3D ナビゲーションを探求し楽しむことができます。

## 目次

| # | セクション | 説明 |
|:-:|:--------|:------------|
| 1 | [Gazebo シミュレーションナビゲーション](#-dddmr-navigation-with-gazebo) | 四足ロボットシミュレーションデモ |
| 2 | [実機ロボットナビゲーション](#-start-dddmr-navigation-with-a-real-robot) | 実機ロボットへの展開（車輪型、四足、ヒューマノイドなど） |

## 🖥️ ソフトウェア要件
- **Ubuntu 22.04**（22.04 でテスト済み、24.04 をサポート予定）
- **Docker** — [Docker のインストール](https://docs.docker.com/engine/install/)

## ✨ DDDMR Navigation with Gazebo

このデモでは、Gazebo で四足ロボットと DDDMR ナビゲーションスタックを実行する方法を説明します。
- 環境を準備するために必要なイメージをビルドします。
- 2つのターミナルでシステムを実行します — 1つは Gazebo ワールド用、もう1つはナビゲーションスタック用です。

### 1. Docker イメージの作成

リポジトリをクローンし、./build.bash を実行します。**`x64_gz`** を選択してください。ナビゲーションと Gazebo に必要なすべてのコンポーネントが含まれています。

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

### 2. ナビゲーションマップのダウンロード

Gazebo で dddmr_navigation を実行するには、デモ用ナビゲーションマップ（12.3MB）をダウンロードする必要があります。

```bash
cd ~ && mkdir dddmr_bags
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./download_files.bash
```

### 3. デモ環境の準備

#### 2つの Docker コンテナを作成（Gazebo とナビゲーション）

> [!IMPORTANT] 
> 以下のコマンドは、ビルドしたイメージを使用して2つのインタラクティブな Docker コンテナを起動します。デモ環境を準備するために**2つの別々のターミナル**を開いてください。

#### 🖥️ ターミナル 1：Go2 の Gazebo シミュレーションを起動

最初のターミナルで、[Gazebo ガイド — Go2](../GAZEBO_GUIDE.md#go2--quadruped) の 2 つの手順に従ってシミュレーション用イメージをビルドし、Gazebo コンテナを起動します。

Go2 シミュレーションが起動したら、このターミナルを開いたままにして、ここに戻り、以下の**ターミナル 2** の手順に進んでください。

#### 🖥️ ターミナル 2（ナビゲーションシステムコンテナの作成）

- ##### ステップ 1（ホスト上で）：ナビゲーションシステムコンテナを作成
```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./run_x64_navigation.bash
```

- ##### ステップ 2（ナビゲーションコンテナ内で）：ビルドと起動
```bash
cd dddmr_navigation/ && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash && ros2 launch p2p_move_base go2_localization.launch
```

### 4. デモの実行

- Gazebo デモでは、マップはすでに位置合わせされているため、自分でマッピングしない限り初期ポーズを設定する必要はありません
- RViz で目標点（3D Goal Pose）を指定すると、ロボットが目標地点に移動します

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/give_goal_in_demo_.png" width="920" height="460"/>
</p>

### 5. 既知の問題

> [!WARNING]
> 以下は現在観察されている動作であり、調査中で今後のアップデートで修正される予定です。
> - Gazebo：斜面で時々スリップする
> - マッピング：重複した床レイヤー

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
