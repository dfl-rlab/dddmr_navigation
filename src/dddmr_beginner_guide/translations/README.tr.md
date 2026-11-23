# DDDMR Başlangıç Kılavuzu

Bu kılavuz DDDMR Navigation sistemine giriş sunar. Lite3 Gazebo simülasyonu ve gerçek robot kurulum adımlarıyla 3D navigasyonu hızla keşfetmeye başlayabilirsiniz.

## İçindekiler

| # | Bölüm | Açıklama |
|:-:|:--------|:------------|
| 1 | [Gazebo Simülasyon Navigasyonu](#-dddmr-navigation-with-gazebo) | RL kontrollü Lite3 dört ayaklı robot simülasyonu |
| 2 | [Gerçek Robot Navigasyonu](#-start-dddmr-navigation-with-a-real-robot) | Gerçek robotunuza dağıtım (tekerlekli, dört ayaklı, insansı vb.) |

## 🖥️ Yazılım Gereksinimleri
- **Ubuntu 22.04** (test edildi; Ubuntu 24.04 ana makinelerde de Humble konteynerleri çalıştırılabilir).
- **Docker** — [Docker Kurulumu](https://docs.docker.com/engine/install/)
- GPU simülasyonu ve TensorRT navigasyon demosu için **NVIDIA GPU ve NVIDIA Container Toolkit**.

## ✨ DDDMR Navigation with Gazebo

DDDMR Navigation sistemini **RL kontrollü Lite3 dört ayaklı robot**, kubbe tipi LiDAR ve çok seviyeli bir Gazebo dünyası ile çalıştırın. Demo iki ayrı imaj ve konteyner kullanır:

| Terminal | Amaç | İmaj | Konteyner |
|:---------|:--------|:------|:----------|
| 1 | Lite3 Gazebo simülasyonu | `dddmr_simulation:rl_lite3` | `rl_lite3` |
| 2 | TensorRT ile konum belirleme ve 3D navigasyon | `dddmr:humble-cuda` | `dddmr_humble_cuda_dev` |

Aşağıdaki komutlarda navigasyon deposu için `~/dddmr_navigation`, simülatör için `~/dfl_mobilerobot_simulator` kullanılır.

### 1. Docker imajını oluşturma

Önce navigasyon deposunu klonlayın. Aşağıdaki iki seçenek için de bu adım gereklidir:

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
```

Ardından navigasyon imajını edinmek için aşağıdaki yöntemlerden **birini** seçin.

#### Seçenek A: Yerel olarak oluşturma

Oluşturma betiğini çalıştırın, **`x64`** seçin ve CUDA sorusuna **`Y`** yanıtını verin. Böylece CUDA ve TensorRT bağımlılıklarını içeren `dddmr:humble-cuda` oluşturulur.

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./build.bash
```

#### Seçenek B: Herkese açık imajı indirme

Önceden oluşturulmuş bir imaj yayımlanmıştır. İmajı indirin ve `run_x64_gpu.bash` tarafından beklenen yerel etiketi ekleyin:

```bash
docker pull tsengapola/dddmr:humble-cuda
docker tag tsengapola/dddmr:humble-cuda dddmr:humble-cuda
```

Her iki seçeneğin ardından aşağıdaki harita indirme adımına geçin. Navigasyon çalışma alanı Terminal 2'deki konteyner içinde derlenir. Lite3 simülasyon imajı aşağıdaki **Terminal 1** adımında ayrıca oluşturulur.

### 2. Navigasyon haritasını indirme

Lite3 dünyası için `stair_2_map.zip` navigasyon haritasını indirin ve açın. Sorulduğunda **`Y`** yanıtını verin:

```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide
./download_files.bash
```

Betik `~/dddmr_bags` dizinini oluşturur ve haritayı `~/dddmr_bags/stair_2_map` içine açar. Başlatma betiği harita dizinini navigasyon konteynerinde `/root/dddmr_bags` konumuna bağlar.

### 3. Demo ortamını hazırlama

Biri simülatör, diğeri navigasyon için **iki ayrı terminal** açın. Her iki konteyner de **`ROS_DOMAIN_ID=22`** kullanmalıdır.

#### 🖥️ Terminal 1: Lite3 Gazebo simülasyonunu başlatma

[Gazebo Kılavuzu — Lite3](../GAZEBO_GUIDE.md#lite3--rl-quadruped) adımlarını izleyerek `rl_lite3` dalını klonlayın, alt modülleri başlatın, imajı oluşturun ve `./rl_lite3_gpu.bash` komutunu çalıştırın.

Robot yere indikten sonra simülatör terminaline odaklanın ve **`r` → `0` → `1` → `n`** tuşlarına sırayla basın. Bu adımlar pozu sıfırlar, robotu ayağa kaldırır, RL denetleyicisini başlatır ve navigasyon komutlarını etkinleştirir. **Tuşlar arasında en az bir saniye bekleyin; `0` sonrasında ayağa kalkma hareketinin tamamlanmasını da bekleyin.**

| Tuş | İşlem |
|:----|:-------|
| `r` | Robotu dünyadaki başlangıç pozuna sıfırlar. |
| `0` | Robotu ayağa kaldırır ve yürümeye hazırlar. |
| `1` | RL hareket politikasını etkinleştirir. |
| `n` | Navigasyonun robotu kontrol edebilmesi için `/cmd_vel` girişini etkinleştirir. |

Simülasyon terminalini açık bırakın ve Terminal 2 adımlarına geçin.

#### 🖥️ Terminal 2: Navigasyon konteynerini oluşturma

##### Adım 1 (ana makinede): CUDA konteynerini başlatma

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./run_x64_gpu.bash
```

Betik `ROS_DOMAIN_ID` değerini sorduğunda **`22`** girin. `dddmr:humble-cuda` imajıyla `dddmr_humble_cuda_dev` konteynerini oluşturur ve depoyu `/root/dddmr_navigation` konumuna bağlar.

##### Adım 2 (navigasyon konteynerinde): TensorRT ile derleme

```bash
cd /root/dddmr_navigation
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=22
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DTRT_ENABLED=ON -DTEST_ENABLED=ON
```

`TRT_ENABLED=ON`, navigasyonun çıkarım kodunu etkinleştirir. Yeni klonlanmış bir depoda, içerilen modelden dönüşüm betiğiyle TensorRT motorunu da oluşturun:

```bash
cd /root/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/engine
bash onnx2trt.bash
cd /root/dddmr_navigation
```

##### Adım 3 (navigasyon konteynerinde): Konum belirleme ve navigasyonu başlatma

```bash
source install/setup.bash
export ROS_DOMAIN_ID=22
ros2 launch p2p_move_base lite3_localization_launch.py
```

### 4. Demoyu çalıştırma

1. Terminal 1'de Lite3'ün RL politika modunda olduğunu ve `/cmd_vel` girişinin etkin olduğunu doğrulayın.
2. RViz'de canlı nokta bulutunun verilen haritayla örtüştüğünü kontrol edin. Demo yapılandırması bu dünya için bir başlangıç pozu içerir. Robot başka bir yere taşındıysa veya sıfırlandıysa **3D Pose Estimate** ile hizalayın.
3. RViz'de **3D Goal Pose** seçin ve yerde ulaşılabilir bir nokta belirleyin. Lite3, navigasyon komutlarını izleyerek hedefe ilerler.

<p align="center">
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/3d_nav_rl_quadruped_deep_learning.png?raw=true" alt="Gazebo ve RViz'de derin öğrenme ile Lite3 RL dört ayaklı robotun 3D navigasyonu" width="920" />
</p>

---

<p align="center">
  <b>━━━━━━━━━━ 🤖 Gerçek Robot Öğreticisi Aşağıda 🤖 ━━━━━━━━━━</b>
</p>

---

## ✨ Start DDDMR Navigation with a Real Robot

Bu kılavuz, gerçek robotunuzda **3D navigasyon** kurulumunda size yol gösterecektir — haritalamadan otonom navigasyona kadar.

DDDMR, tekerlekli robotlarınıza, dört ayaklı robotlarınıza, insansı robotlarınıza ve daha fazlasına 3D navigasyon getirir. Robotunuzu hareket ettirmeye başlayalım!

### 1. Docker İmajı Oluşturma

Depoyu klonlayın ve ./build.bash çalıştırın, platformunuza göre **`x64`** veya **`l4t`** seçin.

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

> **Not:** Test platformumuz Jetson Orin Nano kullandığından **`l4t`** seçiyoruz.

### 2. Test Platformu

Bu öğreticiyi hızlıca takip etmek için kullandığımız donanım yapılandırması:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/hardware_quadruped.png" width="500" height="250"/>
</p>

| Bileşen | Model |
|:----------|:------|
| Robot | Dört ayaklı robot (Lite3) |
| LiDAR | Kubbe LiDAR (RoboSense Airy), 45° eğimli |
| Bilgisayar | Jetson Orin Nano |
| Güç | LiDAR ve Jetson için harici pil |
| Bağlantı | Ethernet (robot, Jetson ve LiDAR ağ üzerinden iletişim kurar) |

> **Not:** Tam olarak aynı donanıma ihtiyacınız yok. Sisteminiz aşağıdaki gereksinimleri karşıladığı sürece DDDMR çalışacaktır.

#### Sistem Gereksinimleri

| Öğe | Konu | Mesaj Türü | Açıklama |
|:----:|:------|:-------------|:------------|
| 🎮 | `/cmd_vel` | `geometry_msgs/msg/Twist` | Robotu kontrol etmek için hız komutu |
| 📡 | `/lidar_point_cloud` | `sensor_msgs/msg/PointCloud2` | 3D LiDAR nokta bulutu (çok katmanlı, kubbe, katı vb.) |
| 📍 | `/odom` | `nav_msgs/msg/Odometry` | TF ile odometri (hata < %10 tercih edilir) |
| 🌳 | `/tf` | `tf2_msgs/msg/TFMessage` | TF ağacı: `odom` → `base_link` → `lidar_link` |

> **Not:** 
> - DDDMR, yukarıdaki gereksinimleri karşıladığı sürece **tekerlekli robotlar, dört ayaklı robotlar ve insansı robotlar** üzerinde kullanılabilir.
> - `/cmd_vel`, robot hareket kontrolü için **giriştir**.
> - `/lidar_point_cloud`, `/odom`, `/tf` robotun **çıkışlarıdır**, DDDMR tarafından haritalama ve konumlandırma için kullanılır.

#### TF Ağacı Yapısı

LiDAR konusundaki `frame_id`'nin TF ağacınızla eşleştiğinden emin olun:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/tf_requirement.png" width="150" height="250"/>
</p>

#### TF Yapılandırma Örneği

Dört ayaklı robotumuzu örnek olarak kullanan TF yapılandırması. LiDAR, `base_link`'ten `x=0.2m`, `z=0.22m` konumunda monte edilmiş ve 45° aşağı eğimlidir:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/setup_lidar_testing_quadruped.png" width="400" height="250"/>
</p>

```xml
<!--- TF: x y z yaw pitch roll -->
<node pkg="tf2_ros" exec="static_transform_publisher" name="sensor2baselink" 
      args="0.2 0.0 0.22 0.0 0.785 0.0 base_link lidar" />
```

> **Not:** 45° ≈ 0.785 radyan. Aşağı eğimli.

#### 👉 Gelişmiş (İsteğe Bağlı)

| Özellik | Açıklama |
|:--------|:------------|
| [3D Odometri](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_odom_3d) | Engebeli arazide daha iyi konumlandırma ve haritalama |

---

### 3. 🚀 Haritalama + Navigasyon Çalıştırma

DDDMR üç navigasyon iş akışını destekler:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/dddmr_nav_workflows.png" width="600" height="450"/>
</p>

> **Not:** Bu öğretici **② Çevrimiçi haritalama + Navigasyon** iş akışını takip eder.

---

### 3.1 🗺️ Haritalama

Başlamadan önce, robotunuzun gerekli konuları yayınladığından emin olun:

- Odometri konusu
- LiDAR nokta bulutu konusu
- TF (`odom` → `base_link` → `lidar_link`)

Ardından haritalmayı başlatın:

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_mapping.launch
```

Başlattıktan sonra şöyle bir RViz arayüzü görmelisiniz:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/mapping_realrobot_rviz.png" width="800" height="500"/>
</p>

Robotunuzu sürdüğünüzde fark edeceksiniz:
- **Anahtar kareler** — robot hareket ettikçe artmalıdır
- **Özellik noktaları** — ortamdan çıkarılan özellikler
- **Zemin noktaları** — algılanan zemin yüzeyi
- **2D projeksiyon** — sahneyi ve LiDAR FOV'unu anlamaya yardımcı olur

#### Haritayı Kaydetme

Alanın haritalamasını tamamladıktan sonra yeni bir terminal açın ve çalıştırın:

```bash
ros2 service call /save_mapped_point_cloud std_srvs/srv/Empty
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/map_save_.png" width="500" height="250"/>
</p>

Terminalde `Create dir:` gördüğünüzde, haritanın kaydedildiği anlamına gelir. Haritalama tamamlandıysa, haritalama düğümünü kapatabilirsiniz.

Harita `/tmp`'ye kaydedilecektir. Klasörü `/root/dddmr_bags`'e taşıyabilirsiniz:

```bash
mv /tmp/2026_03_28_19_25_15/ /root/dddmr_bags/
```

> **Not:** `2026_03_28_19_25_15` klasör adı sadece bir örnektir. Klasör adınız haritlamayı ne zaman çalıştırdığınıza göre farklı olacaktır.

---

### 3.2 📍 Konumlandırma + Navigasyon

Önce yapılandırma dosyasını açın ve harita yolunuzu ayarlayın:

📄 `config/airy_tilt45_navigation.yaml`

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/robot_map_location.png" width="500" height="500"/>
</p>

`pose_graph_dir`'i harita klasörünüze değiştirin:

```yaml
pose_graph_dir: "/root/dddmr_bags/2026_03_28_19_25_15"
```

> **Not:** `2026_03_28_19_25_15`'i gerçek harita klasörü adınızla değiştirin.

Ardından konumlandırma ve navigasyonu başlatın:

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_navigation.launch
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/LOC_NAV_realrobot_rviz.png" width="800" height="500"/>
</p>

##### Adım 1: Başlangıç Pozu Verme

RViz araç çubuğundaki **「3D Pose Estimate」** düğmesine tıklayın ve robotun gerçek dünya konumuyla eşleşen bir zemin noktası seçin.

Başlangıç pozu doğruysa, gözlemlenen nokta bulutunun harita özellikleriyle iyi örtüştüğünü göreceksiniz.

##### Adım 2: Hedef Gönderme

Başlatma tamamlandıktan sonra **「3D Goal Pose」** düğmesine tıklayın ve hedef olarak bir zemin noktası seçin.

Robot hedefe doğru hareket etmeye başlamalıdır.
