# DDDMR Başlangıç Kılavuzu

Bu kılavuz, DDDMR navigasyon sistemine giriş niteliğinde bir öğreticidir. Gazebo'da dört ayaklı robot simülasyonu örneği ve gerçek robot dağıtım kılavuzu içerir, böylece hızlıca başlayabilir ve 3D navigasyonun keyfini çıkarabilirsiniz.

## İçindekiler

| # | Bölüm | Açıklama |
|:-:|:--------|:------------|
| 1 | [Gazebo Simülasyon Navigasyonu](#-dddmr-navigation-with-gazebo) | Dört ayaklı robot simülasyon demosu |
| 2 | [Gerçek Robot Navigasyonu](#-start-dddmr-navigation-with-a-real-robot) | Gerçek robotunuza dağıtım (tekerlekli, dört ayaklı, insansı vb.) |

## 🖥️ Yazılım Gereksinimleri
- **Ubuntu 22.04** (22.04'te test edildi, 24.04 desteklemeli)
- **Docker** — [Docker Kurulumu](https://docs.docker.com/engine/install/)

## ✨ DDDMR Navigation with Gazebo

Bu demo, DDDMR navigasyon yığınının Gazebo'da dört ayaklı bir robotla nasıl çalıştırılacağını gösterir.
- Ortamı hazırlamak için gerekli imajları oluşturun.
- Sistemi iki terminalde çalıştırın — biri Gazebo dünyası için, diğeri navigasyon yığını için.

### 1. Docker İmajı Oluşturma

Depoyu klonlayın ve ./build.bash çalıştırın, navigasyon ve Gazebo için gerekli tüm bileşenleri içeren **`x64_gz`** seçeneğini seçin.

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

### 2. Navigasyon Haritasını İndirme

Gazebo'da dddmr_navigation çalıştırmak için demo navigasyon haritasını (12.3MB) indirmeniz gerekir.

```bash
cd ~ && mkdir dddmr_bags
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./download_files.bash
```

### 3. Demo Ortamını Hazırlama

#### İki Docker Konteyneri Oluşturma (Gazebo ve navigasyon)

> [!IMPORTANT] 
> Aşağıdaki komut, oluşturduğumuz imajı kullanarak iki etkileşimli Docker konteyneri başlatacaktır. Demo ortamını hazırlamak için lütfen **iki ayrı terminal** açın.

#### 🖥️ Terminal 1: Go2 Gazebo Simülasyonunu Başlatma

İlk terminalde, simülasyon imajını oluşturmak ve Gazebo konteynerini başlatmak için [Gazebo Kılavuzu — Go2](../GAZEBO_GUIDE.md#go2--quadruped) içindeki iki adımı da uygulayın.

Go2 simülasyonu çalışmaya başladığında bu terminali açık bırakın ve aşağıdaki **Terminal 2** adımlarına devam etmek için buraya dönün.

#### 🖥️ Terminal 2 (navigasyon sistemi için konteyner oluşturma)

- ##### Adım 1 (ana bilgisayarda): navigasyon sistemi için konteyner oluşturma
```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./run_x64_navigation.bash
```

- ##### Adım 2 (navigasyon konteyneri içinde): derleme ve başlatma
```bash
cd dddmr_navigation/ && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash && ros2 launch p2p_move_base go2_localization.launch
```

### 4. Demo Çalıştırma

- Gazebo demosunda harita zaten hizalanmış durumda, bu yüzden kendiniz haritalamıyorsanız başlangıç pozunu ayarlamanıza gerek yok
- RViz'de hedef verin (3D Goal Pose), robot hedef noktaya hareket edecektir

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/give_goal_in_demo_.png" width="920" height="460"/>
</p>

### 5. Bilinen Sorunlar

> [!WARNING]
> Aşağıdakiler şu anda gözlemlenen davranışlardır, araştırılmakta olup gelecek güncellemelerde düzeltilecektir.
> - Gazebo: Eğimlerde ara sıra kayma
> - Haritalama: Yinelenen zemin katmanları

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
