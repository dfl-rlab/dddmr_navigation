# DDDMR Anfängerhandbuch

Diese Anleitung führt in DDDMR Navigation ein. Mit der Lite3-Gazebo-Simulation und der Anleitung für echte Roboter können Sie schnell mit der 3D-Navigation beginnen.

## Inhaltsverzeichnis

| # | Abschnitt | Beschreibung |
|:-:|:--------|:------------|
| 1 | [Gazebo-Simulationsnavigation](#-dddmr-navigation-with-gazebo) | Simulationsdemo mit dem vierbeinigen Lite3 und RL |
| 2 | [Navigation mit echtem Roboter](#-start-dddmr-navigation-with-a-real-robot) | Bereitstellung auf Ihrem echten Roboter (Radroboter, Vierbeinige, Humanoide usw.) |

## 🖥️ Softwareanforderungen
- **Ubuntu 22.04** (getestet; auch Ubuntu-24.04-Hosts können die Humble-Container ausführen).
- **Docker** — [Docker installieren](https://docs.docker.com/engine/install/)
- **NVIDIA-GPU und NVIDIA Container Toolkit** für die GPU-Simulation und die Navigationsdemo mit TensorRT.

## ✨ DDDMR Navigation with Gazebo

Führen Sie DDDMR Navigation mit dem **vierbeinigen Lite3 mit RL-Steuerung**, einem Dome-LiDAR und einer mehrstöckigen Gazebo-Welt aus. Die Demo verwendet zwei separate Images und Container:

| Terminal | Zweck | Image | Container |
|:---------|:--------|:------|:----------|
| 1 | Lite3-Gazebo-Simulation | `dddmr_simulation:rl_lite3` | `rl_lite3` |
| 2 | Lokalisierung und 3D-Navigation mit TensorRT | `dddmr:humble-cuda` | `dddmr_humble_cuda_dev` |

Die folgenden Befehle verwenden `~/dddmr_navigation` für das Navigationsrepository und `~/dfl_mobilerobot_simulator` für den Simulator.

### 1. Docker-Image erstellen

Klonen Sie zuerst das Navigationsrepository. Dieser Schritt ist für beide folgenden Optionen erforderlich:

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
```

Wählen Sie anschließend **eine** der folgenden Möglichkeiten, um das Navigationsimage zu erhalten.

#### Option A: Lokal bauen

Führen Sie das Build-Skript aus, wählen Sie **`x64`** und beantworten Sie die CUDA-Abfrage mit **`Y`**. Damit wird `dddmr:humble-cuda` einschließlich der CUDA- und TensorRT-Abhängigkeiten erstellt.

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./build.bash
```

#### Option B: Öffentliches Image herunterladen

Ein vorgefertigtes Image ist bereits veröffentlicht. Laden Sie es herunter und ergänzen Sie den von `run_x64_gpu.bash` erwarteten lokalen Tag:

```bash
docker pull tsengapola/dddmr:humble-cuda
docker tag tsengapola/dddmr:humble-cuda dddmr:humble-cuda
```

Fahren Sie bei beiden Optionen mit dem Kartendownload fort. Der Navigationsworkspace wird im Container in Terminal 2 kompiliert. Das Lite3-Simulationsimage wird weiter unten in **Terminal 1** separat gebaut.

### 2. Navigationskarte herunterladen

Laden Sie die Navigationskarte `stair_2_map.zip` für die Lite3-Welt herunter und entpacken Sie sie. Antworten Sie auf die Abfrage mit **`Y`**:

```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide
./download_files.bash
```

Das Skript erstellt `~/dddmr_bags` und entpackt die Karte nach `~/dddmr_bags/stair_2_map`. Das Startskript bindet das Kartenverzeichnis im Navigationscontainer unter `/root/dddmr_bags` ein.

### 3. Demoumgebung vorbereiten

Öffnen Sie **zwei separate Terminals**: eines für den Simulator und eines für die Navigation. Beide Container müssen **`ROS_DOMAIN_ID=22`** verwenden.

#### 🖥️ Terminal 1: Lite3-Gazebo-Simulation starten

Folgen Sie der [Gazebo-Anleitung — Lite3](../GAZEBO_GUIDE.md#lite3--rl-quadruped), um den Branch `rl_lite3` zu klonen, die Submodule zu initialisieren, das Image zu bauen und `./rl_lite3_gpu.bash` auszuführen.

Sobald der Roboter auf dem Boden angekommen ist, fokussieren Sie das Simulatorterminal und drücken Sie **`r` → `0` → `1` → `n`**. Dadurch werden die Pose zurückgesetzt, das Aufstehen ausgeführt, die RL-Steuerung gestartet und Navigationsbefehle freigegeben. **Warten Sie zwischen den Tasten mindestens eine Sekunde; warten Sie nach `0` zusätzlich, bis das Aufstehen abgeschlossen ist.**

| Taste | Aktion |
|:----|:-------|
| `r` | Den Roboter auf seine Startpose in der Welt zurücksetzen. |
| `0` | Aufstehen und den Roboter für die Fortbewegung vorbereiten. |
| `1` | Die RL-Laufstrategie aktivieren. |
| `n` | Den Eingang `/cmd_vel` aktivieren, damit die Navigation den Roboter steuern kann. |

Lassen Sie das Simulationsterminal geöffnet und fahren Sie mit Terminal 2 fort.

#### 🖥️ Terminal 2: Navigationscontainer erstellen

##### Schritt 1 (auf dem Host): CUDA-Container starten

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./run_x64_gpu.bash
```

Geben Sie **`22`** ein, wenn das Skript nach `ROS_DOMAIN_ID` fragt. Es erstellt mit `dddmr:humble-cuda` den Container `dddmr_humble_cuda_dev` und bindet das Repository unter `/root/dddmr_navigation` ein.

##### Schritt 2 (im Navigationscontainer): Mit TensorRT bauen

```bash
cd /root/dddmr_navigation
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=22
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DTRT_ENABLED=ON -DTEST_ENABLED=ON
```

`TRT_ENABLED=ON` aktiviert den Inferenzcode der Navigation. Erzeugen Sie bei einem frisch geklonten Repository außerdem die TensorRT-Engine aus dem enthaltenen Modell mit dem Konvertierungsskript:

```bash
cd /root/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/engine
bash onnx2trt.bash
cd /root/dddmr_navigation
```

##### Schritt 3 (im Navigationscontainer): Lokalisierung und Navigation starten

```bash
source install/setup.bash
export ROS_DOMAIN_ID=22
ros2 launch p2p_move_base lite3_localization_launch.py
```

### 4. Demo ausführen

1. Prüfen Sie, dass Lite3 in Terminal 1 im RL-Modus läuft und der Eingang `/cmd_vel` aktiviert ist.
2. Prüfen Sie in RViz, ob die aktuelle Punktwolke mit der mitgelieferten Karte übereinstimmt. Die Demokonfiguration enthält eine Startpose für diese Welt. Falls der Roboter an eine andere Stelle bewegt oder zurückgesetzt wurde, richten Sie ihn mit **3D Pose Estimate** aus.
3. Wählen Sie in RViz **3D Goal Pose** und geben Sie einen erreichbaren Punkt auf dem Boden an. Lite3 folgt den Navigationsbefehlen zum Ziel.

<p align="center">
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/3d_nav_rl_quadruped_deep_learning.png?raw=true" alt="3D-Navigation mit Deep Learning für den vierbeinigen Lite3 mit RL in Gazebo und RViz" width="920" />
</p>

---

<p align="center">
  <b>━━━━━━━━━━ 🤖 Tutorial für echten Roboter unten 🤖 ━━━━━━━━━━</b>
</p>

---

## ✨ Start DDDMR Navigation with a Real Robot

Diese Anleitung führt Sie durch die Einrichtung der **3D-Navigation** auf Ihrem echten Roboter — von der Kartierung bis zur autonomen Navigation.

DDDMR bringt 3D-Navigation auf Ihre Radroboter, vierbeinigen Roboter, Humanoide und mehr. Lassen Sie uns Ihren Roboter in Bewegung setzen!

### 1. Docker-Image erstellen

Klonen Sie das Repository und führen Sie ./build.bash aus. Wählen Sie je nach Plattform **`x64`** oder **`l4t`**.

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

> **Hinweis:** Unsere Testplattform verwendet Jetson Orin Nano, daher wählen wir **`l4t`**.

### 2. Testplattform

Um diesem Tutorial schnell zu folgen, hier die von uns verwendete Hardware-Konfiguration:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/hardware_quadruped.png" width="500" height="250"/>
</p>

| Komponente | Modell |
|:----------|:------|
| Roboter | Vierbeiniger Roboter (Lite3) |
| LiDAR | Kuppel-LiDAR (RoboSense Airy), 45° geneigt |
| Computer | Jetson Orin Nano |
| Stromversorgung | Externe Batterie für LiDAR und Jetson |
| Verbindung | Ethernet (Roboter, Jetson und LiDAR kommunizieren über Netzwerk) |

> **Hinweis:** Sie benötigen nicht genau die gleiche Hardware. Solange Ihr System die folgenden Anforderungen erfüllt, funktioniert DDDMR.

#### Systemanforderungen

| Element | Topic | Nachrichtentyp | Beschreibung |
|:----:|:------|:-------------|:------------|
| 🎮 | `/cmd_vel` | `geometry_msgs/msg/Twist` | Geschwindigkeitsbefehl zur Steuerung des Roboters |
| 📡 | `/lidar_point_cloud` | `sensor_msgs/msg/PointCloud2` | 3D-LiDAR-Punktwolke (mehrschichtig, kuppelförmig, Solid usw.) |
| 📍 | `/odom` | `nav_msgs/msg/Odometry` | Odometrie mit TF (Fehler < 10% bevorzugt) |
| 🌳 | `/tf` | `tf2_msgs/msg/TFMessage` | TF-Baum: `odom` → `base_link` → `lidar_link` |

> **Hinweis:** 
> - DDDMR kann auf **Radrobotern, vierbeinigen Robotern und Humanoiden** verwendet werden, solange sie die oben genannten Anforderungen erfüllen.
> - `/cmd_vel` ist der **Eingang** für die Bewegungssteuerung des Roboters.
> - `/lidar_point_cloud`, `/odom`, `/tf` sind **Ausgänge** des Roboters, die DDDMR für Kartierung und Lokalisierung verwendet.

#### TF-Baumstruktur

Stellen Sie sicher, dass die `frame_id` im LiDAR-Topic mit Ihrem TF-Baum übereinstimmt:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/tf_requirement.png" width="150" height="250"/>
</p>

#### TF-Konfigurationsbeispiel

Hier ist die TF-Konfiguration am Beispiel unseres vierbeinigen Roboters. Der LiDAR ist bei `x=0,2m`, `z=0,22m` von `base_link` montiert und um 45° nach unten geneigt:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/setup_lidar_testing_quadruped.png" width="400" height="250"/>
</p>

```xml
<!--- TF: x y z yaw pitch roll -->
<node pkg="tf2_ros" exec="static_transform_publisher" name="sensor2baselink" 
      args="0.2 0.0 0.22 0.0 0.785 0.0 base_link lidar" />
```

> **Hinweis:** 45° ≈ 0,785 Radiant. Nach unten geneigt.

#### 👉 Fortgeschritten (Optional)

| Funktion | Beschreibung |
|:--------|:------------|
| [3D-Odometrie](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_odom_3d) | Bessere Lokalisierung und Kartierung auf unebenem Gelände |

---

### 3. 🚀 Kartierung + Navigation ausführen

DDDMR unterstützt drei Navigations-Workflows:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/dddmr_nav_workflows.png" width="600" height="450"/>
</p>

> **Hinweis:** Dieses Tutorial folgt dem Workflow **② Online-Kartierung + Navigation**.

---

### 3.1 🗺️ Kartierung

Stellen Sie vor dem Start sicher, dass Ihr Roboter die erforderlichen Topics veröffentlicht:

- Odometrie-Topic
- LiDAR-Punktwolken-Topic
- TF (`odom` → `base_link` → `lidar_link`)

Dann starten Sie die Kartierung:

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_mapping.launch
```

Nach dem Start sollten Sie die RViz-Oberfläche wie folgt sehen:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/mapping_realrobot_rviz.png" width="800" height="500"/>
</p>

Wenn Sie den Roboter herumfahren, werden Sie Folgendes bemerken:
- **Keyframes** — sollten zunehmen, während sich der Roboter bewegt
- **Merkmalspunkte** — aus der Umgebung extrahierte Merkmale
- **Bodenpunkte** — erkannte Bodenoberfläche
- **2D-Projektion** — hilft beim Verständnis der Szene und des LiDAR-FOV

#### Karte speichern

Nachdem Sie die Kartierung des Bereichs abgeschlossen haben, öffnen Sie ein neues Terminal und führen Sie aus:

```bash
ros2 service call /save_mapped_point_cloud std_srvs/srv/Empty
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/map_save_.png" width="500" height="250"/>
</p>

Wenn im Terminal `Create dir:` erscheint, bedeutet dies, dass die Karte gespeichert wurde. Wenn die Kartierung abgeschlossen ist, können Sie den Kartierungsknoten beenden.

Die Karte wird in `/tmp` gespeichert. Sie können den Ordner nach `/root/dddmr_bags` verschieben:

```bash
mv /tmp/2026_03_28_19_25_15/ /root/dddmr_bags/
```

> **Hinweis:** Der Ordnername `2026_03_28_19_25_15` ist nur ein Beispiel. Ihr Ordnername wird je nach Zeitpunkt der Kartierung unterschiedlich sein.

---

### 3.2 📍 Lokalisierung + Navigation

Öffnen Sie zunächst die Konfigurationsdatei und setzen Sie Ihren Kartenpfad:

📄 `config/airy_tilt45_navigation.yaml`

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/robot_map_location.png" width="500" height="500"/>
</p>

Ändern Sie `pose_graph_dir` zu Ihrem Kartenordner:

```yaml
pose_graph_dir: "/root/dddmr_bags/2026_03_28_19_25_15"
```

> **Hinweis:** Ersetzen Sie `2026_03_28_19_25_15` durch Ihren tatsächlichen Kartenordnernamen.

Dann starten Sie Lokalisierung und Navigation:

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_navigation.launch
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/LOC_NAV_realrobot_rviz.png" width="800" height="500"/>
</p>

##### Schritt 1: Anfangspose setzen

Klicken Sie auf **「3D Pose Estimate」** in der RViz-Symbolleiste und wählen Sie einen Bodenpunkt, der der tatsächlichen Position des Roboters entspricht.

Wenn die Anfangspose korrekt ist, werden Sie sehen, dass die beobachtete Punktwolke gut mit den Kartenmerkmalen übereinstimmt.

##### Schritt 2: Ziel senden

Sobald die Initialisierung abgeschlossen ist, klicken Sie auf **「3D Goal Pose」** und wählen Sie einen Bodenpunkt als Ziel.

Der Roboter sollte beginnen, sich zum Ziel zu bewegen.
