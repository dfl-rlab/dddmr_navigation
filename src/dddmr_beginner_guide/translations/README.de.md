# DDDMR Anfängerhandbuch

Dieses Handbuch ist ein Einführungs-Tutorial für das DDDMR-Navigationssystem. Es enthält ein Gazebo-Simulationsbeispiel mit einem vierbeinigen Roboter und eine Anleitung zur Bereitstellung auf einem echten Roboter, um Ihnen den schnellen Einstieg zu ermöglichen und 3D-Navigation zu erkunden und zu genießen.

## Inhaltsverzeichnis

| # | Abschnitt | Beschreibung |
|:-:|:--------|:------------|
| 1 | [Gazebo-Simulationsnavigation](#-dddmr-navigation-with-gazebo) | Demo der Simulation eines vierbeinigen Roboters |
| 2 | [Navigation mit echtem Roboter](#-start-dddmr-navigation-with-a-real-robot) | Bereitstellung auf Ihrem echten Roboter (Radroboter, Vierbeinige, Humanoide usw.) |

## 🖥️ Softwareanforderungen
- **Ubuntu 22.04** (getestet auf 22.04, sollte 24.04 unterstützen)
- **Docker** — [Docker installieren](https://docs.docker.com/engine/install/)

## ✨ DDDMR Navigation with Gazebo

Diese Demo zeigt, wie der DDDMR Navigation Stack in Gazebo mit einem vierbeinigen Roboter ausgeführt wird.
- Erstellen Sie die erforderlichen Images zur Vorbereitung der Umgebung.
- Führen Sie das System in zwei Terminals aus — eines für die Gazebo-Welt, das andere für den Navigation Stack.

### 1. Docker-Image erstellen

Klonen Sie das Repository und führen Sie ./build.bash aus. Wählen Sie **`x64_gz`**, das bereits alle notwendigen Komponenten für Navigation und Gazebo enthält.

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

### 2. Navigationskarte herunterladen

Um dddmr_navigation in Gazebo auszuführen, müssen Sie die Demo-Navigationskarte (12,3 MB) herunterladen.

```bash
cd ~ && mkdir dddmr_bags
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./download_files.bash
```

### 3. Demo-Umgebung vorbereiten

#### Zwei Docker-Container erstellen (Gazebo und Navigation)

> [!IMPORTANT] 
> Der folgende Befehl startet zwei interaktive Docker-Container mit dem erstellten Image. Bitte öffnen Sie **zwei separate Terminals**, um die Demo-Umgebung vorzubereiten.

#### 🖥️ Terminal 1: Go2-Simulation in Gazebo starten

Befolgen Sie im ersten Terminal beide Schritte im [Gazebo-Leitfaden — Go2](../GAZEBO_GUIDE.md#go2--quadruped), um das Simulationsimage zu erstellen und den Gazebo-Container zu starten.

Sobald die Go2-Simulation läuft, lassen Sie dieses Terminal geöffnet und kehren Sie hierher zurück, um mit **Terminal 2** unten fortzufahren.

#### 🖥️ Terminal 2 (Container für Navigationssystem erstellen)

- ##### Schritt 1 (auf dem Host): Container für Navigationssystem erstellen
```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./run_x64_navigation.bash
```

- ##### Schritt 2 (im Navigations-Container): Bauen und starten
```bash
cd dddmr_navigation/ && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash && ros2 launch p2p_move_base go2_localization.launch
```

### 4. Demo ausführen

- In der Gazebo-Demo ist die Karte bereits ausgerichtet, sodass Sie keine Anfangspose setzen müssen, es sei denn, Sie kartieren selbst
- Geben Sie das Ziel (3D Goal Pose) in RViz an, und der Roboter bewegt sich zum Zielpunkt

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/give_goal_in_demo_.png" width="920" height="460"/>
</p>

### 5. Bekannte Probleme

> [!WARNING]
> Die folgenden Verhaltensweisen werden derzeit beobachtet, werden untersucht und in zukünftigen Updates behoben.
> - Gazebo: Gelegentliches Rutschen an Hängen
> - Kartierung: Doppelte Bodenschichten

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
