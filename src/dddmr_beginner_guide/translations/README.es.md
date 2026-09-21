# Guía para principiantes de DDDMR

Esta guía es un tutorial introductorio al sistema de navegación DDDMR. Incluye un ejemplo de simulación de robot cuadrúpedo en Gazebo y una guía de despliegue en robot real, para ayudarte a comenzar rápidamente y disfrutar de la navegación 3D.

## Tabla de contenidos

| # | Sección | Descripción |
|:-:|:--------|:------------|
| 1 | [Navegación en simulación Gazebo](#-dddmr-navigation-with-gazebo) | Demo de simulación de robot cuadrúpedo |
| 2 | [Navegación con robot real](#-start-dddmr-navigation-with-a-real-robot) | Despliegue en tu robot real (robot con ruedas, cuadrúpedo, humanoide, etc.) |

## 🖥️ Requisitos de software
- **Ubuntu 22.04** (probado en 22.04, debería soportar 24.04)
- **Docker** — [Instalar Docker](https://docs.docker.com/engine/install/)

## ✨ DDDMR Navigation with Gazebo

Esta demo muestra cómo ejecutar el stack de navegación DDDMR en Gazebo con un robot cuadrúpedo.
- Construye las imágenes necesarias para preparar el entorno.
- Ejecuta el sistema en dos terminales — una para el mundo Gazebo, otra para el stack de navegación.

### 1. Crear imagen Docker

Clona el repositorio y ejecuta ./build.bash, selecciona **`x64_gz`**, que ya contiene todos los componentes necesarios para navegación y Gazebo.

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

### 2. Descargar mapa de navegación

Para ejecutar dddmr_navigation en Gazebo, necesitas descargar el mapa de navegación de demo (12.3MB).

```bash
cd ~ && mkdir dddmr_bags
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./download_files.bash
```

### 3. Preparar entorno de demostración

#### Crear dos contenedores Docker (Gazebo y navegación)

> [!IMPORTANT] 
> El siguiente comando iniciará dos contenedores Docker interactivos usando la imagen construida. Por favor abre **dos terminales separadas** para preparar el entorno de demostración.

#### 🖥️ Terminal 1: Iniciar la simulación de Go2 en Gazebo

En el primer terminal, sigue los dos pasos de la [Guía de Gazebo — Go2](../GAZEBO_GUIDE.md#go2--quadruped) para crear la imagen de simulación e iniciar el contenedor de Gazebo.

Una vez que la simulación de Go2 esté en ejecución, mantén este terminal abierto y vuelve aquí para continuar con el **Terminal 2** a continuación.

#### 🖥️ Terminal 2 (crear contenedor para sistema de navegación)

- ##### Paso 1 (en el host): crear contenedor para sistema de navegación
```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./run_x64_navigation.bash
```

- ##### Paso 2 (dentro del contenedor de navegación): construir y lanzar
```bash
cd dddmr_navigation/ && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash && ros2 launch p2p_move_base go2_localization.launch
```

### 4. Ejecutar demo

- En la demo de Gazebo, el mapa ya está alineado, así que no necesitas establecer la pose inicial a menos que estés mapeando tú mismo
- Da el objetivo (3D Goal Pose) en RViz, y el robot se moverá al punto objetivo

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/give_goal_in_demo_.png" width="920" height="460"/>
</p>

### 5. Problemas conocidos

> [!WARNING]
> Los siguientes son comportamientos observados actualmente, están siendo investigados y se corregirán en futuras actualizaciones.
> - Gazebo: deslizamiento ocasional en pendientes
> - Mapeo: capas de suelo duplicadas

---

<p align="center">
  <b>━━━━━━━━━━ 🤖 Tutorial de robot real abajo 🤖 ━━━━━━━━━━</b>
</p>

---

## ✨ Start DDDMR Navigation with a Real Robot

Esta guía te llevará a través de la configuración de **navegación 3D** en tu robot real — desde el mapeo hasta la navegación autónoma.

DDDMR lleva la navegación 3D a tus robots con ruedas, cuadrúpedos, humanoides y más. ¡Pongamos tu robot en movimiento!

### 1. Crear imagen Docker

Clona el repositorio y ejecuta ./build.bash, selecciona **`x64`** o **`l4t`** según tu plataforma.

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

> **Nota:** Nuestra plataforma de prueba usa Jetson Orin Nano, así que seleccionamos **`l4t`**.

### 2. Plataforma de prueba

Para seguir rápidamente este tutorial, aquí está la configuración de hardware que usamos:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/hardware_quadruped.png" width="500" height="250"/>
</p>

| Componente | Modelo |
|:----------|:------|
| Robot | Robot cuadrúpedo (Lite3) |
| LiDAR | LiDAR domo (RoboSense Airy), inclinado 45° |
| Computadora | Jetson Orin Nano |
| Alimentación | Batería externa para LiDAR y Jetson |
| Conexión | Ethernet (robot, Jetson y LiDAR se comunican por red) |

> **Nota:** No necesitas exactamente el mismo hardware. Mientras tu sistema cumpla los requisitos a continuación, DDDMR funcionará.

#### Requisitos del sistema

| Elemento | Topic | Tipo de mensaje | Descripción |
|:----:|:------|:-------------|:------------|
| 🎮 | `/cmd_vel` | `geometry_msgs/msg/Twist` | Comando de velocidad para controlar el robot |
| 📡 | `/lidar_point_cloud` | `sensor_msgs/msg/PointCloud2` | Nube de puntos LiDAR 3D (multicapa, domo, sólido, etc.) |
| 📍 | `/odom` | `nav_msgs/msg/Odometry` | Odometría con TF (error < 10% preferible) |
| 🌳 | `/tf` | `tf2_msgs/msg/TFMessage` | Árbol TF: `odom` → `base_link` → `lidar_link` |

> **Nota:** 
> - DDDMR puede usarse en **robots con ruedas, cuadrúpedos y humanoides** siempre que cumplan los requisitos anteriores.
> - `/cmd_vel` es la **entrada** para el control de movimiento del robot.
> - `/lidar_point_cloud`, `/odom`, `/tf` son **salidas** del robot, usadas por DDDMR para mapeo y localización.

#### Estructura del árbol TF

Asegúrate de que el `frame_id` en el topic LiDAR coincida con tu árbol TF:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/tf_requirement.png" width="150" height="250"/>
</p>

#### Ejemplo de configuración TF

Aquí está la configuración TF usando nuestro robot cuadrúpedo como ejemplo. El LiDAR está montado en `x=0.2m`, `z=0.22m` desde `base_link`, e inclinado hacia abajo 45°:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/setup_lidar_testing_quadruped.png" width="400" height="250"/>
</p>

```xml
<!--- TF: x y z yaw pitch roll -->
<node pkg="tf2_ros" exec="static_transform_publisher" name="sensor2baselink" 
      args="0.2 0.0 0.22 0.0 0.785 0.0 base_link lidar" />
```

> **Nota:** 45° ≈ 0.785 radianes. Inclinación hacia abajo.

#### 👉 Avanzado (Opcional)

| Característica | Descripción |
|:--------|:------------|
| [Odometría 3D](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_odom_3d) | Mejor localización y mapeo en terreno irregular |

---

### 3. 🚀 Ejecutar mapeo + navegación

DDDMR soporta tres flujos de trabajo de navegación:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/dddmr_nav_workflows.png" width="600" height="450"/>
</p>

> **Nota:** Este tutorial sigue el flujo de trabajo **② Mapeo en línea + Navegación**.

---

### 3.1 🗺️ Mapeo

Antes de comenzar, asegúrate de que tu robot esté publicando los topics requeridos:

- Topic de odometría
- Topic de nube de puntos LiDAR
- TF (`odom` → `base_link` → `lidar_link`)

Luego lanza el mapeo:

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_mapping.launch
```

Después del lanzamiento, deberías ver la interfaz RViz así:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/mapping_realrobot_rviz.png" width="800" height="500"/>
</p>

Cuando conduzcas tu robot, notarás:
- **Keyframes** — deberían aumentar a medida que el robot se mueve
- **Puntos característicos** — características extraídas del entorno
- **Puntos de suelo** — superficie del suelo detectada
- **Proyección 2D** — ayuda a entender la escena y el FOV del LiDAR

#### Guardar el mapa

Después de terminar el mapeo del área, abre una nueva terminal y ejecuta:

```bash
ros2 service call /save_mapped_point_cloud std_srvs/srv/Empty
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/map_save_.png" width="500" height="250"/>
</p>

Cuando veas `Create dir:` en la terminal, significa que el mapa ha sido guardado. Si el mapeo está completo, puedes cerrar el nodo de mapeo.

El mapa se guardará en `/tmp`. Puedes mover la carpeta a `/root/dddmr_bags`:

```bash
mv /tmp/2026_03_28_19_25_15/ /root/dddmr_bags/
```

> **Nota:** El nombre de carpeta `2026_03_28_19_25_15` es solo un ejemplo. Tu nombre de carpeta será diferente según cuándo ejecutes el mapeo.

---

### 3.2 📍 Localización + Navegación

Primero, abre el archivo de configuración y establece la ruta de tu mapa:

📄 `config/airy_tilt45_navigation.yaml`

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/robot_map_location.png" width="500" height="500"/>
</p>

Cambia `pose_graph_dir` a tu carpeta de mapa:

```yaml
pose_graph_dir: "/root/dddmr_bags/2026_03_28_19_25_15"
```

> **Nota:** Reemplaza `2026_03_28_19_25_15` con el nombre real de tu carpeta de mapa.

Luego lanza la localización y navegación:

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_navigation.launch
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/LOC_NAV_realrobot_rviz.png" width="800" height="500"/>
</p>

##### Paso 1: Dar pose inicial

Haz clic en **「3D Pose Estimate」** en la barra de herramientas de RViz y selecciona un punto en el suelo que coincida con la posición real del robot.

Si la pose inicial es correcta, verás la nube de puntos observada superpuesta bien con las características del mapa.

##### Paso 2: Enviar objetivo

Una vez completada la inicialización, haz clic en **「3D Goal Pose」** y selecciona un punto en el suelo como objetivo.

El robot debería comenzar a moverse hacia el objetivo.
