# Guía para principiantes de DDDMR

Esta guía introduce DDDMR Navigation. Con la simulación de Lite3 en Gazebo y las instrucciones para un robot real, podrás comenzar rápidamente a explorar la navegación 3D.

## Tabla de contenidos

| # | Sección | Descripción |
|:-:|:--------|:------------|
| 1 | [Navegación en simulación Gazebo](#-dddmr-navigation-with-gazebo) | Demo de simulación del cuadrúpedo Lite3 con RL |
| 2 | [Navegación con robot real](#-start-dddmr-navigation-with-a-real-robot) | Despliegue en tu robot real (robot con ruedas, cuadrúpedo, humanoide, etc.) |

## 🖥️ Requisitos de software
- **Ubuntu 22.04** (probado; los equipos con Ubuntu 24.04 también pueden ejecutar los contenedores Humble).
- **Docker** — [Instalar Docker](https://docs.docker.com/engine/install/)
- **GPU NVIDIA y NVIDIA Container Toolkit** para la simulación con GPU y la demo de navegación con TensorRT.

## ✨ DDDMR Navigation with Gazebo

Ejecuta DDDMR Navigation con el **cuadrúpedo Lite3 controlado por RL**, un LiDAR de tipo domo y un mundo Gazebo de varios niveles. La demo usa dos imágenes y contenedores independientes:

| Terminal | Función | Imagen | Contenedor |
|:---------|:--------|:------|:----------|
| 1 | Simulación de Lite3 en Gazebo | `dddmr_simulation:rl_lite3` | `rl_lite3` |
| 2 | Localización y navegación 3D con TensorRT | `dddmr:humble-cuda` | `dddmr_humble_cuda_dev` |

Los comandos siguientes usan `~/dddmr_navigation` para el repositorio de navegación y `~/dfl_mobilerobot_simulator` para el simulador.

### 1. Crear la imagen Docker

Primero clona el repositorio de navegación. Este paso es necesario para ambas opciones:

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
```

A continuación, elige **una** de las siguientes formas de obtener la imagen de navegación.

#### Opción A: Compilar localmente

Ejecuta el script de construcción, selecciona **`x64`** y responde **`Y`** a la pregunta sobre CUDA. Esto crea `dddmr:humble-cuda` con las dependencias de CUDA y TensorRT.

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./build.bash
```

#### Opción B: Descargar la imagen pública

Ya hay una imagen precompilada publicada. Descárgala y añade la etiqueta local que espera `run_x64_gpu.bash`:

```bash
docker pull tsengapola/dddmr:humble-cuda
docker tag tsengapola/dddmr:humble-cuda dddmr:humble-cuda
```

Con cualquiera de las opciones, continúa con la descarga del mapa. El espacio de trabajo de navegación se compila dentro del contenedor de la terminal 2. La imagen de simulación de Lite3 se construye por separado en el paso **Terminal 1** de abajo.

### 2. Descargar el mapa de navegación

Descarga y descomprime el mapa `stair_2_map.zip` para el mundo de Lite3. Responde **`Y`** cuando se solicite:

```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide
./download_files.bash
```

El script crea `~/dddmr_bags` y extrae el mapa en `~/dddmr_bags/stair_2_map`. El script de inicio monta el directorio de mapas en `/root/dddmr_bags` dentro del contenedor de navegación.

### 3. Preparar el entorno de demostración

Abre **dos terminales independientes**: una para el simulador y otra para la navegación. Ambos contenedores deben usar **`ROS_DOMAIN_ID=22`**.

#### 🖥️ Terminal 1: Iniciar la simulación de Lite3 en Gazebo

Sigue la [guía de Gazebo — Lite3](../GAZEBO_GUIDE.md#lite3--rl-quadruped) para clonar la rama `rl_lite3`, inicializar los submódulos, construir la imagen y ejecutar `./rl_lite3_gpu.bash`.

Cuando el robot haya caído al suelo, enfoca la terminal del simulador y pulsa **`r` → `0` → `1` → `n`**. Esto restablece la pose, levanta el robot, inicia el controlador RL y permite recibir órdenes de navegación. **Espera al menos un segundo entre teclas; después de `0`, espera también a que el robot termine de levantarse.**

| Tecla | Acción |
|:----|:-------|
| `r` | Restablecer la pose inicial del robot en el mundo. |
| `0` | Levantar el robot y prepararlo para caminar. |
| `1` | Activar la política de locomoción RL. |
| `n` | Habilitar la entrada `/cmd_vel` para que la navegación controle el robot. |

Deja abierta la terminal de simulación y continúa con la terminal 2.

#### 🖥️ Terminal 2: Crear el contenedor de navegación

##### Paso 1 (en el equipo anfitrión): Iniciar el contenedor CUDA

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./run_x64_gpu.bash
```

Introduce **`22`** cuando el script solicite `ROS_DOMAIN_ID`. Crea el contenedor `dddmr_humble_cuda_dev` con `dddmr:humble-cuda` y monta el repositorio en `/root/dddmr_navigation`.

##### Paso 2 (dentro del contenedor de navegación): Compilar con TensorRT

```bash
cd /root/dddmr_navigation
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=22
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DTRT_ENABLED=ON -DTEST_ENABLED=ON
```

`TRT_ENABLED=ON` activa el código de inferencia de navegación. En un repositorio recién clonado, genera también el motor TensorRT a partir del modelo incluido, usando el script de conversión:

```bash
cd /root/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/engine
bash onnx2trt.bash
cd /root/dddmr_navigation
```

##### Paso 3 (dentro del contenedor de navegación): Iniciar la localización y la navegación

```bash
source install/setup.bash
export ROS_DOMAIN_ID=22
ros2 launch p2p_move_base lite3_localization_launch.py
```

### 4. Ejecutar la demo

1. Confirma que Lite3 está en modo de política RL en la terminal 1 y que la entrada `/cmd_vel` está habilitada.
2. En RViz, comprueba que la nube de puntos en directo se superpone al mapa proporcionado. La configuración de la demo incluye una pose inicial para este mundo. Si moviste o restableciste el robot en otro lugar, usa **3D Pose Estimate** para alinearlo.
3. Selecciona **3D Goal Pose** en RViz y elige un punto alcanzable sobre el suelo. Lite3 seguirá las órdenes de navegación hacia el objetivo.

<p align="center">
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/3d_nav_rl_quadruped_deep_learning.png?raw=true" alt="Navegación 3D con aprendizaje profundo del cuadrúpedo Lite3 RL en Gazebo y RViz" width="920" />
</p>

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
