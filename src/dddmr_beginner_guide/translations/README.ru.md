# Руководство для начинающих DDDMR

Это руководство поможет начать работу с DDDMR Navigation. Оно включает симуляцию Lite3 в Gazebo и инструкции для реального робота, чтобы вы могли быстро освоить 3D-навигацию.

## Содержание

| # | Раздел | Описание |
|:-:|:--------|:------------|
| 1 | [Навигация в симуляции Gazebo](#-dddmr-navigation-with-gazebo) | Симуляция четвероногого Lite3 с управлением на основе RL |
| 2 | [Навигация реального робота](#-start-dddmr-navigation-with-a-real-robot) | Развёртывание на реальном роботе (колёсный, четырёхногий, гуманоид и др.) |

## 🖥️ Требования к программному обеспечению
- **Ubuntu 22.04** (проверено; на хостах Ubuntu 24.04 также можно запускать контейнеры Humble).
- **Docker** — [Установка Docker](https://docs.docker.com/engine/install/)
- **Видеокарта NVIDIA и NVIDIA Container Toolkit** для GPU-симуляции и демонстрации навигации с TensorRT.

## ✨ DDDMR Navigation with Gazebo

Запустите DDDMR Navigation с **четвероногим Lite3 под управлением RL**, купольным лидаром и многоуровневым миром Gazebo. В демонстрации используются два отдельных образа и контейнера:

| Терминал | Назначение | Образ | Контейнер |
|:---------|:--------|:------|:----------|
| 1 | Симуляция Lite3 в Gazebo | `dddmr_simulation:rl_lite3` | `rl_lite3` |
| 2 | Локализация и 3D-навигация с TensorRT | `dddmr:humble-cuda` | `dddmr_humble_cuda_dev` |

В командах ниже репозиторий навигации расположен в `~/dddmr_navigation`, а симулятор — в `~/dfl_mobilerobot_simulator`.

### 1. Создание образа Docker

Сначала клонируйте репозиторий навигации. Этот шаг необходим для обоих вариантов ниже:

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
```

Затем выберите **один** из следующих способов получения образа навигации.

#### Вариант A: Локальная сборка

Запустите скрипт сборки, выберите **`x64`** и ответьте **`Y`** на вопрос об использовании CUDA. Будет создан образ `dddmr:humble-cuda` с зависимостями CUDA и TensorRT.

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./build.bash
```

#### Вариант B: Загрузка публичного образа

Готовый образ уже опубликован. Загрузите его и добавьте локальный тег, который ожидает `run_x64_gpu.bash`:

```bash
docker pull tsengapola/dddmr:humble-cuda
docker tag tsengapola/dddmr:humble-cuda dddmr:humble-cuda
```

После любого варианта перейдите к загрузке карты. Рабочее пространство навигации компилируется внутри контейнера в терминале 2. Образ симуляции Lite3 собирается отдельно на этапе **Терминал 1** ниже.

### 2. Загрузка навигационной карты

Загрузите и распакуйте карту `stair_2_map.zip` для мира Lite3. На запрос ответьте **`Y`**:

```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide
./download_files.bash
```

Скрипт создаёт `~/dddmr_bags` и распаковывает карту в `~/dddmr_bags/stair_2_map`. Скрипт запуска монтирует каталог карт в `/root/dddmr_bags` внутри контейнера навигации.

### 3. Подготовка среды демонстрации

Откройте **два отдельных терминала**: один для симулятора, другой для навигации. В обоих контейнерах должно использоваться **`ROS_DOMAIN_ID=22`**.

#### 🖥️ Терминал 1: Запуск симуляции Lite3 в Gazebo

Следуйте [руководству Gazebo — Lite3](../GAZEBO_GUIDE.md#lite3--rl-quadruped): клонируйте ветку `rl_lite3`, инициализируйте подмодули, соберите образ и запустите `./rl_lite3_gpu.bash`.

Когда робот опустится на землю, установите фокус на терминал симулятора и нажмите **`r` → `0` → `1` → `n`**. Это сбросит позу, поднимет робота, запустит RL-контроллер и разрешит команды навигации. **Между нажатиями ждите не менее секунды; после `0` также дождитесь завершения подъёма.**

| Клавиша | Действие |
|:----|:-------|
| `r` | Вернуть робота в начальную позу в мире. |
| `0` | Поднять робота и подготовить его к ходьбе. |
| `1` | Активировать политику передвижения на основе RL. |
| `n` | Включить вход `/cmd_vel`, чтобы навигация могла управлять роботом. |

Оставьте терминал симуляции открытым и перейдите к терминалу 2.

#### 🖥️ Терминал 2: Создание контейнера навигации

##### Шаг 1 (на хосте): Запуск контейнера CUDA

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./run_x64_gpu.bash
```

Введите **`22`**, когда скрипт запросит `ROS_DOMAIN_ID`. Он создаст контейнер `dddmr_humble_cuda_dev` из образа `dddmr:humble-cuda` и смонтирует репозиторий в `/root/dddmr_navigation`.

##### Шаг 2 (в контейнере навигации): Сборка с TensorRT

```bash
cd /root/dddmr_navigation
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=22
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DTRT_ENABLED=ON -DTEST_ENABLED=ON
```

`TRT_ENABLED=ON` включает код инференса для навигации. После свежего клонирования также создайте движок TensorRT из включённой модели с помощью скрипта преобразования:

```bash
cd /root/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/engine
bash onnx2trt.bash
cd /root/dddmr_navigation
```

##### Шаг 3 (в контейнере навигации): Запуск локализации и навигации

```bash
source install/setup.bash
export ROS_DOMAIN_ID=22
ros2 launch p2p_move_base lite3_localization_launch.py
```

### 4. Запуск демонстрации

1. Убедитесь, что Lite3 в терминале 1 работает в режиме RL и вход `/cmd_vel` включён.
2. В RViz проверьте, что текущее облако точек совпадает с предоставленной картой. Конфигурация демонстрации содержит начальную позу для этого мира. Если робот был перемещён или сброшен в другое место, используйте **3D Pose Estimate** для совмещения.
3. Выберите **3D Goal Pose** в RViz и укажите достижимую точку на земле. Lite3 будет следовать командам навигации к цели.

<p align="center">
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/3d_nav_rl_quadruped_deep_learning.png?raw=true" alt="3D-навигация четвероногого Lite3 с RL и глубоким обучением в Gazebo и RViz" width="920" />
</p>

---

<p align="center">
  <b>━━━━━━━━━━ 🤖 Туториал по реальному роботу ниже 🤖 ━━━━━━━━━━</b>
</p>

---

## ✨ Start DDDMR Navigation with a Real Robot

Это руководство проведёт вас через настройку **3D-навигации** на реальном роботе — от картографирования до автономной навигации.

DDDMR обеспечивает 3D-навигацию для колёсных роботов, четырёхногих роботов, гуманоидов и других. Давайте заставим вашего робота двигаться!

### 1. Создание Docker-образа

Клонируйте репозиторий и запустите ./build.bash, выберите **`x64`** или **`l4t`** в зависимости от вашей платформы.

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

> **Примечание:** Наша тестовая платформа использует Jetson Orin Nano, поэтому мы выбираем **`l4t`**.

### 2. Тестовая платформа

Для быстрого соответствия этому туториалу, вот конфигурация оборудования, которую мы используем:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/hardware_quadruped.png" width="500" height="250"/>
</p>

| Компонент | Модель |
|:----------|:------|
| Робот | Четырёхногий робот (Lite3) |
| LiDAR | Купольный LiDAR (RoboSense Airy), наклон 45° |
| Компьютер | Jetson Orin Nano |
| Питание | Внешняя батарея для LiDAR и Jetson |
| Подключение | Ethernet (робот, Jetson и LiDAR общаются по сети) |

> **Примечание:** Вам не нужно точно такое же оборудование. Пока ваша система соответствует требованиям ниже, DDDMR будет работать.

#### Системные требования

| Элемент | Топик | Тип сообщения | Описание |
|:----:|:------|:-------------|:------------|
| 🎮 | `/cmd_vel` | `geometry_msgs/msg/Twist` | Команда скорости для управления роботом |
| 📡 | `/lidar_point_cloud` | `sensor_msgs/msg/PointCloud2` | 3D LiDAR облако точек (многослойный, купольный, твердотельный и т.д.) |
| 📍 | `/odom` | `nav_msgs/msg/Odometry` | Одометрия с TF (погрешность < 10% предпочтительна) |
| 🌳 | `/tf` | `tf2_msgs/msg/TFMessage` | TF-дерево: `odom` → `base_link` → `lidar_link` |

> **Примечание:** 
> - DDDMR можно использовать на **колёсных роботах, четырёхногих роботах и гуманоидах**, если они соответствуют вышеуказанным требованиям.
> - `/cmd_vel` — это **вход** для управления движением робота.
> - `/lidar_point_cloud`, `/odom`, `/tf` — это **выходы** робота, используемые DDDMR для картографирования и локализации.

#### Структура TF-дерева

Убедитесь, что `frame_id` в топике LiDAR соответствует вашему TF-дереву:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/tf_requirement.png" width="150" height="250"/>
</p>

#### Пример конфигурации TF

Вот конфигурация TF на примере нашего четырёхногого робота. LiDAR установлен на расстоянии `x=0.2м`, `z=0.22м` от `base_link` и наклонён вниз на 45°:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/setup_lidar_testing_quadruped.png" width="400" height="250"/>
</p>

```xml
<!--- TF: x y z yaw pitch roll -->
<node pkg="tf2_ros" exec="static_transform_publisher" name="sensor2baselink" 
      args="0.2 0.0 0.22 0.0 0.785 0.0 base_link lidar" />
```

> **Примечание:** 45° ≈ 0.785 радиан. Наклон вниз.

#### 👉 Продвинутый уровень (опционально)

| Функция | Описание |
|:--------|:------------|
| [3D-одометрия](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_odom_3d) | Лучшая локализация и картографирование на неровной местности |

---

### 3. 🚀 Запуск картографирования + навигации

DDDMR поддерживает три рабочих процесса навигации:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/dddmr_nav_workflows.png" width="600" height="450"/>
</p>

> **Примечание:** Этот туториал следует рабочему процессу **② Онлайн-картографирование + навигация**.

---

### 3.1 🗺️ Картографирование

Перед началом убедитесь, что ваш робот публикует необходимые топики:

- Топик одометрии
- Топик облака точек LiDAR
- TF (`odom` → `base_link` → `lidar_link`)

Затем запустите картографирование:

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_mapping.launch
```

После запуска вы должны увидеть интерфейс RViz, как показано ниже:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/mapping_realrobot_rviz.png" width="800" height="500"/>
</p>

Когда вы управляете роботом, вы заметите:
- **Ключевые кадры** — должны увеличиваться по мере движения робота
- **Точки признаков** — извлечённые признаки из окружающей среды
- **Точки земли** — обнаруженная поверхность земли
- **2D-проекция** — помогает понять сцену и поле зрения LiDAR

#### Сохранение карты

После завершения картографирования области откройте новый терминал и выполните:

```bash
ros2 service call /save_mapped_point_cloud std_srvs/srv/Empty
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/map_save_.png" width="500" height="250"/>
</p>

Когда в терминале появится `Create dir:`, это означает, что карта сохранена. Если картографирование завершено, вы можете закрыть узел картографирования.

Карта будет сохранена в `/tmp`. Вы можете переместить папку в `/root/dddmr_bags`:

```bash
mv /tmp/2026_03_28_19_25_15/ /root/dddmr_bags/
```

> **Примечание:** Имя папки `2026_03_28_19_25_15` — это просто пример. Ваше имя папки будет отличаться в зависимости от времени запуска картографирования.

---

### 3.2 📍 Локализация + навигация

Сначала откройте файл конфигурации и установите путь к карте:

📄 `config/airy_tilt45_navigation.yaml`

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/robot_map_location.png" width="500" height="500"/>
</p>

Измените `pose_graph_dir` на вашу папку с картой:

```yaml
pose_graph_dir: "/root/dddmr_bags/2026_03_28_19_25_15"
```

> **Примечание:** Замените `2026_03_28_19_25_15` на фактическое имя вашей папки с картой.

Затем запустите локализацию и навигацию:

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_navigation.launch
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/LOC_NAV_realrobot_rviz.png" width="800" height="500"/>
</p>

##### Шаг 1: Установка начальной позы

Нажмите **「3D Pose Estimate」** на панели инструментов RViz и выберите точку на земле, соответствующую реальному положению робота.

Если начальная поза правильная, вы увидите, что наблюдаемое облако точек хорошо совпадает с признаками карты.

##### Шаг 2: Отправка цели

После завершения инициализации нажмите **「3D Goal Pose」** и выберите точку на земле в качестве цели.

Робот должен начать движение к цели.
