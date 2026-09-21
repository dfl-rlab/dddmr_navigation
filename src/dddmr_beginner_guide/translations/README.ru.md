# Руководство для начинающих DDDMR

Это руководство является вводным туториалом по навигационной системе DDDMR. Оно включает пример симуляции четырёхногого робота в Gazebo и руководство по развёртыванию на реальном роботе, чтобы помочь вам быстро начать работу и насладиться 3D-навигацией.

## Содержание

| # | Раздел | Описание |
|:-:|:--------|:------------|
| 1 | [Навигация в симуляции Gazebo](#-dddmr-navigation-with-gazebo) | Демонстрация симуляции четырёхногого робота |
| 2 | [Навигация реального робота](#-start-dddmr-navigation-with-a-real-robot) | Развёртывание на реальном роботе (колёсный, четырёхногий, гуманоид и др.) |

## 🖥️ Требования к программному обеспечению
- **Ubuntu 22.04** (протестировано на 22.04, должно поддерживать 24.04)
- **Docker** — [Установка Docker](https://docs.docker.com/engine/install/)

## ✨ DDDMR Navigation with Gazebo

Эта демонстрация показывает, как запустить навигационный стек DDDMR в Gazebo с четырёхногим роботом.
- Соберите необходимые образы для подготовки среды.
- Запустите систему в двух терминалах — один для мира Gazebo, другой для навигационного стека.

### 1. Создание Docker-образа

Клонируйте репозиторий и запустите ./build.bash, выберите **`x64_gz`**, который уже содержит все необходимые компоненты для навигации и Gazebo.

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

### 2. Загрузка навигационной карты

Для запуска dddmr_navigation в Gazebo необходимо загрузить демонстрационную навигационную карту (12.3 МБ).

```bash
cd ~ && mkdir dddmr_bags
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./download_files.bash
```

### 3. Подготовка демонстрационной среды

#### Создание двух Docker-контейнеров (Gazebo и навигация)

> [!IMPORTANT] 
> Следующая команда запустит два интерактивных Docker-контейнера с использованием созданного образа. Пожалуйста, откройте **два отдельных терминала** для подготовки демонстрационной среды.

#### 🖥️ Терминал 1: Запуск симуляции Go2 в Gazebo

В первом терминале выполните оба шага из [руководства по Gazebo — Go2](../GAZEBO_GUIDE.md#go2--quadruped), чтобы собрать образ симуляции и запустить контейнер Gazebo.

После запуска симуляции Go2 оставьте этот терминал открытым и вернитесь сюда, чтобы продолжить работу в **Терминале 2**, как описано ниже.

#### 🖥️ Терминал 2 (создание контейнера для системы навигации)

- ##### Шаг 1 (на хосте): создание контейнера для системы навигации
```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./run_x64_navigation.bash
```

- ##### Шаг 2 (внутри контейнера навигации): сборка и запуск
```bash
cd dddmr_navigation/ && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash && ros2 launch p2p_move_base go2_localization.launch
```

### 4. Запуск демонстрации

- В демонстрации Gazebo карта уже выровнена, поэтому вам не нужно устанавливать начальную позу, если вы не создаёте карту самостоятельно
- Задайте цель (3D Goal Pose) в RViz, и робот переместится в целевую точку

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/give_goal_in_demo_.png" width="920" height="460"/>
</p>

### 5. Известные проблемы

> [!WARNING]
> Ниже приведены текущие наблюдаемые поведения, которые исследуются и будут исправлены в будущих обновлениях.
> - Gazebo: периодическое проскальзывание на склонах
> - Картографирование: дублирующиеся слои пола

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
