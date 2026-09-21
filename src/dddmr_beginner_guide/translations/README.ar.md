# دليل المبتدئين لـ DDDMR

هذا الدليل هو مقدمة تعليمية لنظام الملاحة DDDMR. يتضمن مثال محاكاة روبوت رباعي الأرجل في Gazebo ودليل نشر على روبوت حقيقي، لمساعدتك على البدء بسرعة والاستمتاع بالملاحة ثلاثية الأبعاد.

## جدول المحتويات

| # | القسم | الوصف |
|:-:|:--------|:------------|
| 1 | [الملاحة في محاكاة Gazebo](#-dddmr-navigation-with-gazebo) | عرض محاكاة روبوت رباعي الأرجل |
| 2 | [الملاحة مع روبوت حقيقي](#-start-dddmr-navigation-with-a-real-robot) | النشر على روبوتك الحقيقي (روبوت بعجلات، رباعي الأرجل، بشري، إلخ) |

## 🖥️ متطلبات البرمجيات
- **Ubuntu 22.04** (تم اختباره على 22.04، يجب أن يدعم 24.04)
- **Docker** — [تثبيت Docker](https://docs.docker.com/engine/install/)

## ✨ DDDMR Navigation with Gazebo

يوضح هذا العرض كيفية تشغيل حزمة ملاحة DDDMR في Gazebo مع روبوت رباعي الأرجل.
- قم ببناء الصور المطلوبة لتحضير البيئة.
- قم بتشغيل النظام في نافذتي طرفية — واحدة لعالم Gazebo والأخرى لحزمة الملاحة.

### 1. إنشاء صورة Docker

قم باستنساخ المستودع وتشغيل ./build.bash، اختر **`x64_gz`**، الذي يحتوي بالفعل على جميع المكونات اللازمة للملاحة و Gazebo.

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

### 2. تحميل خريطة الملاحة

لتشغيل dddmr_navigation في Gazebo، تحتاج إلى تحميل خريطة الملاحة التجريبية (12.3 ميجابايت).

```bash
cd ~ && mkdir dddmr_bags
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./download_files.bash
```

### 3. تحضير بيئة العرض

#### إنشاء حاويتي Docker (Gazebo والملاحة)

> [!IMPORTANT] 
> سيقوم الأمر التالي بتشغيل حاويتي Docker تفاعليتين باستخدام الصورة التي قمنا ببنائها. يرجى فتح **نافذتي طرفية منفصلتين** لتحضير بيئة العرض.

#### 🖥️ الطرفية 1: تشغيل محاكاة Go2 في Gazebo

في الطرفية الأولى، اتبع الخطوتين في [دليل Gazebo — Go2](../GAZEBO_GUIDE.md#go2--quadruped) لبناء صورة المحاكاة وتشغيل حاوية Gazebo.

بعد تشغيل محاكاة Go2، اترك هذه الطرفية مفتوحة وعد إلى هنا لمتابعة خطوات **الطرفية 2** أدناه.

#### 🖥️ الطرفية 2 (إنشاء حاوية لنظام الملاحة)

- ##### الخطوة 1 (على المضيف): إنشاء حاوية لنظام الملاحة
```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./run_x64_navigation.bash
```

- ##### الخطوة 2 (داخل حاوية الملاحة): البناء والتشغيل
```bash
cd dddmr_navigation/ && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash && ros2 launch p2p_move_base go2_localization.launch
```

### 4. تشغيل العرض

- في عرض Gazebo، الخريطة محاذية بالفعل، لذا لا تحتاج إلى تعيين الوضع الأولي إلا إذا كنت تقوم برسم الخريطة بنفسك
- أعط الهدف (3D Goal Pose) في RViz، وسيتحرك الروبوت إلى نقطة الهدف

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/give_goal_in_demo_.png" width="920" height="460"/>
</p>

### 5. المشاكل المعروفة

> [!WARNING]
> فيما يلي السلوكيات الملاحظة حاليًا، وهي قيد التحقيق وسيتم إصلاحها في التحديثات المستقبلية.
> - Gazebo: انزلاق عرضي على المنحدرات
> - رسم الخرائط: طبقات أرضية مكررة

---

<p align="center">
  <b>━━━━━━━━━━ 🤖 دليل الروبوت الحقيقي أدناه 🤖 ━━━━━━━━━━</b>
</p>

---

## ✨ Start DDDMR Navigation with a Real Robot

سيرشدك هذا الدليل خلال إعداد **الملاحة ثلاثية الأبعاد** على روبوتك الحقيقي — من رسم الخرائط إلى الملاحة الذاتية.

يجلب DDDMR الملاحة ثلاثية الأبعاد إلى روبوتاتك ذات العجلات ورباعية الأرجل والبشرية وأكثر. دعنا نحرك روبوتك!

### 1. إنشاء صورة Docker

قم باستنساخ المستودع وتشغيل ./build.bash، اختر **`x64`** أو **`l4t`** حسب منصتك.

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

> **ملاحظة:** منصة الاختبار لدينا تستخدم Jetson Orin Nano، لذا نختار **`l4t`**.

### 2. منصة الاختبار

لمتابعة هذا الدليل بسرعة، إليك تكوين الأجهزة الذي نستخدمه:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/hardware_quadruped.png" width="500" height="250"/>
</p>

| المكون | الطراز |
|:----------|:------|
| الروبوت | روبوت رباعي الأرجل (Lite3) |
| LiDAR | قبة LiDAR (RoboSense Airy)، مائل 45° |
| الحاسوب | Jetson Orin Nano |
| الطاقة | بطارية خارجية لـ LiDAR و Jetson |
| الاتصال | إيثرنت (الروبوت و Jetson و LiDAR يتواصلون عبر الشبكة) |

> **ملاحظة:** لا تحتاج إلى نفس الأجهزة بالضبط. طالما أن نظامك يلبي المتطلبات أدناه، سيعمل DDDMR.

#### متطلبات النظام

| العنصر | الموضوع | نوع الرسالة | الوصف |
|:----:|:------|:-------------|:------------|
| 🎮 | `/cmd_vel` | `geometry_msgs/msg/Twist` | أمر السرعة للتحكم في الروبوت |
| 📡 | `/lidar_point_cloud` | `sensor_msgs/msg/PointCloud2` | سحابة نقاط LiDAR ثلاثية الأبعاد (متعدد الطبقات، قبة، صلب، إلخ) |
| 📍 | `/odom` | `nav_msgs/msg/Odometry` | قياس المسافات مع TF (خطأ < 10% أفضل) |
| 🌳 | `/tf` | `tf2_msgs/msg/TFMessage` | شجرة TF: `odom` → `base_link` → `lidar_link` |

> **ملاحظة:** 
> - يمكن استخدام DDDMR على **الروبوتات ذات العجلات ورباعية الأرجل والبشرية** طالما أنها تلبي المتطلبات أعلاه.
> - `/cmd_vel` هو **المدخل** للتحكم في حركة الروبوت.
> - `/lidar_point_cloud`، `/odom`، `/tf` هي **مخرجات** الروبوت، يستخدمها DDDMR لرسم الخرائط وتحديد الموقع.

#### هيكل شجرة TF

تأكد من أن `frame_id` في موضوع LiDAR يتطابق مع شجرة TF الخاصة بك:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/tf_requirement.png" width="150" height="250"/>
</p>

#### مثال تكوين TF

إليك تكوين TF باستخدام روبوتنا رباعي الأرجل كمثال. تم تركيب LiDAR على `x=0.2m`، `z=0.22m` من `base_link`، ومائل للأسفل بزاوية 45°:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/setup_lidar_testing_quadruped.png" width="400" height="250"/>
</p>

```xml
<!--- TF: x y z yaw pitch roll -->
<node pkg="tf2_ros" exec="static_transform_publisher" name="sensor2baselink" 
      args="0.2 0.0 0.22 0.0 0.785 0.0 base_link lidar" />
```

> **ملاحظة:** 45° ≈ 0.785 راديان. مائل للأسفل.

#### 👉 متقدم (اختياري)

| الميزة | الوصف |
|:--------|:------------|
| [قياس المسافات ثلاثي الأبعاد](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_odom_3d) | تحديد موقع ورسم خرائط أفضل على التضاريس غير المستوية |

---

### 3. 🚀 تشغيل رسم الخرائط + الملاحة

يدعم DDDMR ثلاثة مسارات عمل للملاحة:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/dddmr_nav_workflows.png" width="600" height="450"/>
</p>

> **ملاحظة:** يتبع هذا الدليل مسار العمل **② رسم الخرائط عبر الإنترنت + الملاحة**.

---

### 3.1 🗺️ رسم الخرائط

قبل البدء، تأكد من أن روبوتك ينشر المواضيع المطلوبة:

- موضوع قياس المسافات
- موضوع سحابة نقاط LiDAR
- TF (`odom` → `base_link` → `lidar_link`)

ثم قم بتشغيل رسم الخرائط:

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_mapping.launch
```

بعد التشغيل، يجب أن ترى واجهة RViz كما يلي:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/mapping_realrobot_rviz.png" width="800" height="500"/>
</p>

عندما تقود روبوتك، ستلاحظ:
- **الإطارات الرئيسية** — يجب أن تزداد مع تحرك الروبوت
- **نقاط الميزات** — ميزات مستخرجة من البيئة
- **نقاط الأرض** — سطح الأرض المكتشف
- **الإسقاط ثنائي الأبعاد** — يساعد على فهم المشهد ومجال رؤية LiDAR

#### حفظ الخريطة

بعد الانتهاء من رسم خريطة المنطقة، افتح طرفية جديدة وقم بتشغيل:

```bash
ros2 service call /save_mapped_point_cloud std_srvs/srv/Empty
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/map_save_.png" width="500" height="250"/>
</p>

عندما ترى `Create dir:` في الطرفية، يعني ذلك أن الخريطة قد تم حفظها. إذا اكتمل رسم الخرائط، يمكنك إغلاق عقدة رسم الخرائط.

سيتم حفظ الخريطة في `/tmp`. يمكنك نقل المجلد إلى `/root/dddmr_bags`:

```bash
mv /tmp/2026_03_28_19_25_15/ /root/dddmr_bags/
```

> **ملاحظة:** اسم المجلد `2026_03_28_19_25_15` هو مجرد مثال. سيختلف اسم مجلدك بناءً على وقت تشغيل رسم الخرائط.

---

### 3.2 📍 تحديد الموقع + الملاحة

أولاً، افتح ملف التكوين وقم بتعيين مسار خريطتك:

📄 `config/airy_tilt45_navigation.yaml`

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/robot_map_location.png" width="500" height="500"/>
</p>

قم بتغيير `pose_graph_dir` إلى مجلد خريطتك:

```yaml
pose_graph_dir: "/root/dddmr_bags/2026_03_28_19_25_15"
```

> **ملاحظة:** استبدل `2026_03_28_19_25_15` باسم مجلد خريطتك الفعلي.

ثم قم بتشغيل تحديد الموقع والملاحة:

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_navigation.launch
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/LOC_NAV_realrobot_rviz.png" width="800" height="500"/>
</p>

##### الخطوة 1: إعطاء الوضع الأولي

انقر على **「3D Pose Estimate」** في شريط أدوات RViz واختر نقطة على الأرض تتطابق مع موقع الروبوت الحقيقي.

إذا كان الوضع الأولي صحيحًا، سترى سحابة النقاط المرصودة تتداخل جيدًا مع ميزات الخريطة.

##### الخطوة 2: إرسال الهدف

بمجرد اكتمال التهيئة، انقر على **「3D Goal Pose」** واختر نقطة على الأرض كهدف.

يجب أن يبدأ الروبوت بالتحرك نحو الهدف.
