# دليل المبتدئين لـ DDDMR

يساعدك هذا الدليل على البدء باستخدام DDDMR Navigation. يتضمن محاكاة Lite3 في Gazebo وإرشادات إعداد روبوت حقيقي لاستكشاف الملاحة ثلاثية الأبعاد بسرعة.

## جدول المحتويات

| # | القسم | الوصف |
|:-:|:--------|:------------|
| 1 | [الملاحة في محاكاة Gazebo](#-dddmr-navigation-with-gazebo) | عرض محاكاة روبوت Lite3 رباعي الأرجل باستخدام RL |
| 2 | [الملاحة مع روبوت حقيقي](#-start-dddmr-navigation-with-a-real-robot) | النشر على روبوتك الحقيقي (روبوت بعجلات، رباعي الأرجل، بشري، إلخ) |

## 🖥️ متطلبات البرمجيات
- **Ubuntu 22.04** (تم اختباره؛ يمكن أيضًا تشغيل حاويات Humble على أجهزة مضيفة تعمل بنظام Ubuntu 24.04).
- **Docker** — [تثبيت Docker](https://docs.docker.com/engine/install/)
- **بطاقة NVIDIA GPU وNVIDIA Container Toolkit** لتشغيل المحاكاة على GPU وعرض الملاحة باستخدام TensorRT.

## ✨ DDDMR Navigation with Gazebo

شغّل DDDMR Navigation باستخدام **روبوت Lite3 رباعي الأرجل المتحكم به عبر RL**، ومستشعر LiDAR قببي، وعالم Gazebo متعدد المستويات. يستخدم العرض صورتين وحاويتين منفصلتين:

| الطرفية | الغرض | الصورة | الحاوية |
|:---------|:--------|:------|:----------|
| 1 | محاكاة Lite3 في Gazebo | `dddmr_simulation:rl_lite3` | `rl_lite3` |
| 2 | تحديد الموقع والملاحة ثلاثية الأبعاد باستخدام TensorRT | `dddmr:humble-cuda` | `dddmr_humble_cuda_dev` |

تستخدم الأوامر أدناه المسار `~/dddmr_navigation` لمستودع الملاحة والمسار `~/dfl_mobilerobot_simulator` للمحاكي.

### 1. إنشاء صورة Docker

استنسخ مستودع الملاحة أولًا. هذه الخطوة مطلوبة لكلا الخيارين أدناه:

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
```

ثم اختر **طريقة واحدة** من الطرق التالية للحصول على صورة الملاحة.

#### الخيار A: البناء محليًا

شغّل سكربت البناء، واختر **`x64`**، ثم أجب بـ **`Y`** عند السؤال عن CUDA. ينشئ ذلك الصورة `dddmr:humble-cuda` بما فيها اعتماديات CUDA وTensorRT.

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./build.bash
```

#### الخيار B: تنزيل الصورة العامة

توجد صورة مبنية مسبقًا ومنشورة بالفعل. نزّلها وأضف الوسم المحلي الذي يتوقعه `run_x64_gpu.bash`:

```bash
docker pull tsengapola/dddmr:humble-cuda
docker tag tsengapola/dddmr:humble-cuda dddmr:humble-cuda
```

بعد إكمال أي من الخيارين، تابع تنزيل الخريطة أدناه. تُترجم مساحة عمل الملاحة داخل حاوية الطرفية 2. أما صورة محاكاة Lite3 فتُبنى بشكل منفصل في خطوة **الطرفية 1** أدناه.

### 2. تنزيل خريطة الملاحة

نزّل خريطة الملاحة `stair_2_map.zip` الخاصة بعالم Lite3 وفك ضغطها. أجب بـ **`Y`** عند المطالبة:

```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide
./download_files.bash
```

ينشئ السكربت المجلد `~/dddmr_bags` ويفك ضغط الخريطة إلى `~/dddmr_bags/stair_2_map`. يربط سكربت التشغيل مجلد الخرائط بالمسار `/root/dddmr_bags` داخل حاوية الملاحة.

### 3. تجهيز بيئة العرض

افتح **طرفيتين منفصلتين**: واحدة للمحاكي والأخرى للملاحة. يجب أن تستخدم كلتا الحاويتين **`ROS_DOMAIN_ID=22`**.

#### 🖥️ الطرفية 1: تشغيل محاكاة Lite3 في Gazebo

اتبع [دليل Gazebo — Lite3](../GAZEBO_GUIDE.md#lite3--rl-quadruped) لاستنساخ الفرع `rl_lite3` وتهيئة الوحدات الفرعية وبناء الصورة وتشغيل `./rl_lite3_gpu.bash`.

بعد هبوط الروبوت على الأرض، اجعل طرفية المحاكي هي النافذة النشطة واضغط بالترتيب: **`r` ثم `0` ثم `1` ثم `n`**. تعيد هذه الخطوات ضبط الوضعية، وتجعل الروبوت يقف، وتشغّل متحكم RL، وتسمح باستقبال أوامر الملاحة. **انتظر ثانية واحدة على الأقل بين المفاتيح؛ وبعد `0` انتظر أيضًا حتى تكتمل حركة الوقوف.**

| المفتاح | الإجراء |
|:----|:-------|
| `r` | إعادة الروبوت إلى وضعيته الابتدائية في العالم. |
| `0` | جعل الروبوت يقف ويستعد للمشي. |
| `1` | تفعيل سياسة الحركة المعتمدة على RL. |
| `n` | تفعيل دخل `/cmd_vel` لتمكين نظام الملاحة من التحكم بالروبوت. |

اترك طرفية المحاكاة مفتوحة، ثم تابع الخطوات في الطرفية 2.

#### 🖥️ الطرفية 2: إنشاء حاوية الملاحة

##### الخطوة 1 (على الجهاز المضيف): تشغيل حاوية CUDA

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./run_x64_gpu.bash
```

أدخل **`22`** عندما يطلب السكربت قيمة `ROS_DOMAIN_ID`. ينشئ الحاوية `dddmr_humble_cuda_dev` باستخدام `dddmr:humble-cuda` ويربط المستودع بالمسار `/root/dddmr_navigation`.

##### الخطوة 2 (داخل حاوية الملاحة): البناء مع TensorRT

```bash
cd /root/dddmr_navigation
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=22
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DTRT_ENABLED=ON -DTEST_ENABLED=ON
```

يُفعّل `TRT_ENABLED=ON` شيفرة الاستدلال الخاصة بالملاحة. عند استنساخ المستودع من جديد، أنشئ أيضًا محرك TensorRT من النموذج المرفق باستخدام سكربت التحويل:

```bash
cd /root/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/engine
bash onnx2trt.bash
cd /root/dddmr_navigation
```

##### الخطوة 3 (داخل حاوية الملاحة): تشغيل تحديد الموقع والملاحة

```bash
source install/setup.bash
export ROS_DOMAIN_ID=22
ros2 launch p2p_move_base lite3_localization_launch.py
```

### 4. تشغيل العرض

1. تأكد من أن Lite3 في الطرفية 1 يعمل بوضع سياسة RL وأن دخل `/cmd_vel` مُفعّل.
2. تحقق في RViz من تطابق سحابة النقاط المباشرة مع الخريطة المرفقة. يتضمن إعداد العرض وضعية ابتدائية لهذا العالم. إذا نُقل الروبوت أو أُعيد ضبطه في مكان آخر، فاستخدم **3D Pose Estimate** لمحاذاته.
3. اختر **3D Goal Pose** في RViz وحدد نقطة على الأرض يمكن الوصول إليها. سيتبع Lite3 أوامر الملاحة للوصول إلى الهدف.

<p align="center">
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/3d_nav_rl_quadruped_deep_learning.png?raw=true" alt="الملاحة ثلاثية الأبعاد بالتعلم العميق لروبوت Lite3 رباعي الأرجل باستخدام RL في Gazebo وRViz" width="920" />
</p>

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
