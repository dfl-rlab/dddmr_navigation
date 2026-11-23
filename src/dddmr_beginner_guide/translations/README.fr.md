# Guide du débutant DDDMR

Ce guide présente DDDMR Navigation aux débutants. La simulation Lite3 dans Gazebo et les instructions pour un robot réel vous permettent de découvrir rapidement la navigation 3D.

## Table des matières

| # | Section | Description |
|:-:|:--------|:------------|
| 1 | [Navigation en simulation Gazebo](#-dddmr-navigation-with-gazebo) | Démo de simulation du quadrupède Lite3 avec RL |
| 2 | [Navigation avec robot réel](#-start-dddmr-navigation-with-a-real-robot) | Déploiement sur votre robot réel (robot à roues, quadrupède, humanoïde, etc.) |

## 🖥️ Prérequis logiciels
- **Ubuntu 22.04** (testé ; les hôtes Ubuntu 24.04 peuvent aussi exécuter les conteneurs Humble).
- **Docker** — [Installer Docker](https://docs.docker.com/engine/install/)
- **GPU NVIDIA et NVIDIA Container Toolkit** pour la simulation GPU et la démonstration de navigation avec TensorRT.

## ✨ DDDMR Navigation with Gazebo

Exécutez DDDMR Navigation avec le **quadrupède Lite3 piloté par RL**, un LiDAR de type dôme et un monde Gazebo à plusieurs niveaux. La démonstration utilise deux images et conteneurs distincts :

| Terminal | Rôle | Image | Conteneur |
|:---------|:--------|:------|:----------|
| 1 | Simulation Lite3 dans Gazebo | `dddmr_simulation:rl_lite3` | `rl_lite3` |
| 2 | Localisation et navigation 3D avec TensorRT | `dddmr:humble-cuda` | `dddmr_humble_cuda_dev` |

Les commandes ci-dessous utilisent `~/dddmr_navigation` pour le dépôt de navigation et `~/dfl_mobilerobot_simulator` pour le simulateur.

### 1. Créer l’image Docker

Commencez par cloner le dépôt de navigation. Cette étape est nécessaire pour les deux options ci-dessous :

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
```

Choisissez ensuite **une** des méthodes suivantes pour obtenir l’image de navigation.

#### Option A : Construire localement

Lancez le script de construction, choisissez **`x64`**, puis répondez **`Y`** à la question sur CUDA. Cela crée `dddmr:humble-cuda` avec les dépendances CUDA et TensorRT.

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./build.bash
```

#### Option B : Télécharger l’image publique

Une image précompilée est déjà publiée. Téléchargez-la et ajoutez l’étiquette locale attendue par `run_x64_gpu.bash` :

```bash
docker pull tsengapola/dddmr:humble-cuda
docker tag tsengapola/dddmr:humble-cuda dddmr:humble-cuda
```

Quelle que soit l’option choisie, poursuivez avec le téléchargement de la carte. L’espace de travail de navigation sera compilé dans le conteneur du terminal 2. L’image de simulation Lite3 sera construite séparément à l’étape **Terminal 1** ci-dessous.

### 2. Télécharger la carte de navigation

Téléchargez et décompressez la carte `stair_2_map.zip` destinée au monde Lite3. Répondez **`Y`** à l’invite :

```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide
./download_files.bash
```

Le script crée `~/dddmr_bags` et extrait la carte dans `~/dddmr_bags/stair_2_map`. Le script de lancement monte le répertoire des cartes dans le conteneur de navigation sous `/root/dddmr_bags`.

### 3. Préparer l’environnement de démonstration

Ouvrez **deux terminaux distincts** : un pour le simulateur et un pour la navigation. Les deux conteneurs doivent utiliser **`ROS_DOMAIN_ID=22`**.

#### 🖥️ Terminal 1 : Démarrer la simulation Lite3 dans Gazebo

Suivez le [guide Gazebo — Lite3](../GAZEBO_GUIDE.md#lite3--rl-quadruped) pour cloner la branche `rl_lite3`, initialiser les sous-modules, construire l’image et lancer `./rl_lite3_gpu.bash`.

Une fois le robot posé au sol, donnez le focus au terminal du simulateur et appuyez sur **`r` → `0` → `1` → `n`**. Cela réinitialise la pose, fait lever le robot, démarre le contrôleur RL et autorise les commandes de navigation. **Attendez au moins une seconde entre chaque touche ; après `0`, attendez également que le robot ait fini de se lever.**

| Touche | Action |
|:----|:-------|
| `r` | Rétablir la pose initiale du robot dans le monde. |
| `0` | Faire lever le robot et le préparer à marcher. |
| `1` | Activer la politique de locomotion RL. |
| `n` | Activer l’entrée `/cmd_vel` pour permettre à la navigation de contrôler le robot. |

Gardez le terminal de simulation ouvert, puis passez au terminal 2.

#### 🖥️ Terminal 2 : Créer le conteneur de navigation

##### Étape 1 (sur l’hôte) : Démarrer le conteneur CUDA

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./run_x64_gpu.bash
```

Saisissez **`22`** lorsque le script demande `ROS_DOMAIN_ID`. Il crée le conteneur `dddmr_humble_cuda_dev` avec `dddmr:humble-cuda` et monte le dépôt sous `/root/dddmr_navigation`.

##### Étape 2 (dans le conteneur de navigation) : Compiler avec TensorRT

```bash
cd /root/dddmr_navigation
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=22
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DTRT_ENABLED=ON -DTEST_ENABLED=ON
```

`TRT_ENABLED=ON` active le code d’inférence de la navigation. Après un nouveau clonage, générez aussi le moteur TensorRT à partir du modèle fourni, avec le script de conversion du dépôt :

```bash
cd /root/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/engine
bash onnx2trt.bash
cd /root/dddmr_navigation
```

##### Étape 3 (dans le conteneur de navigation) : Lancer la localisation et la navigation

```bash
source install/setup.bash
export ROS_DOMAIN_ID=22
ros2 launch p2p_move_base lite3_localization_launch.py
```

### 4. Lancer la démonstration

1. Vérifiez que Lite3 utilise la politique RL dans le terminal 1 et que l’entrée `/cmd_vel` est activée.
2. Dans RViz, vérifiez que le nuage de points en direct se superpose à la carte fournie. La configuration de démonstration contient une pose initiale pour ce monde. Si le robot a été déplacé ou réinitialisé ailleurs, utilisez **3D Pose Estimate** pour l’aligner.
3. Sélectionnez **3D Goal Pose** dans RViz et choisissez un point accessible au sol. Lite3 suivra les commandes de navigation jusqu’à l’objectif.

<p align="center">
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/3d_nav_rl_quadruped_deep_learning.png?raw=true" alt="Navigation 3D avec apprentissage profond du quadrupède Lite3 RL dans Gazebo et RViz" width="920" />
</p>

---

<p align="center">
  <b>━━━━━━━━━━ 🤖 Tutoriel robot réel ci-dessous 🤖 ━━━━━━━━━━</b>
</p>

---

## ✨ Start DDDMR Navigation with a Real Robot

Ce guide vous accompagne dans la configuration de la **navigation 3D** sur votre robot réel — de la cartographie à la navigation autonome.

DDDMR apporte la navigation 3D à vos robots à roues, quadrupèdes, humanoïdes et plus encore. Mettons votre robot en mouvement !

### 1. Créer l'image Docker

Clonez le dépôt et exécutez ./build.bash, sélectionnez **`x64`** ou **`l4t`** selon votre plateforme.

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

> **Note :** Notre plateforme de test utilise Jetson Orin Nano, nous sélectionnons donc **`l4t`**.

### 2. Plateforme de test

Pour suivre rapidement ce tutoriel, voici la configuration matérielle que nous utilisons :

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/hardware_quadruped.png" width="500" height="250"/>
</p>

| Composant | Modèle |
|:----------|:------|
| Robot | Robot quadrupède (Lite3) |
| LiDAR | LiDAR dôme (RoboSense Airy), incliné à 45° |
| Ordinateur | Jetson Orin Nano |
| Alimentation | Batterie externe pour LiDAR et Jetson |
| Connexion | Ethernet (robot, Jetson et LiDAR communiquent via le réseau) |

> **Note :** Vous n'avez pas besoin exactement du même matériel. Tant que votre système répond aux exigences ci-dessous, DDDMR fonctionnera.

#### Exigences système

| Élément | Topic | Type de message | Description |
|:----:|:------|:-------------|:------------|
| 🎮 | `/cmd_vel` | `geometry_msgs/msg/Twist` | Commande de vitesse pour contrôler le robot |
| 📡 | `/lidar_point_cloud` | `sensor_msgs/msg/PointCloud2` | Nuage de points LiDAR 3D (multicouche, dôme, solide, etc.) |
| 📍 | `/odom` | `nav_msgs/msg/Odometry` | Odométrie avec TF (erreur < 10% préférable) |
| 🌳 | `/tf` | `tf2_msgs/msg/TFMessage` | Arbre TF : `odom` → `base_link` → `lidar_link` |

> **Note :** 
> - DDDMR peut être utilisé sur des **robots à roues, quadrupèdes et humanoïdes** tant qu'ils répondent aux exigences ci-dessus.
> - `/cmd_vel` est l'**entrée** pour le contrôle du mouvement du robot.
> - `/lidar_point_cloud`, `/odom`, `/tf` sont les **sorties** du robot, utilisées par DDDMR pour la cartographie et la localisation.

#### Structure de l'arbre TF

Assurez-vous que le `frame_id` dans le topic LiDAR correspond à votre arbre TF :

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/tf_requirement.png" width="150" height="250"/>
</p>

#### Exemple de configuration TF

Voici la configuration TF en utilisant notre robot quadrupède comme exemple. Le LiDAR est monté à `x=0,2m`, `z=0,22m` de `base_link`, et incliné vers le bas de 45° :

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/setup_lidar_testing_quadruped.png" width="400" height="250"/>
</p>

```xml
<!--- TF: x y z yaw pitch roll -->
<node pkg="tf2_ros" exec="static_transform_publisher" name="sensor2baselink" 
      args="0.2 0.0 0.22 0.0 0.785 0.0 base_link lidar" />
```

> **Note :** 45° ≈ 0,785 radian. Incliné vers le bas.

#### 👉 Avancé (Optionnel)

| Fonctionnalité | Description |
|:--------|:------------|
| [Odométrie 3D](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_odom_3d) | Meilleure localisation et cartographie sur terrain irrégulier |

---

### 3. 🚀 Exécuter cartographie + navigation

DDDMR supporte trois workflows de navigation :

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/dddmr_nav_workflows.png" width="600" height="450"/>
</p>

> **Note :** Ce tutoriel suit le workflow **② Cartographie en ligne + Navigation**.

---

### 3.1 🗺️ Cartographie

Avant de commencer, assurez-vous que votre robot publie les topics requis :

- Topic d'odométrie
- Topic de nuage de points LiDAR
- TF (`odom` → `base_link` → `lidar_link`)

Puis lancez la cartographie :

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_mapping.launch
```

Après le lancement, vous devriez voir l'interface RViz comme suit :

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/mapping_realrobot_rviz.png" width="800" height="500"/>
</p>

Lorsque vous faites circuler votre robot, vous remarquerez :
- **Images clés** — devraient augmenter à mesure que le robot se déplace
- **Points caractéristiques** — caractéristiques extraites de l'environnement
- **Points de sol** — surface du sol détectée
- **Projection 2D** — aide à comprendre la scène et le FOV du LiDAR

#### Sauvegarder la carte

Après avoir terminé la cartographie de la zone, ouvrez un nouveau terminal et exécutez :

```bash
ros2 service call /save_mapped_point_cloud std_srvs/srv/Empty
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/map_save_.png" width="500" height="250"/>
</p>

Lorsque vous voyez `Create dir:` dans le terminal, cela signifie que la carte a été sauvegardée. Si la cartographie est terminée, vous pouvez fermer le nœud de cartographie.

La carte sera sauvegardée dans `/tmp`. Vous pouvez déplacer le dossier vers `/root/dddmr_bags` :

```bash
mv /tmp/2026_03_28_19_25_15/ /root/dddmr_bags/
```

> **Note :** Le nom du dossier `2026_03_28_19_25_15` n'est qu'un exemple. Votre nom de dossier sera différent selon le moment où vous exécutez la cartographie.

---

### 3.2 📍 Localisation + Navigation

D'abord, ouvrez le fichier de configuration et définissez le chemin de votre carte :

📄 `config/airy_tilt45_navigation.yaml`

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/robot_map_location.png" width="500" height="500"/>
</p>

Changez `pose_graph_dir` vers votre dossier de carte :

```yaml
pose_graph_dir: "/root/dddmr_bags/2026_03_28_19_25_15"
```

> **Note :** Remplacez `2026_03_28_19_25_15` par le nom réel de votre dossier de carte.

Puis lancez la localisation et la navigation :

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_navigation.launch
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/LOC_NAV_realrobot_rviz.png" width="800" height="500"/>
</p>

##### Étape 1 : Donner la pose initiale

Cliquez sur **「3D Pose Estimate」** dans la barre d'outils RViz et sélectionnez un point au sol correspondant à la position réelle du robot.

Si la pose initiale est correcte, vous verrez le nuage de points observé bien superposé aux caractéristiques de la carte.

##### Étape 2 : Envoyer l'objectif

Une fois l'initialisation terminée, cliquez sur **「3D Goal Pose」** et sélectionnez un point au sol comme objectif.

Le robot devrait commencer à se déplacer vers l'objectif.
