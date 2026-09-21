# Guide du débutant DDDMR

Ce guide est un tutoriel d'introduction au système de navigation DDDMR. Il comprend un exemple de simulation de robot quadrupède dans Gazebo et un guide de déploiement sur un robot réel, pour vous aider à démarrer rapidement et profiter de la navigation 3D.

## Table des matières

| # | Section | Description |
|:-:|:--------|:------------|
| 1 | [Navigation en simulation Gazebo](#-dddmr-navigation-with-gazebo) | Démo de simulation de robot quadrupède |
| 2 | [Navigation avec robot réel](#-start-dddmr-navigation-with-a-real-robot) | Déploiement sur votre robot réel (robot à roues, quadrupède, humanoïde, etc.) |

## 🖥️ Prérequis logiciels
- **Ubuntu 22.04** (testé sur 22.04, devrait supporter 24.04)
- **Docker** — [Installer Docker](https://docs.docker.com/engine/install/)

## ✨ DDDMR Navigation with Gazebo

Cette démo montre comment exécuter la pile de navigation DDDMR dans Gazebo avec un robot quadrupède.
- Construisez les images nécessaires pour préparer l'environnement.
- Exécutez le système dans deux terminaux — un pour le monde Gazebo, l'autre pour la pile de navigation.

### 1. Créer l'image Docker

Clonez le dépôt et exécutez ./build.bash, sélectionnez **`x64_gz`**, qui contient déjà tous les composants nécessaires pour la navigation et Gazebo.

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

### 2. Télécharger la carte de navigation

Pour exécuter dddmr_navigation dans Gazebo, vous devez télécharger la carte de navigation de démonstration (12,3 Mo).

```bash
cd ~ && mkdir dddmr_bags
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./download_files.bash
```

### 3. Préparer l'environnement de démonstration

#### Créer deux conteneurs Docker (Gazebo et navigation)

> [!IMPORTANT] 
> La commande suivante démarrera deux conteneurs Docker interactifs en utilisant l'image construite. Veuillez ouvrir **deux terminaux séparés** pour préparer l'environnement de démonstration.

#### 🖥️ Terminal 1 : Démarrer la simulation Go2 dans Gazebo

Dans le premier terminal, suivez les deux étapes du [Guide Gazebo — Go2](../GAZEBO_GUIDE.md#go2--quadruped) pour construire l’image de simulation et démarrer le conteneur Gazebo.

Une fois la simulation Go2 lancée, laissez ce terminal ouvert et revenez ici pour poursuivre avec le **Terminal 2** ci-dessous.

#### 🖥️ Terminal 2 (créer le conteneur pour le système de navigation)

- ##### Étape 1 (sur l'hôte) : créer le conteneur pour le système de navigation
```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./run_x64_navigation.bash
```

- ##### Étape 2 (dans le conteneur de navigation) : construire et lancer
```bash
cd dddmr_navigation/ && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash && ros2 launch p2p_move_base go2_localization.launch
```

### 4. Exécuter la démo

- Dans la démo Gazebo, la carte est déjà alignée, donc vous n'avez pas besoin de définir la pose initiale sauf si vous cartographiez vous-même
- Donnez l'objectif (3D Goal Pose) dans RViz, et le robot se déplacera vers le point cible

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/give_goal_in_demo_.png" width="920" height="460"/>
</p>

### 5. Problèmes connus

> [!WARNING]
> Les comportements suivants sont actuellement observés, sont en cours d'investigation et seront corrigés dans les futures mises à jour.
> - Gazebo : glissement occasionnel sur les pentes
> - Cartographie : couches de sol dupliquées

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
