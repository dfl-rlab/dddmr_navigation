# Guia para Iniciantes DDDMR

Este guia é um tutorial introdutório ao sistema de navegação DDDMR. Inclui um exemplo de simulação de robô quadrúpede no Gazebo e um guia de implantação em robô real, para ajudá-lo a começar rapidamente e aproveitar a navegação 3D.

## Índice

| # | Seção | Descrição |
|:-:|:--------|:------------|
| 1 | [Navegação em simulação Gazebo](#-dddmr-navigation-with-gazebo) | Demo de simulação de robô quadrúpede |
| 2 | [Navegação com robô real](#-start-dddmr-navigation-with-a-real-robot) | Implantação no seu robô real (robô com rodas, quadrúpede, humanoide, etc.) |

## 🖥️ Requisitos de Software
- **Ubuntu 22.04** (testado em 22.04, deve suportar 24.04)
- **Docker** — [Instalar Docker](https://docs.docker.com/engine/install/)

## ✨ DDDMR Navigation with Gazebo

Esta demo mostra como executar a pilha de navegação DDDMR no Gazebo com um robô quadrúpede.
- Construa as imagens necessárias para preparar o ambiente.
- Execute o sistema em dois terminais — um para o mundo Gazebo, outro para a pilha de navegação.

### 1. Criar imagem Docker

Clone o repositório e execute ./build.bash, selecione **`x64_gz`**, que já contém todos os componentes necessários para navegação e Gazebo.

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

### 2. Baixar mapa de navegação

Para executar dddmr_navigation no Gazebo, você precisa baixar o mapa de navegação de demonstração (12,3MB).

```bash
cd ~ && mkdir dddmr_bags
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./download_files.bash
```

### 3. Preparar ambiente de demonstração

#### Criar dois contêineres Docker (Gazebo e navegação)

> [!IMPORTANT] 
> O comando a seguir iniciará dois contêineres Docker interativos usando a imagem construída. Por favor, abra **dois terminais separados** para preparar o ambiente de demonstração.

#### 🖥️ Terminal 1: Iniciar a simulação do Go2 no Gazebo

No primeiro terminal, siga as duas etapas do [Guia do Gazebo — Go2](../GAZEBO_GUIDE.md#go2--quadruped) para criar a imagem de simulação e iniciar o contêiner do Gazebo.

Quando a simulação do Go2 estiver em execução, mantenha este terminal aberto e volte aqui para continuar com o **Terminal 2** abaixo.

#### 🖥️ Terminal 2 (criar contêiner para sistema de navegação)

- ##### Passo 1 (no host): criar contêiner para sistema de navegação
```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide && ./run_x64_navigation.bash
```

- ##### Passo 2 (dentro do contêiner de navegação): construir e lançar
```bash
cd dddmr_navigation/ && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash && ros2 launch p2p_move_base go2_localization.launch
```

### 4. Executar demo

- Na demo do Gazebo, o mapa já está alinhado, então você não precisa definir a pose inicial a menos que esteja mapeando você mesmo
- Dê o objetivo (3D Goal Pose) no RViz, e o robô se moverá para o ponto alvo

<p align='center'>
    <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/give_goal_in_demo_.png" width="920" height="460"/>
</p>

### 5. Problemas conhecidos

> [!WARNING]
> Os seguintes são comportamentos observados atualmente, estão sendo investigados e serão corrigidos em atualizações futuras.
> - Gazebo: deslizamento ocasional em declives
> - Mapeamento: camadas de piso duplicadas

---

<p align="center">
  <b>━━━━━━━━━━ 🤖 Tutorial de robô real abaixo 🤖 ━━━━━━━━━━</b>
</p>

---

## ✨ Start DDDMR Navigation with a Real Robot

Este guia o conduzirá pela configuração de **navegação 3D** no seu robô real — do mapeamento à navegação autônoma.

DDDMR traz navegação 3D para seus robôs com rodas, quadrúpedes, humanoides e mais. Vamos colocar seu robô em movimento!

### 1. Criar imagem Docker

Clone o repositório e execute ./build.bash, selecione **`x64`** ou **`l4t`** dependendo da sua plataforma.

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```

> **Nota:** Nossa plataforma de teste usa Jetson Orin Nano, então selecionamos **`l4t`**.

### 2. Plataforma de teste

Para seguir rapidamente este tutorial, aqui está a configuração de hardware que usamos:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/hardware_quadruped.png" width="500" height="250"/>
</p>

| Componente | Modelo |
|:----------|:------|
| Robô | Robô quadrúpede (Lite3) |
| LiDAR | LiDAR domo (RoboSense Airy), inclinado 45° |
| Computador | Jetson Orin Nano |
| Alimentação | Bateria externa para LiDAR e Jetson |
| Conexão | Ethernet (robô, Jetson e LiDAR se comunicam pela rede) |

> **Nota:** Você não precisa exatamente do mesmo hardware. Desde que seu sistema atenda aos requisitos abaixo, DDDMR funcionará.

#### Requisitos do sistema

| Item | Tópico | Tipo de mensagem | Descrição |
|:----:|:------|:-------------|:------------|
| 🎮 | `/cmd_vel` | `geometry_msgs/msg/Twist` | Comando de velocidade para controlar o robô |
| 📡 | `/lidar_point_cloud` | `sensor_msgs/msg/PointCloud2` | Nuvem de pontos LiDAR 3D (multicamada, domo, sólido, etc.) |
| 📍 | `/odom` | `nav_msgs/msg/Odometry` | Odometria com TF (erro < 10% preferível) |
| 🌳 | `/tf` | `tf2_msgs/msg/TFMessage` | Árvore TF: `odom` → `base_link` → `lidar_link` |

> **Nota:** 
> - DDDMR pode ser usado em **robôs com rodas, quadrúpedes e humanoides** desde que atendam aos requisitos acima.
> - `/cmd_vel` é a **entrada** para controle de movimento do robô.
> - `/lidar_point_cloud`, `/odom`, `/tf` são **saídas** do robô, usadas pelo DDDMR para mapeamento e localização.

#### Estrutura da árvore TF

Certifique-se de que o `frame_id` no tópico LiDAR corresponda à sua árvore TF:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/tf_requirement.png" width="150" height="250"/>
</p>

#### Exemplo de configuração TF

Aqui está a configuração TF usando nosso robô quadrúpede como exemplo. O LiDAR está montado em `x=0,2m`, `z=0,22m` do `base_link`, e inclinado para baixo em 45°:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/setup_lidar_testing_quadruped.png" width="400" height="250"/>
</p>

```xml
<!--- TF: x y z yaw pitch roll -->
<node pkg="tf2_ros" exec="static_transform_publisher" name="sensor2baselink" 
      args="0.2 0.0 0.22 0.0 0.785 0.0 base_link lidar" />
```

> **Nota:** 45° ≈ 0,785 radianos. Inclinação para baixo.

#### 👉 Avançado (Opcional)

| Recurso | Descrição |
|:--------|:------------|
| [Odometria 3D](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_odom_3d) | Melhor localização e mapeamento em terreno irregular |

---

### 3. 🚀 Executar mapeamento + navegação

DDDMR suporta três fluxos de trabalho de navegação:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/dddmr_nav_workflows.png" width="600" height="450"/>
</p>

> **Nota:** Este tutorial segue o fluxo de trabalho **② Mapeamento online + Navegação**.

---

### 3.1 🗺️ Mapeamento

Antes de começar, certifique-se de que seu robô está publicando os tópicos necessários:

- Tópico de odometria
- Tópico de nuvem de pontos LiDAR
- TF (`odom` → `base_link` → `lidar_link`)

Então lance o mapeamento:

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_mapping.launch
```

Após o lançamento, você deve ver a interface RViz assim:

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/mapping_realrobot_rviz.png" width="800" height="500"/>
</p>

Quando você dirigir seu robô, notará:
- **Keyframes** — devem aumentar à medida que o robô se move
- **Pontos característicos** — características extraídas do ambiente
- **Pontos de chão** — superfície do chão detectada
- **Projeção 2D** — ajuda a entender a cena e o FOV do LiDAR

#### Salvar o mapa

Após terminar o mapeamento da área, abra um novo terminal e execute:

```bash
ros2 service call /save_mapped_point_cloud std_srvs/srv/Empty
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/map_save_.png" width="500" height="250"/>
</p>

Quando você ver `Create dir:` no terminal, significa que o mapa foi salvo. Se o mapeamento estiver completo, você pode fechar o nó de mapeamento.

O mapa será salvo em `/tmp`. Você pode mover a pasta para `/root/dddmr_bags`:

```bash
mv /tmp/2026_03_28_19_25_15/ /root/dddmr_bags/
```

> **Nota:** O nome da pasta `2026_03_28_19_25_15` é apenas um exemplo. O nome da sua pasta será diferente com base em quando você executar o mapeamento.

---

### 3.2 📍 Localização + Navegação

Primeiro, abra o arquivo de configuração e defina o caminho do seu mapa:

📄 `config/airy_tilt45_navigation.yaml`

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/robot_map_location.png" width="500" height="500"/>
</p>

Altere `pose_graph_dir` para sua pasta de mapa:

```yaml
pose_graph_dir: "/root/dddmr_bags/2026_03_28_19_25_15"
```

> **Nota:** Substitua `2026_03_28_19_25_15` pelo nome real da sua pasta de mapa.

Então lance a localização e navegação:

```bash
ros2 launch dddmr_beginner_guide airy_tilt45_navigation.launch
```

<p align='center'>
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/LOC_NAV_realrobot_rviz.png" width="800" height="500"/>
</p>

##### Passo 1: Dar pose inicial

Clique em **「3D Pose Estimate」** na barra de ferramentas do RViz e selecione um ponto no chão que corresponda à posição real do robô.

Se a pose inicial estiver correta, você verá a nuvem de pontos observada sobreposta bem às características do mapa.

##### Passo 2: Enviar objetivo

Uma vez que a inicialização esteja concluída, clique em **「3D Goal Pose」** e selecione um ponto no chão como objetivo.

O robô deve começar a se mover em direção ao objetivo.
