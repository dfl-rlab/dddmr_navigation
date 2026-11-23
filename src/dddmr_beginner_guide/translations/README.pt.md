# Guia para Iniciantes DDDMR

Este guia apresenta o DDDMR Navigation para iniciantes. A simulação do Lite3 no Gazebo e as instruções para robôs reais ajudam você a começar rapidamente a explorar a navegação 3D.

## Índice

| # | Seção | Descrição |
|:-:|:--------|:------------|
| 1 | [Navegação em simulação Gazebo](#-dddmr-navigation-with-gazebo) | Demo de simulação do quadrúpede Lite3 com RL |
| 2 | [Navegação com robô real](#-start-dddmr-navigation-with-a-real-robot) | Implantação no seu robô real (robô com rodas, quadrúpede, humanoide, etc.) |

## 🖥️ Requisitos de Software
- **Ubuntu 22.04** (testado; máquinas com Ubuntu 24.04 também podem executar os contêineres Humble).
- **Docker** — [Instalar Docker](https://docs.docker.com/engine/install/)
- **GPU NVIDIA e NVIDIA Container Toolkit** para a simulação com GPU e a demo de navegação com TensorRT.

## ✨ DDDMR Navigation with Gazebo

Execute o DDDMR Navigation com o **quadrúpede Lite3 controlado por RL**, um LiDAR do tipo domo e um mundo Gazebo com vários níveis. A demo usa duas imagens e contêineres separados:

| Terminal | Finalidade | Imagem | Contêiner |
|:---------|:--------|:------|:----------|
| 1 | Simulação do Lite3 no Gazebo | `dddmr_simulation:rl_lite3` | `rl_lite3` |
| 2 | Localização e navegação 3D com TensorRT | `dddmr:humble-cuda` | `dddmr_humble_cuda_dev` |

Os comandos abaixo usam `~/dddmr_navigation` para o repositório de navegação e `~/dfl_mobilerobot_simulator` para o simulador.

### 1. Criar a imagem Docker

Primeiro, clone o repositório de navegação. Esta etapa é necessária para ambas as opções abaixo:

```bash
cd ~
git clone https://github.com/dfl-rlab/dddmr_navigation.git
```

Em seguida, escolha **uma** das formas abaixo para obter a imagem de navegação.

#### Opção A: Construir localmente

Execute o script de construção, selecione **`x64`** e responda **`Y`** à pergunta sobre CUDA. Isso cria `dddmr:humble-cuda`, incluindo as dependências de CUDA e TensorRT.

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./build.bash
```

#### Opção B: Baixar a imagem pública

Uma imagem pré-compilada já está publicada. Baixe-a e adicione a tag local esperada por `run_x64_gpu.bash`:

```bash
docker pull tsengapola/dddmr:humble-cuda
docker tag tsengapola/dddmr:humble-cuda dddmr:humble-cuda
```

Após qualquer uma das opções, continue com o download do mapa. O workspace de navegação será compilado dentro do contêiner no terminal 2. A imagem de simulação do Lite3 será construída separadamente na etapa **Terminal 1** abaixo.

### 2. Baixar o mapa de navegação

Baixe e extraia o mapa `stair_2_map.zip` para o mundo do Lite3. Responda **`Y`** quando solicitado:

```bash
cd ~/dddmr_navigation/src/dddmr_beginner_guide
./download_files.bash
```

O script cria `~/dddmr_bags` e extrai o mapa em `~/dddmr_bags/stair_2_map`. O script de inicialização monta o diretório de mapas em `/root/dddmr_bags` dentro do contêiner de navegação.

### 3. Preparar o ambiente de demonstração

Abra **dois terminais separados**: um para o simulador e outro para a navegação. Ambos os contêineres devem usar **`ROS_DOMAIN_ID=22`**.

#### 🖥️ Terminal 1: Iniciar a simulação do Lite3 no Gazebo

Siga o [guia do Gazebo — Lite3](../GAZEBO_GUIDE.md#lite3--rl-quadruped) para clonar a branch `rl_lite3`, inicializar os submódulos, construir a imagem e executar `./rl_lite3_gpu.bash`.

Quando o robô estiver no chão, mantenha o foco no terminal do simulador e pressione **`r` → `0` → `1` → `n`**. Isso redefine a pose, levanta o robô, inicia o controlador RL e permite receber comandos de navegação. **Espere pelo menos um segundo entre as teclas; após `0`, espere também o robô terminar de se levantar.**

| Tecla | Ação |
|:----|:-------|
| `r` | Redefinir o robô para sua pose inicial no mundo. |
| `0` | Levantar o robô e prepará-lo para caminhar. |
| `1` | Ativar a política de locomoção RL. |
| `n` | Ativar a entrada `/cmd_vel` para que a navegação controle o robô. |

Mantenha o terminal de simulação aberto e continue no terminal 2.

#### 🖥️ Terminal 2: Criar o contêiner de navegação

##### Etapa 1 (na máquina host): Iniciar o contêiner CUDA

```bash
cd ~/dddmr_navigation/dddmr_docker/docker_file
./run_x64_gpu.bash
```

Digite **`22`** quando o script solicitar `ROS_DOMAIN_ID`. Ele cria o contêiner `dddmr_humble_cuda_dev` usando `dddmr:humble-cuda` e monta o repositório em `/root/dddmr_navigation`.

##### Etapa 2 (dentro do contêiner de navegação): Compilar com TensorRT

```bash
cd /root/dddmr_navigation
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=22
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DTRT_ENABLED=ON -DTEST_ENABLED=ON
```

`TRT_ENABLED=ON` ativa o código de inferência da navegação. Em um repositório recém-clonado, gere também o mecanismo TensorRT a partir do modelo incluído, usando o script de conversão:

```bash
cd /root/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/engine
bash onnx2trt.bash
cd /root/dddmr_navigation
```

##### Etapa 3 (dentro do contêiner de navegação): Iniciar localização e navegação

```bash
source install/setup.bash
export ROS_DOMAIN_ID=22
ros2 launch p2p_move_base lite3_localization_launch.py
```

### 4. Executar a demo

1. Confirme que o Lite3 está no modo de política RL no terminal 1 e que a entrada `/cmd_vel` está ativada.
2. No RViz, confira se a nuvem de pontos ao vivo se sobrepõe ao mapa fornecido. A configuração da demo inclui uma pose inicial para este mundo. Se o robô foi movido ou redefinido em outro local, use **3D Pose Estimate** para alinhá-lo.
3. Selecione **3D Goal Pose** no RViz e escolha um ponto alcançável no chão. O Lite3 seguirá os comandos de navegação até o destino.

<p align="center">
  <img src="https://github.com/dfl-rlab/dddmr_documentation_materials/blob/main/dddmr_beginner_guide/3d_nav_rl_quadruped_deep_learning.png?raw=true" alt="Navegação 3D com aprendizado profundo do quadrúpede Lite3 RL no Gazebo e no RViz" width="920" />
</p>

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
