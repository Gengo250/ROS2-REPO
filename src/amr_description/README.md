# AMR warehouse — versão inicial

Modelo próprio, procedural e modular para **Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic**.
Inspiração genérica em AMRs industriais baixos; sem geometria proprietária ou marcas Amazon.
O workspace real desta máquina é **`~/ros2_ws`**, confirmado durante a implementação.
Nenhum arquivo dos pacotes `my_cpp_pkg` e `my_py_pkg` foi modificado; `warehouse_amr/` foi preservado.

## Reconstruir tudo

```bash
cd ~/ros2_ws
bash src/amr_description/scripts/build_amr.sh
```

O comando lê o JSON, gera/valida a cena, exporta oito STLs, valida Xacro/URDF/SDF,
compila **todo o workspace**, testa RViz e Gazebo em ambientes isolados, executa testes
de sensores/movimento/parada e produz os cinco renders. Falhas interrompem a execução.
Também testa duas reconstruções byte a byte, uma variação de dimensões e a preservação de cena externa.
As instâncias usadas no teste são encerradas ao terminar; para manter as interfaces abertas,
use os launches abaixo. Os testes usam domínio ROS e partição Gazebo próprios.

```bash
# Apenas reconstrução e verificações estáticas; não declara aceitação de runtime:
bash src/amr_description/scripts/build_amr.sh --skip-runtime
# Omitir somente os renders:
bash src/amr_description/scripts/build_amr.sh --no-renders
```

Requisitos além de ROS/Gazebo: Blender, `colcon`, `xacro`, `check_urdf`,
`robot_state_publisher`, `joint_state_publisher`, `rviz2`, `ros_gz_sim`,
`ros_gz_bridge`, `gz_ros2_control`, `controller_manager`,
`joint_state_broadcaster` e `diff_drive_controller`.
O teste automatizado de RViz usa `xvfb`, `xwininfo` e `python3-pyqt5`;
eles não são necessários para abrir o RViz normalmente. Nenhuma automação de mouse/teclado é usada.

## Fonte única de parâmetros

Edite **`config/amr_dimensions.json`** e rode novamente o comando mestre.
`scripts/amr_parameters.py` valida as relações geométricas e deriva coordenadas, limites e controle.
`urdf/dimensions.xacro`, `config/controllers.yaml` e `config/joint_limits.yaml` são gerados;
não devem ser editados diretamente. Esses YAMLs usam a sintaxe JSON, válida em YAML.
`validation/meshes.json` registra o hash do JSON, dimensões e contagem de triângulos.

| Parâmetro | Valor inicial |
|---|---:|
| Chassi C × L × A | 0,95 × 0,70 × 0,25 m |
| Plataforma C × L × espessura | 0,88 × 0,64 × 0,055 m |
| Separação vertical plataforma/chassi | 0,004 m |
| Altura total no chão | 0,344 m |
| Clearance do chassi | 0,035 m |
| Raio / largura das rodas | 0,12 / 0,055 m |
| Distância entre centros das rodas | 0,58 m |
| Raio dos apoios esféricos | 0,032 m |
| Centros dos apoios em X | ±0,345 m |
| Massa total sem carga | 84,45 kg |
| Massa chassi / plataforma / cada roda | 65 / 12 / 2,5 kg |
| Limite linear / angular | 0,7 m/s / 1,5 rad/s |
| Timeout de comando | 0,5 s |

As massas são **estimativas do nosso protótipo**, sem relação com dados oficiais de outros robôs.
O COM do chassi está a 0,105 m do chão; as inércias são aproximações analíticas positivas.
Os três links virtuais de sensores e `base_link` não têm massa própria; todos os componentes
físicos têm inércia. A câmera possui lentes com projeção de 1 mm além da frente do chassi.

## Coordenadas, origens e TF

**X forward, Y left, Z up; meters.** Blender Metric, Length Meters, Unit Scale 1.
`base_link` fica em `(0, 0, 0)` no plano do chão, no meio do eixo motriz.
`chassis_link` fica no centro geométrico da carenagem, Z=0,16 m.
Todas as meshes são locais ao link, com escala e rotação de objeto identidades.
As rodas já têm seu eixo geométrico ao longo de Y: o visual não recebe rotação extra.
Nos cilindros URDF de colisão/inércia, a rotação de 90° converte o eixo nativo Z em Y.
Ambos os joints motrizes usam eixo `0 1 0`: velocidades positivas movem o robô para +X.

```text
base_link
├── [base_to_chassis, fixed] chassis_link
│   ├── [left_wheel_joint, continuous] left_wheel_link
│   ├── [right_wheel_joint, continuous] right_wheel_link
│   ├── [front_caster_joint, fixed] front_caster_link
│   ├── [rear_caster_joint, fixed] rear_caster_link
│   ├── [bumper_front_joint, fixed] bumper_front_link
│   ├── [bumper_rear_joint, fixed] bumper_rear_link
│   ├── [lidar_joint, fixed] lidar_link
│   │   └── [laser_frame_joint, fixed] laser_frame
│   ├── [imu_joint, fixed] imu_link
│   └── [camera_joint, fixed] camera_link
│       ├── [camera_optical_joint, fixed] camera_optical_frame
│       └── [camera_depth_joint, fixed] camera_depth_frame
└── [platform_joint, fixed] platform_link
```

São **15 links e 14 joints**. Na simulação, o controlador acrescenta `odom → base_link`.
O link da IMU é filho do chassi, mantendo seu posicionamento ligado ao corpo físico.
O SDFormat agrega os links fixos para física; o `robot_state_publisher` mantém a árvore ROS completa.

`camera_optical_frame` segue REP-103: X direita, Y baixo, Z frente, rotação RPY
`(-pi/2, 0, -pi/2)`. O `camera_depth_frame` tem a mesma origem da lente, porém eixos ROS.
Isso é necessário porque o **PointCloudPacked da instalação Harmonic testada contém X para frente**,
mesmo quando o sensor recebe `optical_frame_id`. A entrada da nuvem em `bridge.yaml`
define `frame_id: camera_depth_frame`; não altera coordenadas. RGB, depth e CameraInfo
continuam no frame óptico. O teste verifica um ponto real contra a profundidade e o obstáculo,
detectando uma eventual mudança desse comportamento ao atualizar Gazebo/ros_gz.

## Blender e exportação

```bash
cd ~/ros2_ws/src/amr_description
blender --background --factory-startup --python-exit-code 1 \
  --python scripts/blender/generate_amr.py

# Validar o .blend e os STLs:
blender --background blender/warehouse_amr.blend --python-exit-code 1 \
  --python scripts/blender/validate_amr.py

# Reexportar somente peças de uma cena atualizada:
blender --background blender/warehouse_amr.blend --python-exit-code 1 \
  --python scripts/blender/export_amr.py

# Refazer somente os renders:
blender --background blender/warehouse_amr.blend --python-exit-code 1 \
  --python scripts/blender/render_amr.py
```

Veja [scripts/blender/README.md](scripts/blender/README.md) para execução por MCP.
O `.blend` é um resultado reproduzível. O gerador só limpa a cena identificada
`AMR_Generation`; cenas alheias são preservadas. Não altere manualmente essa cena
como fonte de verdade: qualquer ajuste deve voltar ao Python/JSON.

O exportador detecta a versão do Blender: `bpy.ops.wm.stl_export` em 4+ e o exportador
STL legado em 3.x. Modifiers são aplicados a cópias avaliadas de cada peça; a cena
permanece editável. `forward_axis=Y` / `up_axis=Z` significa **transformação identidade
no exportador STL do Blender**, preservando os vértices X-frente que o gerador já produziu.
O validador lê o STL binário e confere dimensão, eixo e origem, incluindo a largura da roda em Y.
Blender 5.2.2 foi executado; o ramo de compatibilidade 3.x não foi testado nesta máquina.

## Xacro, build e RViz

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
python3 src/amr_description/scripts/amr_parameters.py
python3 src/amr_description/scripts/validate_urdf.py
colcon build --symlink-install
source install/setup.bash
xacro src/amr_description/urdf/amr.urdf.xacro > /tmp/amr.urdf
check_urdf /tmp/amr.urdf
ros2 launch amr_description display.launch.py
```

O launch abre `robot_state_publisher`, `joint_state_publisher` e RViz. A malha da grade é 0,1 m.
Use `rviz:=false` se quiser somente os publishers. Não mantenha esse launch junto do de
simulação no mesmo domínio: na simulação, os joints são publicados pelo `joint_state_broadcaster`.

## Gazebo Harmonic

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch amr_description sim.launch.py
# Renderer alternativo para a janela (validado também em Xvfb):
ros2 launch amr_description sim.launch.py gui_renderer:=ogre
# Gazebo + RViz:
ros2 launch amr_description sim.launch.py rviz:=true
# Servidor sem janela, ainda com sensores GPU:
ros2 launch amr_description sim.launch.py gui:=false
```

O mundo local contém chão e um obstáculo de referência a X=3 m, sem downloads do Fuel.
O servidor sempre usa Ogre2/EGL headless; a janela é um cliente independente. O renderer
da janela é selecionável (`gui_renderer:=ogre2` ou `ogre`), ambos do Gazebo Harmonic.
Nesta máquina, Ogre2 com o display virtual Xvfb/llvmpipe falhou dentro de Mesa/EGL;
a captura automatizada usa Ogre para a janela e mantém os sensores no servidor Ogre2.
Isso não usa Gazebo Classic. A configuração `gazebo_gui.config` já enquadra o AMR.
O spawn começa 15 mm acima do chão e assenta por gravidade.
`gz_ros2_control/GazeboSimSystem` e `libgz_ros2_control-system.so` conectam as rodas
ao `diff_drive_controller`. Não há plugin Gazebo Classic ou outro controlador de tração concorrente.
Os spawners só começam depois do spawn; o remapeamento é passado por `--controller-ros-args`.
Os limites URDF e as rampas de velocidade do controlador estão habilitados.

```bash
# Em outro terminal com os mesmos sources; Ctrl+C encerra a publicação e o timeout freia:
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: auto, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}' \
  --rate 20 --ros-args -p use_sim_time:=true

ros2 control list_controllers
# Teste completo isolado, inicia e encerra suas próprias instâncias:
python3 ~/ros2_ws/src/amr_description/scripts/smoke_test.py
```

No Jazzy, esse controlador recebe **TwistStamped**, não Twist. Ao integrar Nav2,
alinhe o tipo de saída do pipeline de velocidades ou acrescente um adaptador explícito.
Não existe bridge para `/cmd_vel` nem `/odom`: esses tópicos já são ROS via ros2_control.

| Tópico ROS | Tipo | Fonte / frame |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | entrada do controlador |
| `/odom` | `nav_msgs/msg/Odometry` | encoders; `odom → base_link` |
| `/joint_states` | `sensor_msgs/msg/JointState` | joint_state_broadcaster |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | controlador e state publisher |
| `/clock` | `rosgraph_msgs/msg/Clock` | relógio Gazebo |
| `/scan` | `sensor_msgs/msg/LaserScan` | `/amr/scan`; `laser_frame` |
| `/imu/data` | `sensor_msgs/msg/Imu` | `/amr/imu`; `imu_link` |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | `/amr/camera/image`; óptico |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | `/amr/camera/depth_image`; óptico, `32FC1` em metros |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | `/amr/camera/camera_info`; óptico |
| `/camera/points` | `sensor_msgs/msg/PointCloud2` | `/amr/camera/points`; `camera_depth_frame` |

LiDAR: 361 raios, 180° frontais, 15 Hz, alcance 0,08–20 m.
IMU: 100 Hz. RGB-D: 640×480, 15 Hz, FOV horizontal 75°, alcance 0,08–10 m.
Os sensores usam QoS Sensor Data na bridge. Consumidores precisam aceitar Best Effort.

## Geometria e limitações físicas

- Colisão do chassi: três caixas; plataforma e housings: primitivas; rodas: cilindros;
  apoios passivos: esferas de baixo atrito. Nenhum STL é usado como colisão.
- As caixas simplificam os cantos e omitem parte da borda externa superior do chassi.
  Não representam fielmente cada rebaixo; uma futura análise de contatos pode refiná-las.
- Os apoios esféricos fixos aproximam ball casters; não simulam swivel, rolamentos ou suspensão.
- As massas, inércias e atritos precisam de identificação no hardware. Não há carga na plataforma.
- O LiDAR é frontal (180°), não um scanner de segurança de cobertura integral. Sem lógica de bumper.
- Sensores ideais, sem calibração/ruído realista. RGB-D não modela oclusão estéreo interna.
- Integração Nav2/SLAM preparada por frames e tópicos; mapas, localização, footprint,
  costmaps e navegação autônoma ainda não são entregues por este pacote.
- Modelo visual simples, com identidade grafite/cinza/ciano. STL não retém materiais;
  as cores são declaradas novamente no URDF e convertidas para SDF.

O log Harmonic pode emitir avisos de `gz_frame_id` como extensão preservada do SDFormat.
O controlador de 100 Hz roda a cada dez passos de física de 1 ms; o plugin avisa sobre essa
diferença intencional. A IMU é publicada pela bridge, portanto não é registrada como interface
de hardware ros2_control. Avisos de inicialização do Resource Manager e métricas sem executor
nessa versão são registrados; os testes verificam ativação e dados reais. Timeout de comando
sem publicador é a frenagem esperada. Erros de plugin, malha, TF ou instabilidade não são aceitos.

## Artefatos e evidência

```text
amr_description/
├── CMakeLists.txt, package.xml, README.md, .gitignore
├── config/                 # JSON fonte + limites/controle gerados + bridge
├── scripts/
│   ├── amr_parameters.py, build_amr.sh
│   ├── validate_urdf.py, validate_display.py, validate_runtime.py, smoke_test.py
│   └── blender/            # generate, export, validate, render e README
├── urdf/
│   ├── amr.urdf.xacro, dimensions.xacro, inertials.xacro
│   ├── components/         # chassis, platform, wheels, casters
│   ├── collision/          # chassis, platform, wheels
│   ├── sensors/            # lidar, imu, camera
│   └── gazebo/             # properties, sensors, ros2_control
├── meshes/visual/          # oito STLs locais às peças
├── blender/warehouse_amr.blend
├── renders/                # front, rear, side, top, perspective.png
├── launch/                 # display.launch.py, sim.launch.py
├── rviz/amr.rviz
├── worlds/warehouse_test.sdf
└── validation/             # relatórios JSON, captura RViz; logs ignorados pelo Git
```

`validation/runtime.json` mede contato com o chão, atitude, dados atuais dos oito tópicos,
distâncias ao obstáculo, eixos da nuvem, deslocamento físico versus odometria, giro e timeout.
`validation/display.json` verifica a árvore completa em execução; `validation/rviz.png`
registra a visualização. `validation/gazebo.png` e `gazebo_gui.json` registram a interface
Harmonic; `validation/rebuild.json` registra a reconstrução determinística e a variante.
Os cinco renders são produzidos por Python com câmeras/luzes temporárias.

Próximos passos: definir footprint e limites com carga; configurar SLAM Toolbox e Nav2;
calibrar sensores; substituir o hardware simulado por uma interface ros2_control real;
refinar as colisões após testes de aproximação a obstáculos.

Referências técnicas: [gz_ros2_control Jazzy](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html),
[diff_drive_controller Jazzy](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html),
[sensores Harmonic](https://gazebosim.org/docs/harmonic/sensors/).
