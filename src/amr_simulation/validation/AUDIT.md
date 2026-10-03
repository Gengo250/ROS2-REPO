# Auditoria antes da implementação — 2026-09-30

Workspace `/home/gengo/ros2_ws`, Git com alterações preexistentes em amr_description/validation/gazebo.png, runtime.json e urdf.json; preservadas.
Nenhum AGENTS.md aplicável. Ferramentas: shell/filesystem local, busca web,
conector GitHub disponível; nenhum MCP Blender identificado; Blender CLI local disponível.
Não há necessidade de assets externos, Blender ou acesso ao GitHub para este galpão.

Pacotes existentes: `amr_description` (ament_cmake), `my_cpp_pkg` (ament_cmake),
`my_py_pkg` (ament_python). `src/warehouse_amr` não é pacote ROS.
Nenhum `amr_simulation` ou outro pacote de ambientes existente.

`amr_description/launch/sim.launch.py` inicia ros_gz_sim/gz_sim.launch.py,
servidor separado `-s --headless-rendering`, mundo `worlds/warehouse_test.sdf`
(nome `amr_validation`). GUI separada; Ogre validado, sensores sempre Ogre2.
`ros_gz_sim create` consome `/robot_description`, nome `warehouse_amr`, z=0.015.
Após spawn, ativa joint_state_broadcaster e diff_drive_controller em grupo.
`gz_ros2_control` usa config/controllers.yaml, derivado de amr_dimensions.json.
Comandos ROS `/cmd_vel`: geometry_msgs/msg/TwistStamped; timeout 0.5 s.
Odometria `/odom`: nav_msgs/msg/Odometry, rodas com position_feedback, TF odom→base_link.
Não há bridge de velocidade/odometria: são tópicos do ros2_control.

LiDAR: GPU (`gpu_lidar`), 361 amostras, 180°, 15 Hz, alcance 0.08–20 m,
frame `laser_frame`, posição x=0.440, z=0.213 m. Mede geometria visual;
colisão precisa ser definida separadamente. Bridge /amr/scan → /scan.
RGB-D 640×480, 15 Hz, FOV 75°, alcance 0.08–10 m. RGB, depth e CameraInfo
em camera_optical_frame; PointCloud2 X-forward em camera_depth_frame.
IMU 100 Hz em imu_link, bridge /amr/imu → /imu/data.
Bridge do relógio e sensores: config/bridge.yaml. TF: robot_state_publisher.

`display.launch.py`: xacro sem Gazebo, joint_state_publisher e RViz opcionais.
Dependências existentes incluem ros_gz_sim, ros_gz_bridge, gz_ros2_control,
controller_manager, diff_drive_controller, robot_state_publisher, xacro,
rclpy, sensor_msgs, geometry_msgs, nav_msgs, tf2_ros e ferramentas Blender.

Testes existentes executados ANTES das alterações de implementação:

- `colcon test`: my_py_pkg: 1 aprovado, 1 skipped (copyright), 1 falha flake8,
  12 problemas de estilo em três exemplos Python. Evidência baseline_pytest.xml.
  Pacotes CMake originais não registram testes CTest.
- `validate_urdf.py`: aprovado (URDF display/sim e SDF).
- `smoke_test.py`: aprovado, incluindo display/RViz, TF, Gazebo headless,
  todos os sensores, spawn, linha reta, giro, colisão com piso e timeout.
- `validate_gazebo_gui.py`: aprovado Ogre GUI + Ogre2 sensores, sem interação UI.
- Evidências copiadas para baseline_amr_runtime.json e baseline_amr_gui.json.

Decisão: criar amr_simulation para configuração, geração, modelos, mundos e
aceitação do galpão; estender sim.launch.py apenas com argumentos opcionais
de world/spawn/GUI/resources, preservando defaults. Não duplicar URDF ou controle.
Não modificar os exemplos Python para mascarar falha anterior e não relacionada.

Referências oficiais consultadas:

- https://gazebosim.org/docs/harmonic/actors/ — actors não recebem forças nem
  colisões, mas são visíveis em RGB e sensores GPU; não bastam como obstáculos.
- https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1TrajectoryFollower.html
  — movimento físico 2D por forças/torques; waypoints em coordenadas de mundo.
- https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/src/systems/trajectory_follower/TrajectoryFollower.cc
  — implementação e tópico de pausa; sem garantia de cronograma ao sofrer contato.
- https://gazebosim.org/docs/harmonic/sensors/ — sensores GPU/Ogre2 e contatos.
- https://gazebosim.org/docs/harmonic/building_robot/ — visual e collision distintos.
- Esquema SDFormat instalado, `gz sdf --check`, exemplos nativos do gz-sim 8.15.

Dimensões AMR: 0.95×0.70 m, altura 0.344 m; diagonal no plano ≈1.180 m.
Adotar folga lateral 0.30 m e entre veículos 0.30 m: 3 AMRs precisam 3.30 m.
Via principal 4 m permite essa largura; secundárias 3 m permitem duas faixas,
giro (diagonal + 0.60 ≈1.78 m) e folga junto aos racks. Não é certificação de segurança.

## Retomada — 2026-10-02

A implementação parcial de `amr_simulation` já existia nesta retomada e foi
continuada. Antes de editar: `git status`, `git diff`, inventário de `src/`,
launches, configuração, scripts e evidências. Nenhum AGENTS.md aplicável.
Alterações encontradas: extensão de sim.launch.py, três artefatos de validação
preexistentes em amr_description e o pacote amr_simulation ainda não rastreado.
Não houve reset, exclusão de trabalho do usuário, commit ou novo pacote duplicado.

Testes repetidos antes das novas alterações: 10 testes do galpão aprovados;
mesma falha flake8 de my_py_pkg (12 ocorrências), pep257 aprovado e copyright
skipped. URDF, smoke/display/RViz, runtime AMR e GUI Ogre aprovados em cópia
isolada. Evidências: resume_baseline_colcon.txt e resume_baseline_amr/.

A execução interrompida do cenário normal estava medindo um pilar enquanto
esperava a parede (0,778 m versus 1,260 m). A medição da parede foi deslocada
para x=2 e o pilar recebeu verificação própria. Os testes de caixas passaram a
apontar para uma caixa, evitando o vão real entre duas caixas. A validação
ampliada encontrou o poste da câmera próximo da antiga aproximação de staging;
o goal foi corrigido para (-10,2; -7,3), sem alterar o AMR.

Confirmado na implementação oficial do sistema Contact do Gazebo Sim 8:
`<sensor><topic>` sozinho não seleciona o tópico de contato; este sistema lê
`<sensor><contact><topic>`. O gerador agora fornece ambos. Antes da correção,
o log mostrava os tópicos scoped default, enquanto o teste escutava os nomes
customizados. Após a correção, contato parede/bumper foi recebido e o robô
parou em y=-9,525 m, com parede em y=-10 m. O teste usa ground truth, pois
odometria de rodas pode acumular deslocamento quando há contato e patinagem.
Fonte: https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/contact/Contact.cc

A biblioteca Python de transporte recebe muitos contatos de piso quando se
assina continuamente human_01/02 (sistema Contact publica no passo físico).
A aceitação assina somente parede, barreira e human_03; os tópicos das pessoas
móveis continuam disponíveis. Isso evita sobrecarga dos callbacks durante
requisições síncronas de reposicionamento do teste.

## Retomada final — 2026-10-03

Antes de editar novamente foram lidos git status/diff, todos os pacotes e
launches relevantes, configuração central, gerador, validadores e evidências.
Nenhum simulador estava em execução. MCP GitHub e busca web disponíveis;
filesystem/shell locais suficientes; nenhum MCP Blender necessário. As páginas
oficiais de actors, TrajectoryFollower Sim 8 e sensores Harmonic foram novamente
consultadas. Não foi introduzida arquitetura ou dependência de assets remotos.

Estado encontrado: quatro cenários headless e GUI já aprovados, 16 testes
pytest no pacote, mas sem repeatability.json nem REPORT.md final. Repetidos antes
das edições: suíte completa (16 pytest do warehouse aprovados; falha flake8
preexistente de my_py_pkg inalterada) e regressão AMR URDF/display/RViz/headless/GUI
aprovada em cópia temporária. Evidências resume_20261003_baseline_*.txt.

O build script passou a registrar build e testes do workspace completo, além
de exigir aprovação do pacote warehouse; a falha alheia ao pacote continua
visível. O relatório agora gera o inventário e estado Git referenciados, e a
posição da câmera foi escrita sem ambiguidade entre vírgula decimal e separador.
A geração geométrica e as dimensões do AMR não precisaram ser alteradas nesta
retomada. A aceitação completa foi iniciada após essas correções.

Resultado final: build dos quatro pacotes aprovado; 16 testes pytest do galpão
aprovados; 10 SDFs válidos e reconstrução determinística. Regressão AMR aprovada.
Cenários normal/person_crossing/obstacle_in_aisle/corridor_blocked aprovados
(78/34/45/40 verificações nesta execução). Repetibilidade: erro máximo 1,123 mm
em 23,273 s simulados. GUI Ogre capturada e inspecionada; sensores Ogre2.
REPORT.md foi gerado somente após aprovação de evidências do mesmo manifesto.
Falha flake8 preexistente permanece explicitamente registrada. Nenhum commit.
