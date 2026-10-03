# Galpão industrial procedural

ROS 2 Jazzy + Gazebo Harmonic (Sim 8), consumindo o AMR de `amr_description`.
Não configura SLAM, localização, Nav2 ou gerenciamento de frota.

A fonte da verdade é `config/warehouse_layout.json`, combinada com
`config/scenarios/*.json` e as dimensões existentes de `amr_description`.
O gerador produz modelos SDF locais, quatro composições leves de mundo, bridge,
GUI, planta SVG, manifesto de hashes e `config/station_poses.json`. Não precisa de internet,
Fuel, Blender ou edição manual. As texturas das placas são geradas localmente
com Pillow e DejaVu Sans; colisões usam apenas boxes/cilindros.

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
bash src/amr_simulation/scripts/build_warehouse.sh
source install/setup.bash
```

O comando gera, valida SDF/determinismo, faz `colcon build`, executa os testes do
workspace (registrando separadamente falhas preexistentes dos exemplos), exige
aprovação dos testes do galpão, repete os testes funcionais originais do AMR em
cópia temporária e valida os quatro cenários, repetibilidade das pessoas e a GUI.
Evidências ficam em `validation/`.
As capturas dos quatro launches reais ficam em `validation/scenario_gui/`.
O diagnóstico desta rodada está em
[SCENARIO_DEBUG_REPORT.md](validation/SCENARIO_DEBUG_REPORT.md).
Para regeneração e verificações estáticas sem iniciar simuladores:

```bash
bash src/amr_simulation/scripts/build_warehouse.sh --skip-runtime
```

Para abrir, escolher cenário ou executar sem interface:

```bash
ros2 launch amr_simulation warehouse.launch.py
ros2 launch amr_simulation warehouse.launch.py scenario:=person_crossing
ros2 launch amr_simulation warehouse.launch.py scenario:=obstacle_in_aisle
ros2 launch amr_simulation warehouse.launch.py scenario:=corridor_blocked
ros2 launch amr_simulation warehouse.launch.py gui:=false scenario:=normal
```

A GUI usa `gui_renderer:=ogre` por padrão neste pacote; servidor e sensores
continuam usando Ogre2 com `--headless-rendering`. Os defaults dos launches
originais `display.launch.py` e `sim.launch.py` foram preservados.
O título da vista identifica o cenário. A GUI só abre após a criação do AMR
e a ativação dos controladores. Aguarde também o carregamento das malhas.
Fechar a GUI ou usar Ctrl+C no terminal encerra o servidor supervisionado.
Antes de trocar de cenário, espere o launch terminar e confirme com
`ps -eo pid,ppid,args` que não restaram processos `gz sim` ou `parameter_bridge`
da execução anterior. Não use `pkill` global.

Cada launch recebe uma `GZ_PARTITION` nova, impressa no log com o world, hash
e spawn selecionados. Para introspecção Gazebo em outro terminal, exporte
o valor impresso antes de usar `gz service -l` ou `gz topic -l`.
Uma `GZ_PARTITION` explicitamente fornecida no ambiente é respeitada.
Os tópicos ROS não mudam. `rviz:=true` é opcional. Execute apenas um AMR por domínio ROS/partição Gazebo;
para instâncias independentes, configure `ROS_DOMAIN_ID` e `GZ_PARTITION` distintos.

Comandos manuais (`TwistStamped`, carimbo atualizado a cada publicação):

```bash
# Frente; Ctrl+C interrompe a publicação e o timeout de 0,5 s freia o robô.
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: auto, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}'
# Giro
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: auto, twist: {angular: {z: 0.4}}}'
# Curva
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: auto, twist: {linear: {x: 0.2}, angular: {z: 0.2}}}'
# Parada explícita
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: auto, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}'
```

O controlador recebe `TwistStamped` diretamente via ros2_control; não existe
bridge de `/cmd_vel` nem conversão para `Twist`.

Tópicos ROS esperados: `/clock`, `/cmd_vel`, `/odom`, `/joint_states`, `/tf`,
`/tf_static`, `/scan`, `/imu/data`, `/camera/image_raw`, `/camera/camera_info`,
`/camera/depth/image_raw`, `/camera/points`,
`/warehouse/camera_01/image_raw` e `/warehouse/camera_01/camera_info`.
A câmera fixa tem 640×480, 10 Hz, FOV horizontal de 1,4 rad e TF estático
`world → warehouse_camera_01_link → warehouse_camera_01_optical_frame`.

O interior mede 30×20 m, altura de 5 m; origem no centro do piso, X leste,
Y norte, Z para cima. Há 16 módulos de rack em quatro fileiras, recebimento a
 oeste e expedição a leste. O corredor transversal principal tem 4 m; sete
corredores secundários/perimetrais têm 3 m nominais. A configuração valida o
envelope de giro de 1,78 m com folga e 3,30 m para três AMRs lado a lado.
As secundárias comportam dois lado a lado. Três poses de spawn estão registradas;
apenas `amr_01_spawn` inicia por padrão.

A planta gerada está em [config/warehouse_layout.svg](config/warehouse_layout.svg).
O launch verifica hashes da configuração, dos cenários, dos modelos e das
dimensões do AMR; se estiverem desatualizados, solicita regeneração.
O target CMake também regenera os artefatos em todo `colcon build`, antes
da instalação, tanto com cópias quanto com `--symlink-install`.

As poses de `pickup_station_01/02`, `dropoff_station_01/02`, `charging_station`
e `staging_area` estão em `config/station_poses.json`, no frame **world**.
O staging tem pose de aproximação distinta do centro ocupado por cargas.
Na etapa de SLAM será necessário registrar o frame do mapa em relação ao world;
este pacote não publica uma transformação artificial `world → map/odom`.
Zonas restritas e bloqueios temporários são metadados, sem implementação Nav2.

As três pessoas são manequins físicos de primitivas SDF, com pernas visíveis
no plano do LiDAR (z≈0,213 m) e colisão cilíndrica conservadora contínua.
`human_01` e `human_02` usam o sistema nativo `TrajectoryFollower` por forças e
 torques; `human_03` é estática junto à expedição. Rotas, força, torque, atrito,
inércia e tolerâncias são configuráveis. O movimento responde a contatos e não
segue um cronograma rígido quando bloqueado; não há animação de marcha ou
comportamento social. Reiniciar o cenário restaura suas condições iniciais.
Actors não são usados porque não recebem forças de contato no Harmonic.

`normal` mantém os corredores operacionais; `person_crossing` muda a rota de
`human_01` para cruzar o eixo principal; `obstacle_in_aisle` coloca carga parcial
no `aisle_01`; `corridor_blocked` fecha seus 3 m com barreira sólida. Reinicie
com outro argumento para ativar/desativar bloqueios. São inclusões do mesmo
modelo estático, sem duplicação da estrutura do galpão.

O pallet vazio tem 0,15 m, abaixo do feixe LiDAR: suas caixas/cargas e pallets
 elevados nos racks são detectáveis; RGB-D também vê pallets baixos. Isso é uma
limitação geométrica real de um scanner 2D baixo, não ausência de colisão.
O telhado é aberto para inspeção; paredes completas, pilares e vigas superiores
permanecem presentes. Consulte [validation/REPORT.md](validation/REPORT.md)
para medições, imagens, limitações e a falha preexistente de estilo em `my_py_pkg`.


Testes individuais (depois de `source install/setup.bash`):

```bash
python3 src/amr_simulation/scripts/validate_warehouse.py
colcon test --packages-select amr_simulation --event-handlers console_direct+
colcon test-result --test-result-base build/amr_simulation --verbose
python3 src/amr_simulation/scripts/validate_amr_regression.py
python3 src/amr_simulation/scripts/run_acceptance.py
python3 src/amr_simulation/scripts/validate_gui.py
# Suíte completa do workspace, incluindo exemplos preexistentes:
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

A suíte de `my_py_pkg` já apresentava uma falha flake8 antes deste trabalho;
ela é reportada separadamente, sem alterar os exemplos. O teste de GUI abre o
Gazebo real em um display Xvfb isolado, captura a janela e encerra apenas seus
próprios processos, sem automação de mouse/teclado.

Os testes de contato usam mensagens nativas Gazebo `/warehouse/contacts/wall`,
`/warehouse/contacts/human_03` e `/warehouse/contacts/barrier`. No Sim 8, o tópico
é configurado em `sensor/contact/topic`. Não são bridges ROS nem lógica de
missão. Os reposicionamentos usados para isolar medições pertencem apenas aos
testes; as pessoas se movem fisicamente pelo `TrajectoryFollower` nativo.
As medições de deslocamento usam também ground truth, pois odometria de rodas
não comprova deslocamento durante patinagem contra um obstáculo.

Todos os modelos e texturas foram criados proceduralmente neste pacote.
Não há assets Fuel, AWS ou de terceiros copiados. Pillow e a fonte DejaVu Sans
instalada pelo sistema geram as placas; o arquivo da fonte não é redistribuído.
