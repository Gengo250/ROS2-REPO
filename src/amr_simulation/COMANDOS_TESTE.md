# Comandos para testar o galpão e o AMR

Execute em Bash, no workspace `~/ros2_ws`, com ROS 2 Jazzy e Gazebo Harmonic.
Para apenas ver o resultado, siga as seções 1 e 2. Para repetir todas as
validações e gerar novas evidências, siga a seção 6.

## 1. Compilar e preparar os terminais

Execute uma vez após baixar ou modificar o projeto:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

O build compila o plugin humano e regenera os mundos a partir do JSON.
Só prossiga se terminar sem erros. Não edite manualmente os SDF gerados.

Em **cada terminal usado para a inspeção manual**, execute:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=warehouse_manual
```

Esses valores devem ser iguais no terminal do launch e nos terminais de
comandos/sensores. A partição explícita permite consultar o Gazebo em outro
terminal. Use apenas uma simulação por vez.

## 2. Abrir e observar os quatro cenários

Execute **um comando por vez** no terminal 1. Aguarde o AMR e os controladores
carregarem. Deixe a simulação rodar para observar as pessoas; se estiver
pausada, use Play na GUI. Encerre com `Ctrl+C` e espere o launch terminar
antes de iniciar o próximo cenário.

### Normal

```bash
ros2 launch amr_simulation warehouse.launch.py scenario:=normal
```

Observe as duas pessoas caminhando, a terceira parada, os equipamentos
industriais e os corredores principais livres. Aproxime a câmera das pessoas
para ver braços e pernas alternando, desaceleração e giro nos extremos da rota.

### Pessoa cruzando a rota

```bash
ros2 launch amr_simulation warehouse.launch.py scenario:=person_crossing
```

Observe a pessoa na área de recebimento, à frente do spawn do AMR. Ela cruza
repetidamente o corredor principal, entre `y=-2,5 m` e `y=+2,5 m`, perto de
`x=-8,7 m`. Aguarde uma ida e volta para observar frenagem e mudança de direção.

### Obstáculo parcial no corredor

```bash
ros2 launch amr_simulation warehouse.launch.py scenario:=obstacle_in_aisle
```

Observe o pallet carregado e o cone entre os racks, perto de
`x=-5,35 m, y=-4,5 m`. Existe passagem lateral para o AMR.

### Corredor totalmente bloqueado

```bash
ros2 launch amr_simulation warehouse.launch.py scenario:=corridor_blocked
```

Observe a barreira amarela/preta cobrindo a passagem entre os racks, perto de
`x=-4,5 m, y=-4,5 m`. O teste automatizado confirma o bloqueio por contato físico.

### Abrir também o RViz

Como alternativa aos comandos acima:

```bash
ros2 launch amr_simulation warehouse.launch.py scenario:=normal rviz:=true
```

O argumento `rviz:=true` pode ser usado com qualquer um dos quatro cenários.
No RViz, habilite os displays desejados de Scan/TF e use `odom` como Fixed Frame
para acompanhar o robô. Não há navegação automática/Nav2 neste pacote: abrir
o cenário não comanda uma missão do AMR.

### Executar sem interface gráfica

Como alternativa ao launch com GUI:

```bash
ros2 launch amr_simulation warehouse.launch.py scenario:=normal gui:=false
```

Troque `normal` pelo cenário desejado. Os sensores continuam ativos.

## 3. Movimentar o AMR manualmente

Mantenha o cenário aberto no terminal 1. No terminal 2, prepare o ambiente
conforme a seção 1 e execute **um comando de movimento por vez**.
O tópico recebe `TwistStamped`; `header: auto` atualiza o carimbo de tempo.

Frente a 0,2 m/s:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: auto, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}'
```

Giro no próprio eixo:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: auto, twist: {linear: {x: 0.0}, angular: {z: 0.4}}}'
```

Curva:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: auto, twist: {linear: {x: 0.2}, angular: {z: 0.2}}}'
```

Interrompa a publicação com `Ctrl+C`. O timeout do controlador freia o robô
após aproximadamente 0,5 s simulados. Para parada explícita, depois de
interromper o publicador de movimento:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: auto, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}'
```

## 4. Conferir sensores, TF e desempenho ao vivo

Use outro terminal com o mesmo ambiente da seção 1. Os comandos contínuos
abaixo terminam com `Ctrl+C`; execute-os individualmente.

Listar tópicos e controladores:

```bash
ros2 topic list
ros2 control list_controllers
```

São esperados `joint_state_broadcaster` e `diff_drive_controller` ativos.

Ler uma amostra de LiDAR, IMU e odometria:

```bash
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 topic echo /imu/data --once --qos-reliability best_effort
ros2 topic echo /odom --once --qos-reliability best_effort
```

Medir frequência de recepção:

```bash
ros2 topic hz /scan
ros2 topic hz /imu/data
ros2 topic hz /camera/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/points
ros2 topic hz /warehouse/camera_01/image_raw
```

As taxas observadas em tempo real podem ficar abaixo das taxas configuradas
quando o RTF é menor que 1. O relatório automatizado também registra taxas
por tempo de simulação.

### Abrir as câmeras

Mantenha um cenário da seção 2 em execução e com Play ativo. Em cada novo
terminal, execute primeiro a preparação da seção 1, usando o mesmo
`ROS_DOMAIN_ID` do launch. Os visualizadores precisam de uma sessão gráfica
e também funcionam quando o cenário é iniciado com `gui:=false`.

A configuração atual possui uma câmera fixa no galpão, além da câmera RGB-D
do AMR. Abra cada imagem em um terminal separado:

Câmera fixa do galpão:

```bash
ros2 run rqt_image_view rqt_image_view /warehouse/camera_01/image_raw
```

Câmera RGB do AMR:

```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

Imagem de profundidade do AMR:

```bash
ros2 run rqt_image_view rqt_image_view /camera/depth/image_raw
```

O seletor de tópicos do visualizador permite alternar entre as imagens.
Feche a janela ou use `Ctrl+C` no respectivo terminal para encerrar.
Se aparecer `Package 'rqt_image_view' not found`, instale o visualizador e
repita o comando:

```bash
sudo apt install ros-jazzy-rqt-image-view
```

### Conferir TF e estatísticas do Gazebo

Conferir transformações:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_frame
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
ros2 run tf2_ros tf2_echo world warehouse_camera_01_optical_frame
```

Ver RTF e tempo simulado pelo Gazebo:

```bash
gz topic -e -t /world/warehouse/stats
```

Procure `real_time_factor`, `sim_time` e `real_time`. Para consultar posições
físicas, incluindo `human_01`, `human_02` e `warehouse_amr`:

```bash
gz topic -e -t /world/warehouse/dynamic_pose/info
```

Para uma medição calculada de velocidade humana, use a seção 6; valores de
força e torque não representam velocidade.

## 5. Ver as evidências já produzidas

O relatório existente contém decisões técnicas, velocidades antes/depois,
limitações e a lista de arquivos alterados:

- [Relatório completo](validation/visual_upgrade/REPORT.md)
- [Resumo dos testes](validation/visual_upgrade/final_summary.json)
- [Normal](validation/visual_upgrade/normal.png)
- [Travessia](validation/visual_upgrade/person_crossing.png)
- [Obstáculo parcial](validation/visual_upgrade/obstacle_in_aisle.png)
- [Bloqueio total](validation/visual_upgrade/corridor_blocked.png)
- Movimento humano: [t0](validation/visual_upgrade/human_motion_t0.png),
  [t1](validation/visual_upgrade/human_motion_t1.png),
  [t2](validation/visual_upgrade/human_motion_t2.png)

Abrir pelo gerenciador de arquivos do desktop:

```bash
cd ~/ros2_ws
xdg-open src/amr_simulation/validation/visual_upgrade
```

Ou ler no terminal:

```bash
less src/amr_simulation/validation/visual_upgrade/REPORT.md
python3 -m json.tool src/amr_simulation/validation/visual_upgrade/final_summary.json
```

Use `q` para sair do `less`. `xdg-open` requer uma sessão gráfica.

## 6. Reexecutar todas as validações e gerar novas evidências

Encerre primeiro os launches manuais e os publicadores de movimento. Os
validadores abrem e fecham suas próprias simulações; não devem rodar em paralelo.
A validação GUI usa janelas reais em um display Xvfb isolado: elas não aparecem
necessariamente no desktop, mas suas screenshots são gravadas em disco.
Para observar interativamente, use a seção 2.

No terminal de validação, crie uma pasta nova. Execute os demais blocos desta
seção **no mesmo terminal**, para manter a variável `WAREHOUSE_RESULTS`.

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
unset GZ_PARTITION
export WAREHOUSE_RESULTS="$PWD/src/amr_simulation/validation/manual_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$WAREHOUSE_RESULTS"
printf '%s\n' "$WAREHOUSE_RESULTS"
```

O bloco abaixo executa tudo em sequência e para no primeiro erro. Pode levar
vários minutos. Os logs aparecem no terminal e também ficam na pasta nova.
As evidências originais em `visual_upgrade/` são preservadas.

```bash
(
  set -eo pipefail

  colcon build 2>&1 | tee "$WAREHOUSE_RESULTS/build.log"
  source install/setup.bash

  colcon test --event-handlers console_direct+ \
    2>&1 | tee "$WAREHOUSE_RESULTS/workspace_tests.log"
  colcon test-result --verbose \
    2>&1 | tee "$WAREHOUSE_RESULTS/workspace_test_results.log"

  python3 src/amr_simulation/scripts/validate_warehouse.py \
    --output "$WAREHOUSE_RESULTS/static.json" \
    2>&1 | tee "$WAREHOUSE_RESULTS/static.log"

  python3 src/amr_simulation/scripts/measure_humans.py \
    --scenario normal --seconds 40 --verify \
    --output "$WAREHOUSE_RESULTS/normal_measurement" \
    2>&1 | tee "$WAREHOUSE_RESULTS/normal_measurement.log"

  python3 src/amr_simulation/scripts/measure_humans.py \
    --scenario person_crossing --seconds 40 --verify \
    --output "$WAREHOUSE_RESULTS/crossing_measurement" \
    2>&1 | tee "$WAREHOUSE_RESULTS/crossing_measurement.log"

  python3 src/amr_simulation/scripts/run_acceptance.py \
    --output-root "$WAREHOUSE_RESULTS/headless" \
    2>&1 | tee "$WAREHOUSE_RESULTS/headless.log"

  python3 src/amr_simulation/scripts/validate_repeatability.py \
    --output-root "$WAREHOUSE_RESULTS/headless" \
    2>&1 | tee "$WAREHOUSE_RESULTS/repeatability.log"

  python3 src/amr_simulation/scripts/validate_crossing_contact.py \
    --output "$WAREHOUSE_RESULTS/contact" \
    2>&1 | tee "$WAREHOUSE_RESULTS/contact.log"

  python3 src/amr_simulation/scripts/validate_amr_regression.py \
    --output "$WAREHOUSE_RESULTS/amr_regression" \
    2>&1 | tee "$WAREHOUSE_RESULTS/amr_regression.log"

  python3 src/amr_simulation/scripts/validate_scenario_gui.py \
    --output "$WAREHOUSE_RESULTS/gui" \
    2>&1 | tee "$WAREHOUSE_RESULTS/gui.log"

  printf '\nVALIDAÇÕES CONCLUÍDAS. Resultados: %s\n' "$WAREHOUSE_RESULTS"
)
```

### O que cada etapa verifica

| Etapa | Resultado esperado / saída |
|---|---|
| Build | Quatro pacotes compilados sem erro |
| Testes do workspace | Zero falhas; a rodada registrada teve 35 testes e um ignorado |
| Validação estática | `static.json`: `status: passed`, determinismo e SDF |
| Medições humanas | `*/measurement.json`: `verification: passed`, velocidades e RTF |
| Headless | Quatro `headless/<cenário>/runtime.json` com `status: passed` |
| Repetibilidade | `headless/repeatability.json`: `status: passed` |
| Contato humano | `contact/human_01_contact.json`: contato real e marcha parada em repouso |
| Regressão AMR | `amr_regression/`: URDF, sensores, movimento, RViz e Gazebo |
| GUI | `gui/`: quatro screenshots principais, sequência humana e `*_gui.json` |

O teste de repetibilidade depende de `headless/person_crossing/runtime.json`
produzido pelo teste headless anterior. Os testes de contato/reposicionamento
movem o AMR deliberadamente para medir colisões e sensores.

### Repetir somente uma etapa

Com a pasta e o ambiente acima preparados, pode copiar apenas o comando
correspondente do bloco completo. Para selecionar um único cenário:

```bash
python3 src/amr_simulation/scripts/run_acceptance.py \
  --scenario obstacle_in_aisle --output-root "$WAREHOUSE_RESULTS/headless"

python3 src/amr_simulation/scripts/validate_scenario_gui.py \
  --scenario person_crossing --output "$WAREHOUSE_RESULTS/gui"
```

Para rodar apenas os testes do pacote:

```bash
colcon test --packages-select amr_simulation --event-handlers console_direct+
colcon test-result --test-result-base build/amr_simulation --verbose
```

## 7. Consultar os novos resultados

Abrir a pasta das novas screenshots:

```bash
xdg-open "$WAREHOUSE_RESULTS/gui"
```

Ela deve conter `normal.png`, `person_crossing.png`, `obstacle_in_aisle.png`,
`corridor_blocked.png` e `human_motion_t0.png`, `human_motion_t1.png`,
`human_motion_t2.png`, além de vistas complementares.

Ler resultados individuais:

```bash
cat "$WAREHOUSE_RESULTS/workspace_test_results.log"
python3 -m json.tool "$WAREHOUSE_RESULTS/headless/normal/runtime.json"
python3 -m json.tool "$WAREHOUSE_RESULTS/headless/repeatability.json"
python3 -m json.tool "$WAREHOUSE_RESULTS/contact/human_01_contact.json"
```

Mostrar somente as velocidades e o RTF, sem imprimir todas as poses:

```bash
python3 - <<'PY'
import json
import os
from pathlib import Path

root = Path(os.environ['WAREHOUSE_RESULTS'])
for folder in ('normal_measurement', 'crossing_measurement'):
    data = json.loads((root / folder / 'measurement.json').read_text())
    print(f"\n{data['scenario']}: RTF={data['rtf_mean']:.3f}")
    for name, values in data['humans'].items():
        if values.get('static'):
            print(f'{name}: estática')
        else:
            print(f"{name}: média={values['mean_m_s']:.3f} m/s, "
                  f"P95={values['p95_m_s']:.3f} m/s, "
                  f"máximo={values['max_m_s']:.3f} m/s")
PY
```

Como referência, a medição registrada do cenário normal apresentou P95 de
0,788 m/s e 1,150 m/s para as duas pessoas. Na travessia, o máximo de
`human_01` foi 1,100 m/s. As médias incluem parada/frenagem/giro, portanto são
menores. O RTF depende da máquina, carga do host e renderização: a comparação
registrada foi 0,578 antes e 0,530 depois, no cenário normal sem GUI.

Se abrir outro terminal, recupere a pasta escolhida com seu caminho real:

```bash
cd ~/ros2_ws
ls -dt src/amr_simulation/validation/manual_*/
```

Defina `WAREHOUSE_RESULTS` para a pasta desejada antes de reutilizar os comandos
de consulta. Cada nova pasta tem um timestamp; repetir uma etapa na mesma pasta
substitui as evidências daquela etapa.

## 8. Encerramento e problemas conhecidos

- Use `Ctrl+C` no terminal do launch e espere seu encerramento antes de trocar
  de cenário ou iniciar testes automáticos.
- Se um validador informar `Existing simulation`, encerre o launch anterior.
  Para identificar processos remanescentes, use:

```bash
ps -eo pid,ppid,args | rg 'gz sim|ros_gz_bridge/parameter_bridge|ros2 launch'
```

- Uma execução registrada de `corridor_blocked` passou nos testes físicos,
  mas precisou de SIGKILL pelo próprio launch no encerramento. Consulte o
  `gazebo.log` daquela execução; não use `pkill` global para encerrar processos.
- Se surgir aviso de hashes/arquivos gerados desatualizados, execute novamente
  `colcon build` e `source install/setup.bash` antes de abrir o cenário.
- Se tópicos ou serviços não aparecerem durante a inspeção manual, confirme
  os mesmos `ROS_DOMAIN_ID` e `GZ_PARTITION` nos terminais e que a simulação
  está executando. Aguarde a descoberta dos participantes.
- As pessoas usam animação simples de braços/pernas. Não possuem navegação
  social para contornar obstáculos; contatos continuam sendo físicos.
