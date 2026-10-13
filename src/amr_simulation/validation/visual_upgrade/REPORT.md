# Melhoria visual e física do galpão — ROS 2 Jazzy / Gazebo Harmonic

Data: 2026-10-06; validação final retomada em 2026-10-07. Pacote: `src/amr_simulation`. Gazebo instalado: **8.15.0**.

## Escopo e situação anterior

O gerador JSON já produzia quatro mundos determinísticos, 16 racks, pallets,
placas, estações e três pessoas com colisão cilíndrica. Duas pessoas recebiam
força/torque pelo `TrajectoryFollower`; a terceira era estática. Não havia marcha.
Os corredores tinham 3 m e o corredor transversal principal 4 m. Foram preservados.

A referência foi executada **antes de alterar a simulação**: mundo `normal`, AMR
original presente, sensores ligados, GUI desligada, 40 segundos simulados após
aquecimento. Poses de `/world/warehouse/dynamic_pose/info`, com timestamps de
simulação, foram amostradas aproximadamente a cada 0,5 s. A velocidade é a soma
fundamentada de `hypot(dx,dy) / dt`, nunca um valor inferido de força ou torque.
Dados brutos: [before/measurement.json](before/measurement.json).

Problemas encontrados: força constante não controla velocidade, percursos de
comprimentos distintos atingiam velocidades distintas, faltava frenagem antecipada,
e a orientação era controlada por torque de sinal fixo com zeramento explícito
da velocidade angular. As figuras humanas eram rígidas e os espaços periféricos
pouco equipados. A primeira verificação visual também revelou que atualizar uma
pose no ECM sem marcar `PeriodicChange` anima o servidor, mas não transmite a
animação para esta GUI; isso foi corrigido antes das evidências finais.

## Movimento e configuração

O plugin local `src/HumanSystem.cc` substitui somente o `TrajectoryFollower` das
duas pessoas, dentro do **mesmo servidor Gazebo**. Não há novo simulador, novo nó
ROS, alterações do URDF/Xacro, controle diferencial, bridges do AMR, tópicos,
frames ou TF. O launch acrescenta o caminho da biblioteca instalada.

- Velocidade em m/s: `humans[].speed`, com override por cenário.
- Normal: `human_01 = 0,85 m/s` (inspeção lenta), `human_02 = 1,15 m/s`.
- Travessia: `human_01 = 1,10 m/s` em `person_crossing`.
- `human_03` permanece estática na expedição.
- `human_model.motion`: aceleração 0,8 m/s², frenagem comandada 1,1 m/s²,
  força máxima 140 N, torque máximo 18 N·m e referência de giro até 1,15 rad/s.
  Atrito físico também participa da desaceleração real.
- Servo de velocidade com compensação do atrito configurado; aproximação do
  waypoint reduz a referência pela distância de frenagem. Um novo ponto só é
  selecionado após aproximação e redução da velocidade.
- Referência de translação é reduzida enquanto o corpo muda de orientação.
  Nenhuma pose ou velocidade física é imposta: usa-se `Link::AddWorldWrench`.
- Corpos bloqueados continuam sujeitos a contato; não atravessam o AMR para
  cumprir um cronograma. Rotas continuam sendo verificadas contra geometria fixa.

A leitura do código oficial da versão instalada confirmou que o plugin nativo
**não oferece velocidade/aceleração/frenagem configuráveis**. Os novos campos SDF
pertencem explicitamente ao nosso plugin. A comparação das alternativas e as
fontes oficiais estão em [research/DECISIONS.md](research/DECISIONS.md) e
[research/sources.json](research/sources.json).

## Animação

Foi implementada a separação física/visual dentro do mesmo link: cilindro,
inércia e sensores de contato preservados; braços, pernas e sapatos visuais
alternam em torno de ombros/quadris e o tronco oscila levemente. Os visuais
participam do renderer dos sensores e da GUI.

A fase da marcha depende da **distância física efetivamente percorrida**. A
amplitude acompanha a velocidade medida; em repouso retorna à postura neutra.
As atualizações visuais são limitadas a 30 Hz simulados. Não foram acrescentados
joints físicos, malhas externas, texturas grandes ou luzes.

Actors com skin/skeleton DAE/BVH foram investigados, incluindo `FollowActor`.
Actors isolados não recebem contato. Uma camada de actor sobre um corpo físico
é possível com sincronização própria, mas `FollowActor` segue o alvo a uma
distância com velocidade própria, não mantém a sobreposição exata. A alternativa
implementada evita esse desacoplamento e preserva o corpo físico já validado.
Não se afirma que animação esquelética seja impossível no Harmonic.

## Objetos e distribuição

Seis modelos reutilizáveis foram adicionados, todos gerados de receitas em
`industrial_assets` e posicionados por `industrial_objects` no JSON:

| Modelo | Uso no layout |
|---|---|
| `wh_industrial_cart` | Recebimento, expedição e buffer norte |
| `wh_tool_cart` | Apoio ao staging e manutenção junto ao charging |
| `wh_industrial_bin` | Descarte próximo às paredes do recebimento e charging |
| `wh_cone` | Delimitação dos apoios e aviso no cenário de obstrução parcial |
| `wh_stacked_crates` | Buffer norte e materiais da expedição |
| `wh_parked_forklift` | Empilhadeira simplificada estacionada junto ao recebimento |

São 12 instâncias industriais permanentes, mais dois loaded pallets de buffer
com leve desalinhamento. Os modelos usam caixas e cilindros simples. Racks e
estações existentes foram mantidos; áreas de aproximação e corredores permanecem
livres. O gerador rejeita equipamento permanente sobre corredores, goals, spawns
ou rotas humanas. A planta SVG também mostra o equipamento.

`normal` mantém a operação; `person_crossing` muda a rota da pessoa para atravessar
`y=0` diante do AMR; `obstacle_in_aisle` mantém o pallet parcial e acrescenta um
cone de aviso; `corridor_blocked` mantém a barreira sólida cobrindo os 3 m da passagem.

## Medições e testes

### Velocidade por deslocamento físico

Janela de aproximadamente 40 s simulados, amostras de pose a cada aproximadamente
0,5 s. A média inclui aceleração, frenagem e giros quase parados. O P95 representa
os trechos próximos da velocidade de caminhada; não é a média do percurso.

| Pessoa, cenário normal | Média antes | P95 antes | Máximo antes | Média depois | P95 depois | Máximo depois |
|---|---:|---:|---:|---:|---:|---:|
| human_01 | 0,355 | 0,667 | 0,699 | 0,281 | 0,788 | 0,811 |
| human_02 | 0,688 | 1,308 | 1,398 | 0,753 | 1,150 | 1,150 |
| human_03 (estática) | 0 | 0 | 0 | 0 | 0 | 0 |

Valores em m/s. Distâncias antes: 14,199 m e 27,523 m em 39,995 s;
depois: 11,211 m e 30,068 m em 39,932 s. O trajeto curto de `human_01`
passa uma fração maior do tempo freando e girando; portanto a média diminuiu,
embora os trechos de caminhada tenham ficado na faixa lenta pretendida.
Taxas máximas de giro medidas depois: 0,515 e 0,616 rad/s; acelerações absolutas
estimadas entre amostras: 1,075 e 1,170 m/s², incluindo frenagem e atrito.
A resolução de 0,5 s não permite afirmar limites de pico instantâneo.

Fontes: [before/measurement.json](before/measurement.json) e
[after_final/measurement.json](after_final/measurement.json), incluindo poses,
tempos, velocidades calculadas e amostras de RTF. `human_03` é um modelo
estático SDF; sua velocidade zero é registrada como tal, sem inventar amostras
no tópico de poses dinâmicas.

### Desempenho

RTF médio publicado pelo Gazebo: **0,5777 antes → 0,5304 depois**, redução
relativa de **8,19%** (0,0473 absoluto). Mesma máquina, mundo `normal`, AMR e
sensores ativos, sem GUI. É uma regressão relevante e mensurável, mas não
uma queda de ordem de grandeza. São duas janelas únicas, sem intervalo de
confiança; carga do host e renderização podem afetar a comparação.
O cenário já executava abaixo de tempo real antes da mudança. Não se atribui
esse custo exclusivamente à animação: também foram adicionados modelos.

### Testes

- Build completo: quatro pacotes compilados, conforme [build_final.log](build_final.log).
- `colcon test`: **35 testes, zero erros, zero falhas, um ignorado**;
  [workspace_test_results_final.log](workspace_test_results_final.log).
  A primeira execução encontrou 12 avisos de estilo em três arquivos preexistentes
  de `my_py_pkg`; foram corrigidos somente imports, linhas em branco e newline
  final. O resultado inicial foi preservado em `workspace_test_results.log`.
- Pacote `amr_simulation`: resumo de **32 testes, zero falhas**, incluindo
  geração determinística, composição dos quatro cenários e rejeição de
  velocidades inválidas ou equipamentos sobre áreas protegidas.
- Validação estática: geração determinística e idempotente, SDF validado,
  sem assets de rede; [static.json](static.json).
- Medição final de velocidade: `--verify` aprovado.
- GUI real dos quatro cenários: **aprovada**, registros `*_gui.json`.
  Foram observados galpão, AMR, humanos e novos equipamentos; o cenário normal
  também exercitou translação, curva, giro e parada do robô.
- Contato da pessoa móvel: **aprovado**, 17.028 eventos de contato com o AMR,
  distância mínima entre centros no eixo de aproximação de 0,755 m;
  [contact/human_01_contact.json](contact/human_01_contact.json).
  A perna apresentou variação de orientação ao caminhar e o corpo imobilizado
  teve deslocamento zero, com retorno da perna à pose neutra.
- LiDAR na GUI durante a travessia: retorno central mais próximo de 1,108 m;
  [person_crossing_gui.json](person_crossing_gui.json).

- Headless: **quatro cenários aprovados**, incluindo locomoção/contatos,
  passagem livre no normal, passagem ao lado do pallet e bloqueio pela barreira.
  Resultados completos: [normal](headless/normal/runtime.json),
  [person_crossing](headless/person_crossing/runtime.json),
  [obstacle_in_aisle](headless/obstacle_in_aisle/runtime.json) e
  [corridor_blocked](headless/corridor_blocked/runtime.json).

- Repetibilidade: **aprovada**, comparação em 92 instantes comuns ao longo de
  23,192 s simulados. Diferenças máximas: **3,4 mm** (`human_01`) e
  **5,0 mm** (`human_02`), abaixo da tolerância de 50 mm;
  [headless/repeatability.json](headless/repeatability.json).

- Regressão original de `amr_description`: **aprovada**, URDF/Xacro e
  conversão SDF, sensores, odometria, movimento, RViz e Gazebo GUI real.
  Translação física de 0,56405 m contra 0,56404 m na odometria; giro de
  1,23748 rad. [amr_regression/runtime.json](amr_regression/runtime.json),
  [amr_regression/gazebo_gui.json](amr_regression/gazebo_gui.json) e
  [amr_regression/display.json](amr_regression/display.json).
- `git diff --check`: aprovado. Fontes de `amr_description` sem alterações.

A medição adicional de `person_crossing` também passou com `--verify`:
`human_01` percorreu 17,651 m em 39,799 s, média de **0,443 m/s**, P95 de
**1,038 m/s** e máximo de **1,100 m/s**, incluindo as inversões de percurso.
`human_02`: média de 0,753 m/s e P95/máximo de 1,150 m/s.
[Dados completos](crossing_measurement/measurement.json).
O RTF desta execução adicional foi **0,418**; ela ocorreu em outro momento e
outro cenário, portanto não substitui a comparação antes/depois do `normal`.

Todos os quatro cenários foram abertos efetivamente no Gazebo GUI, e todos os
quatro também executaram os testes físicos headless. Foram preservadas as
capturas e os dados de cada execução. A versão da biblioteca e do manifesto
atual coincide com a das evidências; hashes em
[evidence_manifest.json](evidence_manifest.json).

## Evidências visuais

Na sequência `human_motion_t0/t1/t2`, os registros mostram `human_01` em
`y=-1,534 m`, `y=-0,088 m` e `y=+1,587 m`, respectivamente. O intervalo t0→t1
inclui a chegada ao extremo e a inversão do percurso; não deve ser usado como
estimativa de velocidade de cruzeiro. A câmera permanece igual nas três imagens.

As imagens finais são capturas da janela do **Gazebo GUI real** em display Xvfb
isolado, não renders de SDF nem imagens geradas. GUI: Ogre; sensores: Ogre2/EGL.
A câmera foi movida pelo serviço de câmera da GUI. As pessoas não foram
reposicionadas para as fotografias. O AMR só foi reposicionado nos testes
funcionais de sensores e contato explicitamente registrados nos validadores.

- [normal.png](normal.png): vista geral, AMR, pessoas e equipamentos.
- [person_crossing.png](person_crossing.png): recebimento, AMR e travessia.
- [obstacle_in_aisle.png](obstacle_in_aisle.png): pallet parcial e cone.
- [corridor_blocked.png](corridor_blocked.png): barreira entre racks.
- [human_motion_t0.png](human_motion_t0.png), [human_motion_t1.png](human_motion_t1.png),
  [human_motion_t2.png](human_motion_t2.png): três instantes da mesma execução,
  com a mesma câmera e posições distintas da pessoa.
- `*_overview.png`, `*_amr.png`, `*_closeup.png`: vistas complementares.
- `*_gui.json`: horários simulados, poses, sensores e identificação das capturas.
- `*_lifecycle.json`: processos antes/depois, confirmando encerramento dos launches.

## Reprodução manual

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

# Executar um cenário de cada vez; encerrar com Ctrl+C antes do próximo.
ros2 launch amr_simulation warehouse.launch.py scenario:=normal
ros2 launch amr_simulation warehouse.launch.py scenario:=person_crossing
ros2 launch amr_simulation warehouse.launch.py scenario:=obstacle_in_aisle
ros2 launch amr_simulation warehouse.launch.py scenario:=corridor_blocked
```

Para medir especificamente a travessia:

```bash
python3 src/amr_simulation/scripts/measure_humans.py \
  --scenario person_crossing --seconds 40 --verify \
  --output /tmp/warehouse_validation/crossing_measurement
```

A GUI do pacote usa Ogre por padrão; o AMR continua usando seus sensores Ogre2.
`gui:=false` executa cada cenário headless. `rviz:=true` habilita a interface ROS
existente. O build recompila o plugin e regenera deterministicamente os mundos.

```bash
# Depois do build e do source; sem outro Gazebo aberto.
python3 src/amr_simulation/scripts/validate_warehouse.py \
  --output /tmp/warehouse_validation/static.json
colcon test --event-handlers console_direct+
colcon test-result --verbose
colcon test-result --test-result-base build/amr_simulation --verbose
python3 src/amr_simulation/scripts/measure_humans.py \
  --output /tmp/warehouse_validation/measurement --seconds 40 --verify
python3 src/amr_simulation/scripts/run_acceptance.py \
  --output-root /tmp/warehouse_validation/headless
python3 src/amr_simulation/scripts/validate_repeatability.py \
  --output-root /tmp/warehouse_validation/headless
python3 src/amr_simulation/scripts/validate_crossing_contact.py \
  --output /tmp/warehouse_validation/contact
python3 src/amr_simulation/scripts/validate_amr_regression.py \
  --output /tmp/warehouse_validation/amr_regression
python3 src/amr_simulation/scripts/validate_scenario_gui.py \
  --output /tmp/warehouse_validation/gui
```

## Arquivos modificados

- `CMakeLists.txt`, `package.xml`, `launch/warehouse.launch.py`: compilação,
  dependência Harmonic e descoberta da biblioteca do plugin.
- `src/HumanSystem.cc`: movimento físico e marcha visual.
- `config/warehouse_layout.json`, `config/scenarios/person_crossing.json`,
  `config/scenarios/obstacle_in_aisle.json`: parâmetros, equipamentos e cenários.
- `scripts/generate_warehouse.py`, `scripts/render_layout.py`: geração,
  validação geométrica e planta.
- `models/wh_{industrial_cart,tool_cart,industrial_bin,cone,stacked_crates,parked_forklift}/`:
  novos modelos gerados; `models/wh_building/model.sdf`: inclusões geradas.
- `worlds/warehouse_*.sdf`, `config/generated_manifest.json`,
  `config/warehouse_layout.svg`: artefatos regenerados.
- `scripts/measure_humans.py`: medição reproduzível de velocidade e RTF.
- Validadores `validate_runtime.py`, `validate_crossing_contact.py`,
  `validate_scenario_gui.py`, `validate_amr_regression.py`,
  `validate_repeatability.py`, `validate_warehouse.py`: novas verificações e
  saída opcional para preservar evidências anteriores.
- `tests/test_generation.py`, `README.md` e esta pasta de evidências.
- Fora do pacote: `src/my_py_pkg/my_py_pkg/{my_first_node,robot_news_station,smartphone}.py`:
  somente correções de estilo necessárias para a suíte completa do workspace.

## Limitações

Uma execução headless de `corridor_blocked` concluiu todas as 43 verificações,
mas o processo Gazebo não respondeu ao encerramento normal e o launch escalou
para SIGKILL. O log preserva esse problema; o verificador confirmou que nenhum
processo da simulação permaneceu ativo. As outras três execuções headless
encerraram sem erros. O resultado funcional aprovado não significa encerramento
limpo em todas as execuções; esse comportamento merece investigação separada.

A marcha é procedural simples, sem joelhos, foot IK ou passos específicos de giro.
O cilindro é um envelope conservador contínuo, enquanto o LiDAR GPU vê os membros
visuais: a distância visual pode diferir alguns centímetros da superfície física.
Não há navegação social ou desvio autônomo de pessoas; diante de um bloqueio o
corpo pode permanecer parado ou empurrar fisicamente um objeto móvel até o limite
de força. `human_03` continua estática. Os equipamentos industriais são estáticos.
Pallets vazios muito baixos continuam abaixo do plano do LiDAR 2D; isso não elimina
suas colisões nem a detecção pela câmera de profundidade. Não foram introduzidos
Nav2, SLAM, gerenciamento de frota nem alterações nas interfaces existentes.
