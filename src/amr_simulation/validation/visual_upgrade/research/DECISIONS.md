# Inspeção do Harmonic instalado

Versão confirmada por `gz sim --versions`: **8.15.0**, distribuída no vendor ROS Jazzy.
URLs e SHA256 do código consultado estão em `sources.json`.

## TrajectoryFollower

A implementação `TrajectoryFollowerPrivate::Load` aceita `link_name`, `waypoints/waypoint`,
`circle/radius`, `line/direction`, `line/length`, `loop`, `force`, `torque`,
`range_tolerance`, `bearing_tolerance` (graus), `zero_vel_on_bearing_reached` e `topic`.
Não há parâmetro de velocidade, aceleração, frenagem ou PID. `PreUpdate` aplica força
constante no X local apenas quando o rumo está dentro da tolerância; caso contrário,
aplica torque de sinal fixo. Não freia na chegada ao waypoint. A opção de zerar
velocidade angular chama `Link::SetAngularVelocity` e mantém um componente de comando.

Calibrar somente a força produz velocidade dependente do comprimento do percurso,
contato e atrito. Acrescentar amortecimento passivo limitaria a velocidade terminal,
mas não resolveria frenagem antecipada e controle de rumo. Portanto substituímos apenas
o sistema de movimento das duas pessoas por `amr_simulation::HumanSystem`, no mesmo
processo Gazebo. Seus parâmetros SDF são definidos pelo nosso código, não atribuídos
indevidamente ao TrajectoryFollower. Não há novo nó ROS nem alterações do robô.

## Alternativas visuais investigadas

- [Actor, skin e skeleton no Harmonic](https://gazebosim.org/docs/harmonic/actors/):
  skin/animações COLLADA/DAE e BVH; combinação com trajetória por tempo. Não recebe
  forças de gravidade/contato. Visível a câmeras e sensores GPU, mas não substitui
  um modelo físico nesta tarefa.
- Duas entidades (corpo físico + actor): possível com código adicional que atualize
  `TrajectoryPose`, `AnimationName` e `AnimationTime`. Exige calibrar origem, orientação,
  skin e avanço do skeleton; scripts temporizados independentes deslizariam quando
  o corpo fosse bloqueado. Não foi declarada uma impossibilidade do Gazebo.
- O `FollowActor` realmente instalado foi inspecionado: segue um alvo com velocidade
  própria e limites de distância, em vez de copiar rigidamente a pose física. Não é
  uma solução pronta para sobrepor exatamente visual e colisão durante contatos.
- Links articulados e joints físicos: suportados, mas acrescentariam massas, limites,
  controladores e contatos dos pés, alterando a dinâmica estável já validada.
- Alternativa implementada: **duas camadas dentro do mesmo link**. A colisão/inércia
  de `body` permanece física; poses locais dos visuais fazem pernas/braços alternarem
  em torno de quadris/ombros e oscilam o tronco. Não são joints dinâmicos.
  `RenderUtil::UpdateFromECM` lê `components::Visual + Pose`; `SceneBroadcaster`
  transmite o estado alterado para a GUI quando a pose é marcada com
  `SetChanged(..., PeriodicChange)`. `SetComponentData` sozinho não marca essa
  alteração: a primeira inspeção da GUI revelou e corrigiu esse detalhe. A sincronização é interna ao ECM, sem atraso
  de bridge ROS ou segundo modelo perseguindo o corpo.

A fase cresce com distância física percorrida, não com velocidade comandada nem
relógio de animação independente. A amplitude diminui com a velocidade real; abaixo
do limiar volta à postura neutra. Atualização visual: 30 Hz simulados. Controle físico:
cada passo da simulação, aplicando somente forças/torques limitados no centro de massa.

Esta é uma marcha simplificada: pernas rígidas sem joelhos, sem foot IK, sem passos
específicos para giros estacionários. Prioriza contatos, sensores e estabilidade.
