# Relatório de implementação e aceitação

Data: 2026-09-29T23:14:52.206968+00:00

Workspace autorizado: `/home/gengo/ros2_ws`. Pacote novo: `src/amr_description`.
Não existia `amr_description`; `my_cpp_pkg`, `my_py_pkg` e a pasta `warehouse_amr` foram preservados.
Nenhum AGENTS.md aplicável foi encontrado. Nenhuma ação de mouse/teclado foi utilizada.
MCP Blender local ausente; geração executada no Blender 5.2.2 headless com Python 3.13.13.

## Verificações executadas

- Blender: parâmetros, unidades, objetos, origens, escala, manifold, dimensões, folga das rodas e raios LiDAR.
- Exportação: oito STLs individuais, eixos ROS, origem local, dimensões e conteúdo binário válidos.
- Reconstrução: STLs idênticos byte a byte, cena externa preservada, variante dimensional aprovada.
- ROS: XML, Xacro de display e simulação, check_urdf e conversão Harmonic SDF aprovados.
- Árvore: 15 links, 14 joints; todos os TFs publicados.
- Build: três pacotes compilaram. O único warning de build foi o aviso setuptools/pytest-repeat do pacote preexistente my_py_pkg.
- RViz: modelo carregado, status global OK, captura em rviz.png.
- Gazebo: spawn, controladores ativos, contato com chão, estabilidade, sensores, movimento e timeout aprovados.
- GUI Harmonic: imagem em gazebo.png; cliente Ogre, servidor de sensores Ogre2.

## Medições do teste final

- LiDAR central até obstáculo: 2.360036 m.
- Profundidade central RGB-D: 2.323000 m.
- Translação física: 0.575962 m.
- Translação pela odometria: 0.575965 m.
- Giro físico: 1.255426 rad.
- Altura do base_link assentado: -0.000000391 m.
- Massa total estimada: 84.45 kg.

## Correções verificadas durante a implementação

- Preservação de largura máxima no contorno afunilado.
- Recessos de bumper/câmera para eliminar superfícies coplanares.
- Cabeçalho STL estável e índices determinísticos na esfera dos apoios.
- Frame separado para nuvem de pontos X-forward do Harmonic, com frame_id específico na bridge.
- Remapeamentos passados ao spawner pela API atual e limites de comando habilitados.
- Plano de origem das câmeras ortográficas acima do piso dos renders.

## Limitações observadas

Ogre2 sobre Xvfb/llvmpipe falhou dentro de Mesa/EGL. O teste da janela usa o renderer Ogre
fornecido pelo Gazebo Harmonic; os sensores permanecem em Ogre2/EGL. Não é Gazebo Classic.
O launch também permite `gui_renderer:=ogre`. O servidor é separado da janela.

Avisos esperados do plugin/SDFormat: extensão gz_frame_id preservada; atualização de controle
100 Hz versus física 1000 Hz; IMU fora das interfaces ros2_control pois usa bridge; inicialização
transitória do Resource Manager; métricas internas sem executor. A frenagem por timeout é intencional.
As linhas de encerramento por SIGINT nos logs decorrem do cleanup do teste, não de instabilidade.

Massas/inércias estimadas, casters esféricos simplificados, sensores ideais, LiDAR frontal de 180°,
sem carga, bumper lógico, navegação Nav2 ou SLAM configurados. Próximas etapas estão no README.

Os relatórios JSON e as imagens são versionáveis. Logs transitórios e caches são ignorados.
O inventário completo está em FILES.txt.
