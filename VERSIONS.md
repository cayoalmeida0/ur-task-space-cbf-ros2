# Registro inicial de versoes

Data da definicao do ambiente: 17 de julho de 2026.

## Infraestrutura 0.2.0 / bringup 0.2.0 — 20 de agosto de 2026

- Adiciona as descricoes OnRobot RG2 e RG6, selecionaveis por
  `ONROBOT_TYPE`, fixadas no commit
  `29180b3fa9cba6555f3e515e789b8ccd34252fab`.
- Acopla a gripper ao `tool0` do Universal Robots e publica o frame
  `gripper_tcp` fornecido pela descricao upstream.
- Adapta a simulacao ao Gazebo Harmonic com
  `gz_ros2_control/GazeboSimSystem`, sem carregar o plugin Gazebo Classic do
  projeto OnRobot.
- Adiciona e ativa o controlador interno
  `onrobot_joint_position_controller` e um adaptador deterministico cuja
  interface comum recebe a largura em metros por
  `/finger_width_controller/commands`.
- Remove as tags `mimic` somente da copia instalada na imagem e comanda
  explicitamente as sete juntas da gripper, preservando o movimento fisico no
  motor DART do Gazebo Harmonic.
- Preserva o `forward_velocity_controller` do braco e nao introduz MoveIt na
  arquitetura.
- Integra `tonydle/OnRobot_ROS2_Driver` como backend do hardware real e o
  compila no commit
  `b99abaccfbbe90f2096feff833f4c0849757a587`.
- Integra o RG2 real em `/onrobot/controller_manager`, preservando como interface
  externa comum `/finger_width_controller/commands` e mantendo o gerenciador do
  braco isolado em `/controller_manager`.
- Suporta Modbus serial pelo Tool I/O do UR, com 1 Mbaud, paridade par, um stop
  bit e 24 V, ou Modbus TCP pela OnRobot Compute Box.
- Instala explicitamente `libnet1-dev`, dependencia nativa da implementacao
  Modbus TCP que nao e declarada ao `rosdep` pelo driver OnRobot upstream.
- Atrasa a inicializacao do driver RG para permitir a criacao de `/tmp/ttyUR` e
  publica a geometria real acoplada a `tool0` a partir da largura medida.
- Faz `make init` migrar o `.env` automaticamente para `IMAGE_TAG=0.2.0` e
  `ONROBOT_TYPE=rg2`, preservando configuracoes locais, e adiciona
  `make configure-real` para gravar os parametros de rede sem edicao manual.
- Atualiza a imagem para `ur-cbf-jazzy:0.2.0` e adiciona testes dos limites,
  TCPs e selecao dos modelos RG2/RG6.

## Revisao 0.4.0 / infraestrutura 0.1.9 — 20 de agosto de 2026

- Adiciona o OSQP 1.1.3 e formula o controlador cartesiano nominal como um QP
  convexo com regularizacao equivalente ao amortecimento da solucao DLS.
- Impoe os limites simetricos de velocidade articular dentro do otimizador.
- Preserva `controller_mode:=dls` para comparacoes na mesma imagem e usa
  `controller_mode:=qp` como modo padrao.
- Reutiliza o workspace e o warm start do OSQP quando a dimensao e preservada.
- Publica comando nulo e reprova o ensaio se o resolvedor falhar, exceder o limite
  de tempo, retornar estado invalido ou violar limites acima da tolerancia.
- Registra por amostra o status do QP, iteracoes, tempos, residuos, restricoes
  ativas e violacao numerica maxima, com resumo agregado no JSON experimental.
- Adiciona oito testes unitarios para equivalencia QP-DLS, limites, reuso,
  dimensao variavel, mapeamento por nomes e falhas do resolvedor, elevando o
  total do pacote para 38 testes.
- Atualiza a imagem para `ur-cbf-jazzy:0.1.9`, o pacote `ur_cbf_bringup` para
  `0.1.9` e o pacote `ur_cbf_control` para `0.4.0`.

## Revisao 0.3.2 — 20 de agosto de 2026

- Mede o limite dinamico de `30 s` pelo relogio ROS/Gazebo, tornando o criterio
  independente do fator de tempo real da simulacao.
- Mantem watchdogs de comunicacao em relogio monotonicamente crescente e adiciona
  um limite absoluto de seguranca de `180 s` reais.
- Usa tempo simulado tambem nas fases de estabilizacao e confirmacao da parada.
- Registra `t_sim`, `t_real` e o fator de tempo real aproximado no log.
- Gera um JSON atomico por execucao com versoes, parametros, seed, ordem das
  juntas, posicoes, metricas e serie temporal de erros e comandos.
- Publica comando nulo imediatamente ao solicitar uma interrupcao do ensaio.
- Adiciona oito testes de temporizacao, retrocesso de relogio e persistencia,
  elevando o total do pacote para 30 testes unitarios.
- Mantem a infraestrutura Docker `ur-cbf-jazzy:0.1.8`.

## Revisao 0.3.1 / infraestrutura 0.1.8 — 20 de agosto de 2026

- Disponibiliza o `site-packages` do ambiente virtual do UAIbot aos executaveis
  ROS gerados pelo `colcon`, que usam `/usr/bin/python3` no shebang.
- Preserva a precedencia dos prefixos Python do ROS e acrescenta o ambiente
  virtual ao final de `PYTHONPATH` no entrypoint.
- Amplia o diagnostico para importar UAIbot 1.2.7 diretamente com o interpretador
  do ROS, evitando validar somente o Python do ambiente virtual.
- Atualiza a imagem para `ur-cbf-jazzy:0.1.8` e o pacote `ur_cbf_bringup` para
  `0.1.8`.
- Atualiza o pacote `ur_cbf_control` para `0.3.1` e o tempo maximo do ensaio
  cartesiano de `8 s` para `30 s`.
- O ajuste preservou ganhos, amortecimento, referencia, limites e criterio de
  tolerancia. A repeticao atingiu a convergencia, mas revelou que o limite ainda
  dependia do tempo real e do desempenho grafico do host.

## Revisao 0.3.0 — 20 de agosto de 2026

- Adiciona a regulacao cartesiana nominal de posicao antes da introducao do QP e
  das CBFs.
- Usa o UAIbot 1.2.7 para calcular a posicao do efetuador e o Jacobiano
  translacional a partir do estado articular medido no Gazebo.
- Implementa inversa de minimo quadrado amortecida, limitacao da velocidade
  cartesiana e saturacao simetrica das velocidades articulares.
- Torna explicitas em YAML a ordem das juntas do modelo, a transformacao do ponto
  controlado, ganhos, amortecimento, limites, frequencia e tolerancias.
- Acrescenta watchdog, parada nula, armamento explicito e bloqueio do ensaio fora
  do backend `/gz_ros_control`.
- Adiciona testes de singularidade, dimensao variavel, mapeamento por nomes,
  saturacoes, falhas numericas e rejeicao de modelos sem adaptador.
- Mantem a imagem Docker consolidada `ur-cbf-jazzy:0.1.7`.

## Revisao 0.2.1 — 20 de agosto de 2026

- Corrige a descoberta dos testes do pacote `ament_python` pelo `colcon`,
  declarando `pytest` no grupo opcional `test` de `setup.py`.
- Substitui a dependencia de teste do manifesto por `python3-pytest`, conforme o
  mecanismo utilizado pelo executor Python do `colcon` no ROS 2 Jazzy.
- Mantem inalterados o comportamento do ensaio, seus parametros de seguranca e a
  imagem Docker consolidada `ur-cbf-jazzy:0.1.7`.

## Revisao 0.2.0 — 20 de agosto de 2026

- Adiciona o pacote ROS 2 `ur_cbf_control` para iniciar a validacao da camada de
  controle sem alterar a imagem Docker consolidada `ur-cbf-jazzy:0.1.7`.
- Introduz um ensaio monoarticular de baixa velocidade, desarmado por padrao e
  restrito ao backend de simulacao pela presenca de `/gz_ros_control`.
- Consulta a ordem das juntas no `forward_velocity_controller` e reordena
  `/joint_states` pelos nomes recebidos, sem assumir a ordem da mensagem.
- Aplica saturacao simetrica e comando nulo durante estabilizacao, parada,
  timeout, interrupcao ou falha de validacao.
- Acrescenta testes unitarios para reordenacao, estados invalidos, saturacao,
  dimensao do comando e deteccao de estado obsoleto.

## Revisao 0.1.7 — 23 de julho de 2026

- Integra ao alvo `make sim` a autorizacao grafica X11/XWayland que antes
  precisava ser executada manualmente com `xhost`.
- Restringe a autorizacao ao usuario local que inicia o ambiente, sem habilitar
  acesso global ao servidor X.
- Verifica previamente a existencia de `DISPLAY` e do comando `xhost`, retornando
  orientacoes objetivas quando o host nao estiver preparado.
- Remove a criacao e a montagem de `~/.Xauthority`, que podiam produzir um
  arquivo vazio e impedir a inicializacao da interface Qt do Gazebo.
- Atualiza a imagem consolidada para `ur-cbf-jazzy:0.1.7`.
- Atualiza os metadados do pacote ROS `ur_cbf_bringup` para a versao `0.1.7`.

## Revisao 0.1.6 — 23 de julho de 2026

- Corrige o alvo `make build` para ativar explicitamente o perfil `dev` e
  construir o servico `ur_cbf_dev`.
- Elimina a resposta enganosa `No services to build`, que mantinha em uso a
  imagem antiga sem `ros-jazzy-ros2controlcli`.
- Versiona a imagem Docker como `ur-cbf-jazzy:0.1.6`, evitando que revisoes
  diferentes compartilhem silenciosamente a tag `latest`.
- O diagnostico agora diferencia a instalacao do pacote Debian
  `ros-jazzy-ros2controlcli` do registro da extensao `ros2 control`.

## Revisao 0.1.5 — 23 de julho de 2026

- Instala explicitamente o pacote `ros-jazzy-ros2controlcli`, responsavel por
  registrar a extensao `control` no comando `ros2`.
- Acrescenta ao `make check` uma verificacao autossuficiente de
  `ros2 control list_controllers -h`, sem exigir um `controller_manager` ativo.
- Mantem a simulacao UR na versao 2.5.0 e a interface de velocidades articulares
  pelo `forward_velocity_controller`.

## Revisao 0.1.4 — 23 de julho de 2026

- Substitui o commit `90fa0ee`, exclusivo da branch `ros2` e posterior a uma
  alteracao incompatível, pelo tag oficial `2.5.0` da simulacao UR.
- Fixa `ur_simulation_gz` no commit
  `048c80cd1faf87a2c74e14baadb65bd22b564d8f`, pertencente à ancestralidade da
  branch `kilted`, utilizada para ROS 2 Jazzy.
- Acrescenta ao `make diagnose` a referência exata da simulação UR usada na
  construção.

## Revisao 0.1.3 — 23 de julho de 2026

- Corrige a falha `COLCON_TRACE: unbound variable` ao carregar o ambiente
  gerado pelo `colcon`.
- Mantem `set -euo pipefail` em todo o diagnostico e suspende `nounset` somente
  durante o `source` de `install/setup.bash`.

## Revisao 0.1.2 — 23 de julho de 2026

- Corrige o e-mail invalido `cayo@localhost` no manifesto do pacote
  `ur_cbf_bringup`.
- Carrega explicitamente o ambiente instalado do workspace no diagnostico,
  garantindo que o pacote local seja encontrado apos a compilacao.
- Substitui a mensagem generica `Package not found` por um diagnostico que
  identifica o pacote ausente e exibe o `AMENT_PREFIX_PATH`.

## Revisao 0.1.1 — 23 de julho de 2026

- Atualiza o indice APT imediatamente antes de o `rosdep` instalar as dependencias
  da simulacao oficial da Universal Robots.
- Reutiliza e renomeia o usuario e o grupo de UID/GID 1000 ja presentes na imagem
  `ubuntu:24.04`, evitando a falha `GID '1000' already exists`.
- Acrescenta `make diagnose` para confirmar o projeto e o Dockerfile usados pelo
  Docker Compose antes da construcao.

| Componente | Versao ou referencia |
|---|---|
| Sistema base | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy |
| Python | 3.12 |
| UAIbot | 1.2.7 |
| Gazebo | Harmonic |
| Simulacao UR | tag `2.5.0`, commit `048c80cd1faf87a2c74e14baadb65bd22b564d8f` |
| Descricao OnRobot | commit `29180b3fa9cba6555f3e515e789b8ccd34252fab` |
| Controlador ROS | `forward_velocity_controller` |

## Evidencia de compatibilidade do UAIbot

O pacote binario `uaibot==1.2.7` foi instalado e testado em CPython 3.12.13. O teste
confirmou a importacao da biblioteca, a existencia de `Robot.create_ur_ur3e()` e a
criacao de um modelo com seis elos.

O arquivo `/opt/ur_cbf_python_versions.txt`, gerado durante a construcao da imagem,
registra todas as dependencias Python efetivamente instaladas.

## Politica de reproducibilidade

- O UAIbot e fixado por versao.
- A simulacao UR e fixada por commit.
- Pacotes ROS instalados por APT recebem atualizacoes compatíveis com Jazzy durante a
  construcao. Para cada campanha experimental, deve-se arquivar tambem a saida de
  `apt list --installed` e o identificador da imagem Docker utilizada.
