# Registro inicial de versoes

Data da definicao do ambiente: 17 de julho de 2026.

## Revisao experimental 0.6.0 / controle 0.6.0 / bringup 0.3.0 — 21 de agosto de 2026

- Formula a primeira CBF cinemática de autocolisao como
  `J_d qdot >= -gamma (d-d_safe)` para pares nao adjacentes.
- Generaliza o QP OSQP para desigualdades CBF, preservando limites articulares,
  warm start, diagnostico de restricoes ativas e verificacao de viabilidade.
- Adiciona os modos `off`, `monitor` e `enforce`; o padrao permanece `off` ate a
  validacao geometrica e temporal no container.
- Usa explicitamente `compute_dist_auto` do UAIbot em modo Python e registra a
  origem da geometria, o par mais proximo, a distancia e o valor da barreira.
- Substitui as nove aproximacoes visuais anteriores pelas mesmas 19 primitivas
  da fabrica UR3e do UAIbot: 14 cilindros, duas esferas e tres caixas.
- Converte as matrizes dos frames DH para os frames de elo da descricao oficial
  Jazzy e adiciona regressao para nomes, tipos, poses e dimensoes dos 19 objetos.
- Valida o wheel UAIbot em tempo de execucao e interrompe o modo de autocolisao
  se contagem, tipo, matriz ou dimensao divergir da copia visual versionada.
- Remove a capsula RG2 separada: a visualizacao distal passa a exibir a mesma
  garra generica usada pelo calculo, evitando sobreposicao de modelos distintos.
- Registra que a coerencia visual/matematica nao certifica fidelidade dimensional
  da garra generica UAIbot em relacao a RG2 fisica; esta revisao ainda nao esta
  liberada para o robo real.
- Atualiza o esquema experimental para `1.4` e adiciona testes da formulacao,
  conversao das distancias e integracao das desigualdades no QP.

## Revisao de consolidacao 0.5.7 — 21 de agosto de 2026

- Reorganiza a documentacao em uma entrada rapida e guias dedicados para
  instalacao, simulacao e hardware real, preservando os procedimentos validados.
- Atualiza o estado do projeto, a estrutura do repositorio, a citacao cientifica
  e os avisos de licencas das dependencias fixadas.
- Corrige o aviso do driver OnRobot: o backend Modbus real e compilado na imagem
  Docker desde a infraestrutura `0.2.0`.
- Amplia `check_system.sh` para exigir tambem o pacote local `ur_cbf_control`.
- Reduz o contexto enviado ao Docker e exclui resultados, caches de teste e
  configuracoes de IDE.
- Adiciona uma verificacao rapida no GitHub para sintaxe Bash/Python, XML/Xacro e
  testes unitarios independentes do ROS; a validacao completa permanece no
  container Jazzy.
- Mantem inalteradas as versoes funcionais da imagem (`0.2.0`), do bringup
  (`0.2.4`) e do controle (`0.5.1`).

## Revisao experimental 0.5.6 / bringup 0.2.4 — 21 de agosto de 2026

- Ajusta somente o roteiro `make test-cbf-motion` para
  `state_timeout=1.0 s`, tolerando pausas ocasionais de `/joint_states`
  quando Gazebo, RViz e o container de teste disputam recursos graficos.
- Mantem o limite conservador de `0.25 s` no ensaio monoarticular padrao e nas
  demais configuracoes de controle.
- Preserva a interrupcao segura e o comando nulo caso a ausência de estado
  ultrapasse o novo limite durante o ensaio visual.

## Revisao visual 0.5.5 / bringup 0.2.4 — 21 de agosto de 2026

- Corrige `CBF_VOLUMES_GAZEBO=false` gerando descricoes Xacro independentes
  para o `robot_state_publisher`/RViz e para `ros_gz_sim create`/Gazebo.
- Remove a dependencia de `visibility_flags` aplicada a links fixos sem
  inercia, pois esses links sao agrupados durante a conversao URDF para SDFormat.
- Mantem os nove volumes no RViz e os remove integralmente apenas da entidade
  criada no Gazebo quando a chave exclusiva esta desativada.
- Adiciona regressao para as quatro combinacoes de `CBF_VOLUMES` e
  `CBF_VOLUMES_GAZEBO`.
- Adiciona `make test-cbf-motion`, um ensaio somente para simulacao com seis
  pulsos protegidos, deslocamento nominal de `0.6 rad` e retorno das tres juntas.
- Atualiza `ur_cbf_bringup` para `0.2.4`.

## Revisao visual 0.5.4 / bringup 0.2.3 — 21 de agosto de 2026

- Substitui `wrist_1_connector` por um cilindro de `0.10405 m`, removendo as duas
  esferas terminais que coincidiam com a capsula do antebraco e com a esfera de
  `wrist_1_link`.
- Remove `elbow_cbf_volume`, pois as capsulas deslocadas do braco e antebraco ja
  se sobrepoem em `0.047 m` e cobrem o centro da junta 3 com margem de `0.038 m`.
- Reduz a representacao do conjunto UR3e/RG2 para nove volumes sem criar lacunas
  na uniao geometrica.
- Preserva os offsets fisicos, a capsula unica da RG2 e os controles independentes
  `CBF_VOLUMES` e `CBF_VOLUMES_GAZEBO`.
- Atualiza `ur_cbf_bringup` para `0.2.3` e adiciona testes contra a reintroducao
  das tres esferas redundantes.

## Revisao visual 0.5.3 / bringup 0.2.2 — 21 de agosto de 2026

- Desloca a capsula do braco em `0.120 m` e a do antebraco em `0.027 m`, seguindo
  `shoulder_offset` e `elbow_offset` da descricao fisica oficial do UR3e Jazzy.
- Preserva as esferas nos frames articulares, distinguindo os eixos DH da linha
  central dos corpos mecanicos deslocados.
- Adiciona uma capsula de `0.10405 m` na extremidade do antebraco para completar
  a cobertura ate `wrist_1_link`, posterior a junta 4.
- Mantem todos os novos elementos exclusivamente visuais, sem alterar colisao,
  inercia ou interfaces do `ros2_control`.
- Adiciona `CBF_VOLUMES_GAZEBO=false` para ocultar as aproximacoes somente no
  Gazebo por `visibility_flags=0`, preservando o `RobotModel` completo no RViz.
- Atualiza `ur_cbf_bringup` para `0.2.2` e adiciona regressao para os tres offsets
  oficiais e para os controles independentes de visualizacao.

## Revisao visual 0.5.2 / bringup 0.2.1 — 21 de agosto de 2026

- Adiciona aproximacoes geometricas sem colisao para inspecao no RViz e no
  Gazebo antes da formulacao das CBFs.
- Cobre o UR3e com uma esfera na base, esferas no ombro, cotovelo e punhos e
  capsulas no braco e antebraco, todas presas por juntas fixas aos elos.
- Representa toda a RG2 por uma unica capsula, sem criar volumes separados para
  os dedos.
- Mantem os volumes exclusivamente visuais, sem massa, inercia, colisao ou
  interfaces de controle; a fisica da simulacao permanece inalterada.
- Condiciona as dimensoes do braco ao modelo UR3e para impedir que sejam
  aplicadas silenciosamente a outro manipulador.
- Adiciona `CBF_VOLUMES`, habilitado por padrao, e permite comparar a cena com
  `make sim CBF_VOLUMES=false`, sem edicao manual do `.env`.
- Atualiza `ur_cbf_bringup` para `0.2.1` e adiciona testes de estrutura,
  cobertura, separacao RG2/RG6 e propagacao da chave pelo Compose.

## Revisao 0.5.1 / infraestrutura 0.2.0 — 20 de agosto de 2026

- Corrige no adaptador do UAIbot o quinto parametro DH do UR3e de `0.10535 m`
  para o valor `0.08535 m` da descricao oficial Jazzy.
- Mantem inalterado o TCP fisico da RG2 em `0.218 m`; a diferenca de `20 mm` nao
  e tratada como deslocamento de ferramenta porque afeta posicao e Jacobiano de
  forma dependente da configuracao.
- Aceita tanto o valor conhecido do UAIbot 1.2.7 quanto um modelo futuro ja
  corrigido, mas interrompe a execucao se encontrar qualquer terceiro valor.
- Documenta as convencoes de `base_link`, `base`, `tool0` e `gripper_tcp` e
  estabelece `base -> gripper_tcp` como comparacao correta entre TF e UAIbot.
- Registra nome e correcoes do modelo cinemático no esquema experimental `1.3`.
- Forca o caminho Python do UAIbot no adaptador UR3e, garantindo que DH e TCP
  corrigidos nao sejam ignorados por uma copia C++ criada anteriormente.
- Atualiza `ur_cbf_control` para `0.5.1` e adiciona quatro testes de regressao,
  elevando o total do pacote para 48 testes.

## Revisao 0.5.0 / infraestrutura 0.2.0 — 20 de agosto de 2026

- Substitui o deslocamento generico de `0.2 m` da fabrica UR3e do UAIbot pelo
  frame operacional `gripper_tcp`, localizado no centro dos dedos fechados.
- Seleciona automaticamente a transformacao rigida a partir de `onrobot_type`:
  `0.218 m` para RG2 e `0.268 m` para RG6, com a orientacao de montagem
  `RPY=[0, 0, -pi/2]` definida na descricao OnRobot.
- Faz posicao e Jacobiano translacional do controlador DLS/QP referenciarem o
  TCP real da gripper, mantendo `tool0` somente como flange mecanica.
- Registra modelo da gripper, frame controlado e transformacao completa no JSON
  experimental, cujo esquema passa para `1.2`.
- Atualiza `ur_cbf_control` para `0.5.0` e adiciona testes das transformacoes RG2,
  RG6 e da convencao RPY fixa usada pelo URDF.

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
  explicitamente as seis juntas fisicas da gripper. A junta virtual
  `finger_width` permanece na interface abstrata e na visualizacao, mas nao e
  registrada como hardware inexistente no Gazebo.
- Adiciona o prefixo `share` do pacote `onrobot_description` a
  `GZ_SIM_RESOURCE_PATH`, permitindo que o Gazebo Harmonic resolva os meshes
  `package://` da RG2/RG6 alem da visualizacao ja disponivel no RViz.
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
