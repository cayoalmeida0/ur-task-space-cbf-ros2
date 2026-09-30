# Simulação e ensaios

Este guia reúne os procedimentos visuais e funcionais do Gazebo, RViz, RG2/RG6,
volumes geométricos e camada nominal de controle.

## Iniciar e inspecionar

No host:

```bash
make sim
```

O launch inicia Gazebo, RViz, `joint_state_broadcaster`,
`forward_velocity_controller`, `onrobot_joint_position_controller` e o adaptador
de largura. A gripper é acoplada a `tool0` e publica `gripper_tcp`.

Em outro terminal:

```bash
make shell
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic info /forward_velocity_controller/commands
```

Os três controladores devem estar ativos:

```text
joint_state_broadcaster
forward_velocity_controller
onrobot_joint_position_controller
```

## Teste da gripper

A interface externa recebe a largura total em metros por
`Float64MultiArray`. O adaptador converte esse valor para as seis juntas físicas
exportadas pelo Gazebo. A junta virtual `finger_width` permanece somente na
abstração de largura e na descrição visual.

Para RG2, abra em 80 mm e feche em 20 mm:

```bash
ros2 topic pub --once /finger_width_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.08]}"

ros2 topic echo /joint_states --once

ros2 topic pub --once /finger_width_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.02]}"
```

A RG2 aceita larguras de `0` a `0.110 m`; a RG6, de `0` a `0.160 m`. Para um
ensaio pontual com RG6:

```bash
make down
make sim ONROBOT_TYPE=rg6
```

Os meshes instalados pelo pacote `onrobot_description` aparecem no Gazebo e no
RViz. O backend simulado usa Gazebo Harmonic e `gz_ros2_control`; não usa o
plugin Gazebo Classic do driver OnRobot.

## Volumes geométricos para as CBFs

O modelo UR3e/RG2 do projeto usa 19 primitivas matemáticas: 11 distribuídas
pelos elos do braço e os oito objetos originais da RG2 no elo final. A geometria
é validada contra o modelo UAIbot fixado e também publicada no RViz; os volumes
visuais adicionados pelo projeto não alteram massa, dinâmica ou contato físico.
As posições dos proxies acompanham a cadeia cinemática e os quatro objetos
móveis da RG2 acompanham a largura comandada.

As matrizes dos objetos são relativas aos frames DH posteriores às juntas. A
geometria do UR3e é corrigida para coincidir com a descrição oficial Jazzy, e a
RG2 preserva seus oito proxies, em vez de ser substituída por um único cilindro
e uma esfera. A fonte UAIbot está fixada no commit
[`1acb5ed`](https://github.com/UAIbot/UAIbotPy/blob/1acb5ed637738aca4ea05945e6c065c3757bc13d/uaibot/robot/_create_ur_ur3e.py).

As dimensões físicas foram confrontadas com os arquivos oficiais:

- [`physical_parameters.yaml`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/39242984dc8d1fff9584c922c17c69c58df3591d/config/ur3e/physical_parameters.yaml)
- [`default_kinematics.yaml`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/39242984dc8d1fff9584c922c17c69c58df3591d/config/ur3e/default_kinematics.yaml)

### Controle de visualização

| Comando | RViz | Gazebo |
|---|---|---|
| `make sim` | visíveis | ocultos |
| `make sim CBF_VOLUMES_GAZEBO=false` | visíveis | ocultos |
| `make sim CBF_VOLUMES=false` | ausentes | ausentes |

Reinicie a simulação ao alterar as opções:

```bash
make down
make sim CBF_VOLUMES_GAZEBO=false
```

O launch gera descrições Xacro independentes: o `robot_state_publisher` recebe a
árvore completa para o RViz, e `ros_gz_sim create` recebe a árvore apropriada ao
Gazebo. Isso evita depender de `visibility_flags` em links fixos agrupados pela
conversão URDF/SDFormat.

O envelope cartesiano da CBF é publicado no tópico
`/workspace/boundary_markers` e carregado automaticamente pela configuração
`ur_cbf.rviz`. Por padrão, ele utiliza `x=[-0,45; 0,45] m`, `y=[-0,55; 0,55] m`
e `z=[0,02; 0,90] m`, com margem de segurança de `0,05 m`; os parâmetros podem
ser alterados em `ur_cbf_control/config/cartesian_position.yaml`. Para impor as
seis restrições no QP, acrescente `workspace_cbf_mode:=enforce` ao lançamento do
ensaio; `monitor` apenas calcula e registra as barreiras.
O witness do plano mais próximo é publicado em
`/workspace/boundary_witness_markers`. A linha começa na superfície do proxy
de colisão mais próximo da face ativa e termina na face física. A CBF de
workspace no QP permanece baseada na posição do TCP; o log também mostra
`d_workspace_volume` e `workspace_volume` para identificar a geometria usada
pela visualização.

### Cena de manipulação

O mundo parametrizado contém uma mesa cilíndrica de raio `0,08 m`, um cubo de
aresta `0,04 m` e uma caixa aberta de `0,08×0,08×0,04 m`. A
posição da mesa, do cubo, da caixa e suas dimensões podem ser alteradas pelos
argumentos do launch (`table_x`, `table_y`, `table_height`, `cube_x`, `cube_y`,
`drop_x`, `drop_y`, entre outros).

O controlador de manipulação executa somente três waypoints cartesianos:
`cubo -> caixa -> HOME`. Não são adicionadas poses explícitas de aproximação,
elevação ou retração. Nos três launchers integrados, a mesa e o cubo são CBFs
externas em `enforce`; a CBF do cubo é avaliada por uma distância assinada à
superfície axis-aligned, dilatada pelo raio conservador de cada primitiva do
robô. Os quatro volumes móveis da RG2 ficam protegidos durante a aproximação e
são excluídos apenas na janela final de contato, quando o TCP já está a menos
de `0,06 m` do alvo; palma, punho e braço continuam protegidos.

### Volumes da cena e witness points

Quando `cube_cbf_mode` está em `monitor` ou `enforce`, o controlador publica a
caixa física e sua margem adicional no tópico
`/cube_collision/obstacle_marker`. O par de pontos testemunha mais próximo é
publicado em `/cube_collision/witness_markers`; ambos já estão configurados
como `MarkerArray` no RViz. O marcador é expresso no frame DH `base`, enquanto
`cube_position` continua sendo fornecido no frame da cena indicado por
`manipulation_object_frame`.

Na simulação, o modelo `manipulation_cube` possui um `PosePublisher` do Gazebo.
A ponte publica `/model/manipulation_cube/pose` como `PoseStamped`; o controlador
converte essa pose para o frame DH `base`. Assim, o volume visual e a posição
usada pela CBF acompanham o cubo depois da captura. Se a pose ficar ausente por
mais de `cube_pose_timeout`, o sistema usa temporariamente a pose estática
configurada em `cube_position`.

A geometria da mesa é publicada como cilindro em
`/table_collision/obstacle_marker`, e a caixa aberta de deposito é mostrada
como volume axis-aligned em `/drop_box/obstacle_marker`. Esses marcadores são
publicados durante o waypoint de controle mesmo quando as respectivas CBFs
estão em `off`. A distância até a caixa também é publicada em
`/drop_box/witness_markers` somente durante o segundo waypoint, mas permanece
monitor-only porque a caixa é o destino da tarefa. O tópico não é inserido no
QP e é limpo nos demais waypoints.

Com `closest`, as relações visuais ficam separadas em displays RViz:
autocolisão, mesa, cubo, fronteira e caixa (esta última somente no waypoint de
colocação). As linhas de mesa, cubo e caixa começam na superfície do proxy
esférico do robô e terminam na superfície física do obstáculo; a linha da
fronteira termina na face física ativa do envelope. A margem de segurança
continua somente no valor da barreira. Use `all` nos parâmetros
`self_collision_witness_mode`, `cylinder_witness_mode`, `cube_witness_mode` ou
`drop_box_witness_mode` quando precisar de todos os pares de uma família.

Para observar a CBF sem alterar o comando nominal, use `monitor`. Para ativá-la
no QP, use `enforce` com `controller_mode:=qp`:

```bash
ros2 launch ur_cbf_control cartesian_position.launch.py \
  task_type:=manipulation \
  cube_cbf_mode:=monitor \
  cube_witness_mode:=closest \
  execute_test:=true
```

### Ensaio visual de movimento

Mantenha `make sim` ativo no primeiro terminal. Em um segundo terminal do host,
fora de `make shell`, execute:

```bash
make test-cbf-motion
```

O roteiro aplica e desfaz deslocamentos nominais de `0.6 rad` em
`shoulder_pan_joint`, `elbow_joint` e `wrist_1_joint`, usando velocidades entre
`0.20 rad/s` e `0.30 rad/s`. Cada pulso verifica o deslocamento medido e termina
com comando nulo. O teste exige `/gz_ros_control`, recusa o robô real e usa
`state_timeout=1.0 s` para tolerar pausas ocasionais do simulador sob carga
gráfica.

O erro `ERRO: /gz_ros_control nao foi encontrado` significa que a simulação não
está ativa ou ainda não terminou de inicializar. O comando `make` não deve ser
executado dentro do container, pois o Docker pertence ao host.

### Witness points da autocolisão

O ensaio cartesiano publica os pontos testemunha e os segmentos entre os pares
avaliados em `/self_collision/witness_markers`. A configuração padrão do RViz já
inclui esse `MarkerArray`. A cor indica a margem em relação a `d_safe`: verde
acima de `1,5 d_safe`, amarelo entre `d_safe` e `1,5 d_safe`, e vermelho abaixo
de `d_safe`.

O parâmetro `self_collision_witness_mode` aceita `off`, `closest` e `all`. O
padrão `closest` mostra apenas o par de menor distância. Para exibir todos os
pares durante o monitoramento:

```bash
ros2 launch ur_cbf_control cartesian_position.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  controller_mode:=qp \
  self_collision_cbf_mode:=monitor \
  self_collision_witness_mode:=all \
  experiment_id:=self_collision_witness_all_001 \
  execute_test:=true
```

## Frames do efetuador

| Frame | Papel |
|---|---|
| `base_link` | frame visual do URDF |
| `base` | base industrial/DH, rotacionada em `pi` sobre `z` em relação a `base_link` |
| `tool0` | flange mecânica do UR |
| `gripper_tcp` | centro dos dedos fechados e ponto controlado |

Compare TF e UAIbot usando `base -> gripper_tcp`. Na configuração inicial do
UR3e/RG2, o valor esperado é aproximadamente `[0.000, -0.441, 0.694] m` em
`base`:

```bash
timeout 5 ros2 run tf2_ros tf2_echo base gripper_tcp
```

A RG2 usa TCP a `0.218 m`; a RG6, a `0.268 m`, ambas com a orientação rígida da
descrição OnRobot. O adaptador também corrige de forma controlada o quinto
parâmetro DH do UAIbot 1.2.7, de `0.10535 m` para o valor oficial `0.08535 m`.

## Teste da interface de velocidade

O ensaio é desarmado por padrão e só opera com `/gz_ros_control`:

```bash
ros2 launch ur_cbf_control joint_velocity_pulse.launch.py \
  target_joint:=shoulder_pan_joint \
  execute_test:=true
```

Ele consulta a ordem das juntas do controlador, reordena `/joint_states`, limita
o comando e publica zero em timeout, interrupção ou falha.

## Ensaio cartesiano DLS/QP

Modo QP:

```bash
ros2 launch ur_cbf_control cartesian_position.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  controller_mode:=qp \
  experiment_id:=cartesian_qp_ur3e_001 \
  execute_test:=true
```

Referência DLS comparável:

```bash
ros2 launch ur_cbf_control cartesian_position.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  controller_mode:=dls \
  experiment_id:=cartesian_dls_ur3e_001 \
  execute_test:=true
```

O alvo é relativo, inicialmente `10 mm` em `z`; cada execução cria um novo alvo.
O limite de controle é `30 s` simulados e o limite absoluto é `180 s` reais. O
resultado JSON é salvo em `/workspace/results` e inclui parâmetros, versões,
seed, ordem das juntas, erros e comandos. No QP, inclui também diagnóstico do
OSQP.

### Monitor da primeira CBF de autocolisão

Antes de impor a restrição, execute o mesmo ensaio em modo de observação:

```bash
ros2 launch ur_cbf_control cartesian_position.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  controller_mode:=qp \
  self_collision_cbf_mode:=monitor \
  experiment_id:=self_collision_monitor_ur3e_001 \
  execute_test:=true
```

O log deve informar `d_self_min`, `h_self_min`, `cbf_ms` e `par`. Esse modo não
altera o comando. Os volumes transparentes reproduzem o modelo corrigido
aplicado ao UAIbot; o roteiro de validação está documentado em
[SELF_COLLISION_CBF.md](SELF_COLLISION_CBF.md).

Consulte a [documentação de `ur_cbf_control`](../ur_cbf_ws/src/ur_cbf_control/README.md)
para a formulação, proteções e critérios completos.
