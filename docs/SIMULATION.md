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

Na combinação UR3e/RG2, nove volumes sem colisão são presos aos elos do robô:

- esferas na base, ombro e punhos;
- cápsulas no braço e antebraço;
- cilindro entre a extremidade do antebraço e `wrist_1_link`;
- cápsula única envolvendo a RG2.

As cápsulas longas incorporam os deslocamentos físicos da descrição oficial do
UR3e: `0.120 m` no braço e `0.027 m` no antebraço. O conector da junta 4 cobre os
`0.10405 m` restantes até `wrist_1_link`. A esfera explícita do cotovelo foi
removida porque as cápsulas adjacentes já cobrem essa região.

Esses elementos possuem apenas `<visual>`: não têm `<collision>`, massa, inércia
ou interfaces de controle. Portanto, não alteram contato ou dinâmica. Ainda são
uma representação para inspeção; a geometria matemática das CBFs será
implementada separadamente.

As dimensões físicas foram confrontadas com os arquivos oficiais:

- [`physical_parameters.yaml`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/jazzy/config/ur3e/physical_parameters.yaml)
- [`default_kinematics.yaml`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/jazzy/config/ur3e/default_kinematics.yaml)

### Controle de visualização

| Comando | RViz | Gazebo |
|---|---|---|
| `make sim` | visíveis | visíveis |
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

### Ensaio visual de movimento

Com a simulação ativa, execute no host:

```bash
make test-cbf-motion
```

O roteiro aplica e desfaz deslocamentos nominais de `0.6 rad` em
`shoulder_pan_joint`, `elbow_joint` e `wrist_1_joint`, usando velocidades entre
`0.20 rad/s` e `0.30 rad/s`. Cada pulso verifica o deslocamento medido e termina
com comando nulo. O teste exige `/gz_ros_control`, recusa o robô real e usa
`state_timeout=1.0 s` para tolerar pausas ocasionais do simulador sob carga
gráfica.

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

Consulte a [documentação de `ur_cbf_control`](../ur_cbf_ws/src/ur_cbf_control/README.md)
para a formulação, proteções e critérios completos.
