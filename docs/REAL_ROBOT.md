# Backend do robô real

O backend real preserva as mesmas interfaces externas da simulação:

- braço: `/forward_velocity_controller/commands`;
- gripper: `/finger_width_controller/commands`.

O UR usa `ur_robot_driver` em `/controller_manager`. A RG2 usa o driver OnRobot
em `/onrobot/controller_manager`, por Modbus serial no Tool I/O ou por uma
Compute Box TCP.

## Preparação

1. coloque o host Ubuntu e o controlador UR na mesma rede;
2. instale e configure o External Control URCap;
3. para a RG2 serial, instale o RS485 Daemon URCap exigido pelo driver OnRobot;
4. mantenha o acionamento desabilitado durante a primeira inspeção;
5. configure sem editar `.env`:

```bash
make configure-real ROBOT_IP=192.168.0.10
make build
make check
```

O modo serial é o padrão. O launch habilita Tool Communication, cria
`/tmp/ttyUR` e usa 1 Mbaud, paridade par, um stop bit e 24 V.

Para uma Compute Box Modbus TCP:

```bash
make configure-real \
  ROBOT_IP=192.168.0.10 \
  ONROBOT_CONNECTION_TYPE=tcp \
  ONROBOT_IP=192.168.1.1 \
  ONROBOT_PORT=502
```

## Inspeção sem movimento

Inicie:

```bash
make real
```

O perfil usa rede Docker `host`, necessária para as conexões com o controlador.
Em outro terminal:

```bash
make shell
ros2 control list_controllers
ros2 control list_controllers -c /onrobot/controller_manager
ros2 topic echo /onrobot/joint_states --once
ros2 topic info /finger_width_controller/commands
```

O resultado esperado inclui `forward_velocity_controller` ativo no gerenciador
do UR e `finger_width_controller` ativo no gerenciador OnRobot.

Para testar somente o UR, sem iniciar a RG2:

```bash
docker compose --env-file .env -f docker/compose.yaml --profile dev run --rm \
  ur_cbf_dev ros2 launch ur_cbf_bringup real.launch.py \
  ur_type:=ur3e robot_ip:=192.168.0.10 launch_gripper:=false
```

## Primeiro comando da RG2

Somente depois da inspeção, com área livre e parada de emergência acessível:

```bash
ros2 topic pub --once /finger_width_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.08]}"
```

O adaptador rejeita valores não finitos, vetores com dimensão diferente de um e
larguras fora de `[0, 0.110] m`. O driver upstream aplica internamente metade da
força máxima do modelo; a força ainda não é um parâmetro ROS exposto.

## Lista de segurança obrigatória

Antes de qualquer movimento real:

1. carregue e valide a calibração específica do manipulador;
2. confirme a ordem das juntas e os controladores ativos;
3. limite velocidades e acelerações a valores conservadores;
4. mantenha área de trabalho livre e botão de emergência acessível;
5. teste sem carga e em velocidade reduzida;
6. confirme que o watchdog envia velocidade articular nula se o controlador
   parar ou perder estado;
7. não reutilize diretamente um roteiro marcado como exclusivo da simulação.

Endereços IP, calibrações e dados específicos da instalação não devem ser
versionados.
