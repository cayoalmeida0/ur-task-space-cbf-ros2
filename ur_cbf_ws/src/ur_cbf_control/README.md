# `ur_cbf_control`

Pacote de controle seguro desenvolvido sobre a infraestrutura `v0.1.7`. O primeiro
ensaio valida a cadeia de velocidades articulares no Gazebo antes da implementacao
do controle cartesiano nominal.

## Protecoes do ensaio de pulso

- consulta a ordem das juntas no parametro `joints` do controlador;
- reordena `/joint_states` por nome;
- deriva a dimensao do comando da configuracao recebida;
- rejeita estados incompletos, duplicados, `NaN` ou infinitos;
- limita a velocidade solicitada;
- usa relogio monotonicamente crescente no watchdog;
- publica zero durante estabilizacao, parada, timeout e interrupcao;
- exige armamento explicito;
- recusa o ensaio quando `/gz_ros_control` nao esta presente.

## Compilacao e testes

Dentro do container de desenvolvimento:

```bash
cd /workspace/ur_cbf_ws
colcon build --symlink-install --packages-select ur_cbf_control
source install/setup.bash
colcon test --packages-select ur_cbf_control --event-handlers console_direct+
colcon test-result --verbose
```

## Ensaio no Gazebo

Com `make sim` ativo em outro terminal:

```bash
ros2 launch ur_cbf_control joint_velocity_pulse.launch.py \
  target_joint:=shoulder_pan_joint \
  execute_test:=true
```

Parametros iniciais: `0.03 rad/s` durante `0.5 s`, limitados a `0.05 rad/s`. O
deslocamento esperado e aproximadamente `0.015 rad`. O ensaio e aprovado se o
movimento ocorrer no sentido comandado, dentro da tolerancia configurada, sem
deslocamento excessivo das demais juntas e com estado final valido.

O executavel nao deve ser usado com `make real`.
