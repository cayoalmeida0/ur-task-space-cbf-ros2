# `ur_cbf_control`

Pacote de controle seguro desenvolvido sobre a infraestrutura Docker `v0.1.7`.
Ele contem o ensaio da cadeia de velocidades articulares e a primeira regulacao
cartesiana nominal de posicao, ainda sem QP ou CBF.

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

## Ensaio monoarticular no Gazebo

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

## Controlador cartesiano nominal

Para a posicao do efetuador `p`, referencia `p_d` e Jacobiano translacional `J_v`,
o comando nominal implementa:

```text
v = K_p (p_d - p)
qdot = J_v^T (J_v J_v^T + lambda^2 I)^-1 v
```

O UAIbot 1.2.7 fornece `p` e `J_v`. O vetor `q` vem exclusivamente de
`/joint_states`, cuja ordem e convertida explicitamente para a ordem configurada
do modelo. A saida e convertida novamente para a ordem consultada no parametro
`joints` do `forward_velocity_controller`.

Protecoes adicionais:

- referencia relativa ao estado estabilizado, inicialmente `10 mm` em `z`;
- limitacao da norma da velocidade cartesiana;
- saturacao simetrica das velocidades articulares;
- inversa amortecida para manter solucao finita em singularidades;
- watchdog baseado no instante monotonicamente crescente de recepcao do estado;
- comando nulo durante espera, estabilizacao, parada e falhas;
- armamento explicito e bloqueio quando `/gz_ros_control` esta ausente;
- identificador, seed e parametros registrados no log do ensaio;
- rejeicao explicita de modelos sem adaptador cinetico UAIbot.

Com `make sim` ativo em outro terminal:

```bash
ros2 launch ur_cbf_control cartesian_position.launch.py \
  ur_type:=ur3e \
  execute_test:=true
```

Os parametros estao em `config/cartesian_position.yaml`. A transformacao do ultimo
frame DH ao ponto controlado e explicita em `eef_offset_xyz`; o valor inicial
`[0.0, 0.0, 0.2]` preserva o ponto de efetuador da fabrica UR3e do UAIbot 1.2.7.

Esta revisao suporta o UR3e no adaptador UAIbot. Modelos adicionais devem declarar
sua fabrica e a ordem de juntas correspondente; a execucao e recusada se o modelo
nao estiver implementado.
