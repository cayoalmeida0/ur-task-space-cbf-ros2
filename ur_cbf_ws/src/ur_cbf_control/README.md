# `ur_cbf_control`

Pacote de controle seguro desenvolvido sobre a infraestrutura Docker `v0.2.0`.
Ele contem o ensaio da cadeia de velocidades articulares, a regulacao cartesiana
por DLS/QP e a primeira CBF cinemática de autocolisao.

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
o modo de referencia DLS implementa:

```text
v = K_p (p_d - p)
qdot = J_v^T (J_v J_v^T + lambda^2 I)^-1 v
```

O modo QP minimiza:

```text
1/2 ||J_v qdot - v||^2 + 1/2 lambda^2 ||qdot||^2
```

sujeito a `-qdot_max <= qdot <= qdot_max`. Quando as restricoes estao inativas,
essa solucao coincide numericamente com DLS. O OSQP 1.1.3 reutiliza o workspace e
o warm start entre iteracoes; qualquer status diferente de `solved` ou
`solved inaccurate` causa comando nulo e reprova o ensaio.

## CBF de autocolisao

Para cada par nao adjacente, a barreira e `h(q) = d(q) - d_safe`. Como o
comando e velocidade articular, a restricao entra diretamente no QP:

```text
J_d(q) qdot >= -gamma (d(q) - d_safe)
```

Use `self_collision_cbf_mode:=monitor` para calcular e registrar a distancia
minima sem alterar o comando. `enforce` acrescenta todas as linhas ao OSQP e so
e aceito com `controller_mode:=qp`; `off` e o padrao. Os 19 volumes transparentes
copiam as mesmas primitivas internas do UAIbot, após a conversao fixa dos frames
DH para os frames URDF. A garra representada continua sendo a geometria generica
da fabrica UAIbot, nao uma aproximacao dimensional certificada da RG2. Consulte
[`docs/SELF_COLLISION_CBF.md`](../../../docs/SELF_COLLISION_CBF.md) antes de
ativar a restricao.

O UAIbot 1.2.7 fornece `p` e `J_v`. O vetor `q` vem exclusivamente de
`/joint_states`, cuja ordem e convertida explicitamente para a ordem configurada
do modelo. A saida e convertida novamente para a ordem consultada no parametro
`joints` do `forward_velocity_controller`.

Protecoes adicionais:

- referencia relativa ao estado estabilizado, inicialmente `10 mm` em `z`;
- limitacao da norma da velocidade cartesiana;
- saturacao simetrica no modo DLS e limites internos ao otimizador no modo QP;
- inversa amortecida para manter solucao finita em singularidades;
- watchdog baseado no instante monotonicamente crescente de recepcao do estado;
- comando nulo durante espera, estabilizacao, parada e falhas;
- armamento explicito e bloqueio quando `/gz_ros_control` esta ausente;
- identificador, seed, versoes e parametros registrados no resultado do ensaio;
- rejeicao explicita de modelos sem adaptador cinetico UAIbot.

O tempo maximo de regulacao e `30 s` no relogio ROS/Gazebo. O watchdog de estado
continua usando relogio monotonicamente crescente, e um limite separado de `180 s`
reais encerra o ensaio em caso de lentidao extrema. Assim, a dinamica simulada nao
e interrompida apenas porque o fator de tempo real esta abaixo de um.

Com `make sim` ativo em outro terminal:

```bash
ros2 launch ur_cbf_control cartesian_position.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  controller_mode:=qp \
  experiment_id:=cartesian_qp_ur3e_001 \
  execute_test:=true
```

Primeiro ensaio de geometria, sem modificar o comando do QP:

```bash
ros2 launch ur_cbf_control cartesian_position.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  controller_mode:=qp \
  self_collision_cbf_mode:=monitor \
  experiment_id:=self_collision_monitor_ur3e_001 \
  execute_test:=true
```

Referencia DLS comparavel:

```bash
ros2 launch ur_cbf_control cartesian_position.launch.py \
  ur_type:=ur3e \
  onrobot_type:=rg2 \
  controller_mode:=dls \
  experiment_id:=cartesian_dls_ur3e_001 \
  execute_test:=true
```

O terminal informa `t_sim`, `t_real` e `RTF`. Ao finalizar, o diretorio
`/workspace/results` recebe um JSON com o resumo e a serie temporal completa. No
modo QP, o registro inclui status, iteracoes, tempos, residuos, limites ativos e
violacao numerica maxima das restricoes:

```bash
ls -1t /workspace/results/*.json | head -n 1
```

Cada nova execucao usa uma referencia relativa de `10 mm` a partir de sua propria
posicao inicial; portanto, repetir o comando cria um novo alvo, em vez de retomar
o alvo absoluto da execucao anterior.

Os parametros estao em `config/cartesian_position.yaml`. O ponto controlado e
`gripper_tcp`, localizado no centro dos dedos fechados. A transformacao rigida
do ultimo frame DH, equivalente a `tool0`, e selecionada por `onrobot_type`: RG2
usa translacao `[0.0, 0.0, 0.218] m` e RG6 usa `[0.0, 0.0, 0.268] m`; ambas usam
RPY `[0.0, 0.0, -pi/2]`, coerente com a montagem descrita no Xacro.

Para comparar o modelo com TF, use `base -> gripper_tcp`. O frame industrial
`base`, adotado pela convencao DH, difere do frame visual `base_link` por uma
rotacao de `pi` em `z`. O adaptador tambem corrige de forma versionada o quinto
parametro DH do UAIbot 1.2.7, de `0.10535 m` para o valor oficial `0.08535 m`.
Essa correcao atua antes do calculo de posicao e Jacobiano; o TCP da RG2 permanece
em `0.218 m`. Valores inesperados da dependencia interrompem a execucao, e toda
correcao aplicada e registrada no JSON experimental de esquema `1.3`.
O modo efetivo do UAIbot e `python`, garantindo que o DH e o TCP corrigidos nao
sejam substituidos por uma copia C++ criada anteriormente pela dependencia.

Esta revisao suporta o UR3e no adaptador UAIbot. Modelos adicionais devem declarar
sua fabrica e a ordem de juntas correspondente; a execucao e recusada se o modelo
nao estiver implementado.

## Referencias metodologicas

- WAMPLER, C. W. *Manipulator inverse kinematic solutions based on vector
  formulations and damped least-squares methods*. IEEE Transactions on Systems,
  Man, and Cybernetics, 1986. DOI: https://doi.org/10.1109/TSMC.1986.289285.
- STELLATO, B. et al. *OSQP: an operator splitting solver for quadratic programs*.
  Mathematical Programming Computation, 2020.
  DOI: https://doi.org/10.1007/s12532-020-00179-2.
- AMES, A. D. et al. *Control barrier function based quadratic programs for
  safety critical systems*. IEEE Transactions on Automatic Control, 2017.
  DOI: https://doi.org/10.1109/TAC.2016.2638961.
