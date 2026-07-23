# Orientacoes do projeto UR CBF

## Objetivo

Este repositorio suporta experimentos de controle restrito no espaco de tarefa para
manipuladores Universal Robots, com CBFs, QPs e metricas de distancia diferenciaveis.

## Decisoes arquiteturais

- ROS 2 Jazzy sobre Ubuntu 24.04 e Python 3.12.
- Gazebo Harmonic e `gz_ros2_control` formam a planta simulada.
- O robo real usa `ur_robot_driver`.
- Simulacao e hardware recebem velocidades articulares por
  `/forward_velocity_controller/commands`.
- `ur_type` seleciona o modelo; `ur3e` e o padrao, mas o codigo novo nao deve fixar
  dimensoes, nomes de elos ou limites do UR3e.
- MoveIt nao participa da arquitetura de controle.
- UAIbot e usado como biblioteca matematica/geometrica, nao como segunda planta de
  simulacao. O Gazebo e a fonte do estado simulado.

## Convencoes para o desenvolvimento futuro

- Parametrizar modelo, frequencia, ganhos, margens e limites em arquivos YAML ou
  parametros ROS.
- Manter a mesma interface de topicos nos backends `simulation` e `real`.
- Ler a ordem das juntas da configuracao do robo; nao assumir ordem implicitamente.
- Publicar comando nulo quando dados de estado ou solucao do QP estiverem obsoletos.
- Adicionar testes para saturacao, limites articulares, CBFs e troca de modelo.
- Registrar versoes, parametros e seeds utilizados em cada experimento.

## Verificacao minima

```bash
./scripts/check_system.sh
cd ur_cbf_ws
colcon test --event-handlers console_direct+
colcon test-result --verbose
```
