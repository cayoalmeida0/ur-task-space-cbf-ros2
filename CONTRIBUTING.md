# Contribuindo

Este repositorio sustenta experimentos de controle restrito de manipuladores
Universal Robots. Mudancas devem preservar a mesma interface ROS nos backends de
simulacao e hardware real.

## Fluxo recomendado

1. Crie uma branch a partir de `main`.
2. Parametrize modelos, ganhos, frequencias, margens e limites; nao fixe valores
   especificos do UR3e no codigo novo.
3. Inclua testes para comportamentos de seguranca e para os modelos afetados.
4. Execute a verificacao minima antes de abrir um pull request:

```bash
./scripts/check_system.sh
cd ur_cbf_ws
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

O script `check_system.sh` deve ser executado dentro do container.

O workflow `Repository checks` executa uma verificacao rapida em pull requests,
mas nao substitui o teste acima: apenas o container do projeto possui o conjunto
completo de dependencias ROS 2, Xacro e Gazebo.

## Requisitos de seguranca

- Publique comando nulo quando o estado ou a solucao do QP estiverem obsoletos.
- Respeite limites articulares, de velocidade e de aceleracao.
- Valide primeiro em simulacao e depois em hardware real com velocidade reduzida.
- Nao inclua enderecos IP, calibracoes, credenciais ou dados especificos de uma
  instalacao no repositorio.

## Reprodutibilidade

Cada experimento deve registrar a versao da imagem Docker, o modelo do robo, os
arquivos de parametros, as seeds e as versoes das dependencias utilizadas.
