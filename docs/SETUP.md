# Instalação e diagnóstico

Este guia cobre a preparação do host, a criação automática do ambiente local, o
build da imagem e os problemas mais comuns. Os comandos partem da raiz do
repositório.

## Requisitos do host

- Docker Engine e plugin Docker Compose;
- sessão gráfica X11/XWayland ou WSLg para Gazebo e RViz;
- `xhost` disponível no host Linux;
- acesso HTTPS dos containers ao Ubuntu, GitHub e PyPI durante o build.

Confirme:

```bash
docker --version
docker compose version
echo "$DISPLAY"
command -v xhost
```

No Ubuntu, instale `xhost` com:

```bash
sudo apt install x11-xserver-utils
```

## Clone e configuração automática

```bash
git clone https://github.com/cayoalmeida0/ur-task-space-cbf-ros2.git
cd ur-task-space-cbf-ros2
make init
make diagnose
```

`make init` cria ou migra `.env` a partir de `.env.example`. As chaves gerenciadas
pela infraestrutura são sincronizadas automaticamente:

```dotenv
IMAGE_TAG=0.2.0
ONROBOT_TYPE=rg2
CBF_VOLUMES=true
CBF_VOLUMES_GAZEBO=true
```

Valores locais já preenchidos, como `ROBOT_IP`, `ROS_DOMAIN_ID` e parâmetros da
Compute Box, são preservados. Para configurar o robô real sem editar `.env`:

```bash
make configure-real ROBOT_IP=192.168.0.10
```

Com OnRobot Compute Box por Modbus TCP:

```bash
make configure-real \
  ROBOT_IP=192.168.0.10 \
  ONROBOT_CONNECTION_TYPE=tcp \
  ONROBOT_IP=192.168.1.1 \
  ONROBOT_PORT=502
```

## Revisões externas fixadas

O diagnóstico mostra os commits usados para tornar a imagem reproduzível:

| Componente | Commit |
|---|---|
| Simulação Gazebo oficial do UR | `048c80cd1faf87a2c74e14baadb65bd22b564d8f` |
| Descrição OnRobot RG2/RG6 | `29180b3fa9cba6555f3e515e789b8ccd34252fab` |
| Driver OnRobot do hardware real | `b99abaccfbbe90f2096feff833f4c0849757a587` |

## Build e diagnóstico completo

```bash
make build
make check
```

O build cria `ur-cbf-jazzy:0.2.0`. O diagnóstico valida, entre outros itens:

- ROS 2 Jazzy e Python 3.12;
- UAIbot 1.2.7 e OSQP 1.1.3 nos interpretadores do venv e do ROS;
- Gazebo Harmonic e `ros2 control`;
- pacotes oficiais UR e OnRobot;
- pacotes locais `ur_cbf_bringup` e `ur_cbf_control`;
- descrições RG2/RG6 preparadas para as juntas explícitas;
- launches de simulação e hardware real.

Para entrar no ambiente:

```bash
make shell
```

Após modificar um pacote ROS, recompile dentro do container:

```bash
cd /workspace/ur_cbf_ws
colcon build --symlink-install
source install/setup.bash
```

## Compatibilidade com WSL 2

A arquitetura é compatível com WSL 2 porque usa containers Linux e aplicações
gráficas X11. O caminho recomendado é:

1. instalar WSL 2 com Ubuntu 24.04;
2. instalar Docker Desktop com integração habilitada para essa distribuição;
3. confirmar `docker run --rm hello-world` dentro do Ubuntu do WSL;
4. confirmar que WSLg fornece `DISPLAY`;
5. clonar o repositório no sistema de arquivos Linux, por exemplo em `~/`, e não
   em `/mnt/c`;
6. executar `make init`, `make build`, `make check` e `make sim`.

O projeto ainda não declara WSL 2 como plataforma validada em CI. Além disso, uma
rede corporativa pode permitir HTTPS no WSL e bloquear o mesmo tráfego originado
por containers. O sintoma observado é:

```text
TLS alert, handshake failure
```

seguido de uma mensagem enganosa do `pip`, como `No matching distribution found`.
Verifique de forma independente:

```bash
curl -4fsSIL https://pypi.org/simple/uaibot/

docker run --rm curlimages/curl:latest \
  -4fsSIL https://pypi.org/simple/uaibot/
```

Se o host funcionar e o container falhar, a causa está na política de rede,
proxy, inspeção TLS ou firewall, não na disponibilidade do UAIbot. Não desative a
verificação de certificados e não adicione `trusted-host` como correção. Solicite
ao administrador a liberação do tráfego TLS dos containers ou faça o build em
uma rede permitida.

## Interface gráfica

`make sim` verifica `DISPLAY` e autoriza automaticamente no servidor X somente o
usuário local que iniciou o ambiente. Se a conexão for recusada:

```bash
xhost +SI:localuser:"$(id -un)"
make sim
```

Para interromper e remover os containers do projeto:

```bash
make down
```

## Verificação mínima antes de publicar

Dentro do container:

```bash
./scripts/check_system.sh
cd /workspace/ur_cbf_ws
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

O `check_system.sh` depende do workspace já compilado. Cada experimento deve
registrar a imagem, parâmetros, seed e revisão do Git.
