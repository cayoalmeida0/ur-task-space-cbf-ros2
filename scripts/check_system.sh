#!/usr/bin/env bash
set -euo pipefail

expected_ros="jazzy"
expected_python_prefix="3.12"
workspace="${UR_CBF_WS:-/workspace/ur_cbf_ws}"

if [[ "${ROS_DISTRO:-}" != "${expected_ros}" ]]; then
  echo "ERRO: ROS_DISTRO=${ROS_DISTRO:-nao definido}; esperado ${expected_ros}." >&2
  exit 1
fi

# O entrypoint compila o workspace quando necessario. O diagnostico carrega
# novamente o ambiente para tambem funcionar em um processo filho independente.
if [[ ! -f "${workspace}/install/setup.bash" ]]; then
  echo "ERRO: workspace local ainda nao foi compilado: ${workspace}" >&2
  echo "Execute: cd ${workspace} && colcon build --symlink-install" >&2
  exit 1
fi

# Os scripts de ambiente gerados pelo colcon consultam variaveis opcionais,
# como COLCON_TRACE, sem valores padrao. Suspenda nounset somente durante o
# carregamento e restaure imediatamente o modo estrito do diagnostico.
set +u
source "${workspace}/install/setup.bash"
set -u

python_version="$(python3 -c 'import platform; print(platform.python_version())')"
if [[ "${python_version}" != ${expected_python_prefix}* ]]; then
  echo "ERRO: Python ${python_version}; esperado ${expected_python_prefix}.x." >&2
  exit 1
fi

python3 - <<'PY'
import osqp
import uaibot as ub

assert ub.__version__ == "1.2.7", ub.__version__
assert osqp.__version__ == "1.1.3", osqp.__version__
assert hasattr(ub.Robot, "create_ur_ur3e")
robot = ub.Robot.create_ur_ur3e(name="ur3e_check")
assert len(robot.links) == 6
print(f"UAIbot {ub.__version__}: UR3e criado com {len(robot.links)} elos")
print(f"OSQP {osqp.__version__}: resolvedor QP encontrado")
PY

# Reproduz o interpretador gravado nos executaveis Python gerados pelo colcon.
# Isso evita aprovar uma imagem na qual apenas o Python do venv importa UAIbot.
PYTHONNOUSERSITE=1 /usr/bin/python3 - <<'PY'
import sys
import osqp
import uaibot as ub

assert ub.__version__ == "1.2.7", ub.__version__
assert osqp.__version__ == "1.1.3", osqp.__version__
print(f"Python ROS {sys.executable}: acesso ao UAIbot {ub.__version__}: OK")
print(f"Python ROS {sys.executable}: acesso ao OSQP {osqp.__version__}: OK")
PY

packages=(
  onrobot_description
  onrobot_driver
  ur_description
  ur_controllers
  ur_robot_driver
  ur_simulation_gz
  ur_cbf_bringup
  ur_cbf_control
)
for package in "${packages[@]}"; do
  if ! ros2 pkg prefix "${package}" >/dev/null 2>&1; then
    echo "ERRO: pacote ROS nao encontrado no ambiente: ${package}" >&2
    echo "AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:-nao definido}" >&2
    exit 1
  fi
  echo "Pacote ROS encontrado: ${package}"
done

onrobot_share="$(ros2 pkg prefix onrobot_description)/share/onrobot_description"
for model_file in rg2_macro.xacro rg6_macro.xacro; do
  model_path="${onrobot_share}/urdf/${model_file}"
  if [[ ! -f "${model_path}" ]]; then
    echo "ERRO: descricao OnRobot nao encontrada: ${model_path}" >&2
    exit 1
  fi
  if grep -q '<mimic joint=' "${model_path}"; then
    echo "ERRO: ${model_file} ainda contem tags mimic incompativeis." >&2
    exit 1
  fi
done
echo "Descricao OnRobot preparada para juntas explicitas: OK"

if ! ros2 pkg executables ur_cbf_bringup \
    | grep -q 'ur_cbf_bringup onrobot_width_adapter'; then
  echo "ERRO: executavel onrobot_width_adapter nao encontrado." >&2
  exit 1
fi
echo "Adaptador de largura OnRobot encontrado: OK"

if ! ros2 pkg executables ur_cbf_bringup \
    | grep -q 'ur_cbf_bringup onrobot_real_adapter'; then
  echo "ERRO: executavel onrobot_real_adapter nao encontrado." >&2
  exit 1
fi
echo "Adaptador do RG real encontrado: OK"

# Distingue a ausencia do pacote Debian de uma falha no registro da extensao
# Python do comando ros2.
ros2_control_cli_package="ros-${ROS_DISTRO}-ros2controlcli"
if ! dpkg-query -W -f='${Status}\n' "${ros2_control_cli_package}" 2>/dev/null \
    | grep -qx "install ok installed"; then
  echo "ERRO: pacote Debian nao instalado: ${ros2_control_cli_package}." >&2
  exit 1
fi
echo "Pacote Debian encontrado: ${ros2_control_cli_package}"

# A extensao "control" pertence ao pacote ros2controlcli. A ajuda do
# subcomando valida sua instalacao sem depender de um controller_manager ativo.
if ! ros2 control list_controllers -h >/dev/null 2>&1; then
  echo "ERRO: extensao 'ros2 control' nao encontrada." >&2
  echo "Verifique a instalacao de ros-${ROS_DISTRO}-ros2controlcli." >&2
  exit 1
fi
echo "Extensao ROS 2 encontrada: ros2 control"

ros2 launch ur_cbf_bringup simulation.launch.py --show-args >/dev/null
ros2 launch ur_cbf_bringup real.launch.py --show-args >/dev/null

if ! command -v gz >/dev/null; then
  echo "ERRO: comando gz nao encontrado." >&2
  exit 1
fi

echo "ROS 2 ${ROS_DISTRO}: OK"
echo "Python ${python_version}: OK"
echo "Gazebo Harmonic: OK"
echo "Diagnostico concluido com sucesso."
