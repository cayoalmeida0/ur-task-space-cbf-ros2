#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source /opt/ur_vendor_ws/install/setup.bash

workspace="${UR_CBF_WS:-/workspace/ur_cbf_ws}"
auto_build="${AUTO_BUILD:-missing}"

build_workspace() {
  cd "${workspace}"
  colcon build --symlink-install
}

if [[ -d "${workspace}/src" ]]; then
  case "${auto_build}" in
    always)
      build_workspace
      ;;
    missing)
      if [[ ! -f "${workspace}/install/setup.bash" ]]; then
        build_workspace
      fi
      ;;
    never)
      ;;
    *)
      echo "AUTO_BUILD deve ser always, missing ou never." >&2
      exit 2
      ;;
  esac

  if [[ -f "${workspace}/install/setup.bash" ]]; then
    source "${workspace}/install/setup.bash"
  fi
fi

# Os executaveis Python instalados pelo colcon preservam o interpretador que
# executa /usr/bin/colcon, normalmente /usr/bin/python3. O UAIbot, por sua vez,
# fica isolado no ambiente virtual da imagem. Anexe seu site-packages depois dos
# prefixos ROS para disponibiliza-lo aos nos sem sombrear os pacotes do sistema.
venv_python="${VIRTUAL_ENV:-/opt/ur_cbf_venv}/bin/python3"
if [[ ! -x "${venv_python}" ]]; then
  echo "Ambiente virtual Python nao encontrado: ${venv_python}" >&2
  exit 1
fi
venv_site_packages="$(
  "${venv_python}" -c 'import sysconfig; print(sysconfig.get_path("purelib"))'
)"
if [[ ! -d "${venv_site_packages}" ]]; then
  echo "Diretorio site-packages nao encontrado: ${venv_site_packages}" >&2
  exit 1
fi
case ":${PYTHONPATH:-}:" in
  *":${venv_site_packages}:"*)
    ;;
  *)
    export PYTHONPATH="${PYTHONPATH:+${PYTHONPATH}:}${venv_site_packages}"
    ;;
esac

cd /workspace
exec "$@"
