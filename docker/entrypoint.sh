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

cd /workspace
exec "$@"
