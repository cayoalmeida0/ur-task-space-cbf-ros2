#!/usr/bin/env bash
set -euo pipefail

CONTROLLER="forward_velocity_controller"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERRO: ros2 nao esta disponivel neste ambiente."
  exit 1
fi

if ! ros2 node list | grep -qx "/gz_ros_control"; then
  echo "ERRO: /gz_ros_control nao foi encontrado. Inicie a simulacao primeiro."
  exit 1
fi

if ! ros2 control list_controllers |
    grep -Eq "^$CONTROLLER[[:space:]].*[[:space:]]active[[:space:]]*$"; then
  echo "ERRO: $CONTROLLER nao esta ativo."
  exit 1
fi

run_pulse() {
  local joint="$1"
  local velocity="$2"
  local duration="$3"

  ros2 run ur_cbf_control joint_velocity_pulse_test --ros-args \
    -p execute_test:=true \
    -p target_joint:="$joint" \
    -p pulse_velocity:="$velocity" \
    -p max_abs_velocity:=0.35 \
    -p pulse_duration:="$duration" \
    -p settle_duration:=0.5 \
    -p zero_hold_duration:=0.5 \
    -p minimum_motion_ratio:=0.6 \
    -p maximum_motion_ratio:=1.4 \
    -p max_other_joint_displacement:=0.01
}

echo "Etapa 1/6: rotacao ampla da base."
run_pulse shoulder_pan_joint 0.20 3.0
echo "Etapa 2/6: retorno da base."
run_pulse shoulder_pan_joint -0.20 3.0

echo "Etapa 3/6: flexao ampla do cotovelo."
run_pulse elbow_joint 0.20 3.0
echo "Etapa 4/6: retorno do cotovelo."
run_pulse elbow_joint -0.20 3.0

echo "Etapa 5/6: rotacao rapida do primeiro punho."
run_pulse wrist_1_joint 0.30 2.0
echo "Etapa 6/6: retorno do primeiro punho."
run_pulse wrist_1_joint -0.30 2.0

echo "ENSAIO VISUAL CONCLUIDO: seis pulsos aprovados e comando final nulo."
