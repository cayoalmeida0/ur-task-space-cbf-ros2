"""Adaptador entre o controlador ROS 2 e a cinemática do UAIbot."""

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np


class KinematicsError(RuntimeError):
    """Indica incompatibilidade de modelo ou resultado cinemático inválido."""


@dataclass(frozen=True)
class KinematicState:
    """Posição do efetuador e Jacobiano translacional na ordem do modelo."""

    position: tuple[float, float, float]
    translational_jacobian: np.ndarray


def homogeneous_transform_from_xyz_rpy(
    xyz: Sequence[float],
    rpy: Sequence[float],
) -> np.ndarray:
    """Cria uma HTM usando a convencao URDF Rz(yaw) Ry(pitch) Rx(roll)."""

    translation = np.asarray(xyz, dtype=float).reshape(-1)
    angles = np.asarray(rpy, dtype=float).reshape(-1)
    if translation.size != 3 or not np.all(np.isfinite(translation)):
        raise KinematicsError("eef_offset_xyz deve conter tres valores finitos.")
    if angles.size != 3 or not np.all(np.isfinite(angles)):
        raise KinematicsError("eef_offset_rpy deve conter tres valores finitos.")

    roll, pitch, yaw = angles
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rotation = np.array(
        (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        ),
        dtype=float,
    )
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


class UaibotKinematics:
    """Encapsula a API do UAIbot e explicita a ordem das juntas do modelo."""

    def __init__(
        self,
        robot: Any,
        model_joint_names: Sequence[str],
        eef_offset_xyz: Sequence[float],
        eef_offset_rpy: Sequence[float],
        mode: str = "auto",
    ) -> None:
        self.robot = robot
        self.model_joint_names = tuple(str(name) for name in model_joint_names)
        if not self.model_joint_names or len(set(self.model_joint_names)) != len(
            self.model_joint_names
        ):
            raise KinematicsError("Ordem de juntas do modelo vazia ou duplicada.")
        if len(self.model_joint_names) != len(self.robot.links):
            raise KinematicsError(
                "Quantidade de juntas configuradas difere dos elos do modelo UAIbot."
            )
        if mode not in {"auto", "python", "c++"}:
            raise KinematicsError("Modo UAIbot deve ser auto, python ou c++.")
        self.mode = mode
        htm_n_eef = homogeneous_transform_from_xyz_rpy(
            eef_offset_xyz,
            eef_offset_rpy,
        )
        self.robot.set_htm_to_eef(htm_n_eef)

    @classmethod
    def create(
        cls,
        *,
        ur_type: str,
        model_joint_names: Sequence[str],
        eef_offset_xyz: Sequence[float],
        eef_offset_rpy: Sequence[float],
        mode: str = "auto",
    ) -> "UaibotKinematics":
        """Cria o modelo solicitado e rejeita explicitamente modelos sem adaptador."""

        try:
            import uaibot as ub
        except ImportError as error:
            raise KinematicsError("UAIbot nao esta instalado no ambiente.") from error

        factories = {
            "ur3e": ub.Robot.create_ur_ur3e,
        }
        if ur_type not in factories:
            supported = ", ".join(sorted(factories))
            raise KinematicsError(
                f"Modelo {ur_type!r} ainda nao possui adaptador UAIbot; "
                f"suportados: {supported}."
            )
        robot = factories[ur_type](
            name=f"{ur_type}_control_model",
            eef_frame_visible=False,
        )
        return cls(
            robot,
            model_joint_names,
            eef_offset_xyz,
            eef_offset_rpy,
            mode,
        )

    def evaluate(self, model_positions: Sequence[float]) -> KinematicState:
        """Calcula posição e Jacobiano translacional para uma configuração."""

        positions = np.asarray(model_positions, dtype=float).reshape(-1)
        if positions.size != len(self.model_joint_names):
            raise KinematicsError(
                "Dimensao da configuracao difere da ordem de juntas do modelo."
            )
        if not np.all(np.isfinite(positions)):
            raise KinematicsError("Configuracao contém NaN ou infinito.")
        try:
            jacobian, htm_eef = self.robot.jac_geo(
                q=positions,
                axis="eef",
                mode=self.mode,
            )
        except Exception as error:
            raise KinematicsError(f"Falha na cinemática UAIbot: {error}") from error

        jacobian_array = np.asarray(jacobian, dtype=float)
        htm_array = np.asarray(htm_eef, dtype=float)
        expected_shape = (6, len(self.model_joint_names))
        if jacobian_array.shape != expected_shape:
            raise KinematicsError(
                f"Jacobiano UAIbot tem forma {jacobian_array.shape}; "
                f"esperada {expected_shape}."
            )
        if htm_array.shape != (4, 4):
            raise KinematicsError("Transformacao do efetuador nao e 4x4.")
        values = np.concatenate((jacobian_array.reshape(-1), htm_array.reshape(-1)))
        if not np.all(np.isfinite(values)):
            raise KinematicsError("Cinematica UAIbot contem NaN ou infinito.")
        position = tuple(float(value) for value in htm_array[:3, 3])
        return KinematicState(
            position=position,
            translational_jacobian=jacobian_array[:3, :].copy(),
        )
