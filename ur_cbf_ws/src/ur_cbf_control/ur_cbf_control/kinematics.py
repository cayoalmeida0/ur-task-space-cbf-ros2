"""Adaptador entre o controlador ROS 2 e a cinemática do UAIbot."""

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np


class KinematicsError(RuntimeError):
    """Indica incompatibilidade de modelo ou resultado cinemático inválido."""


@dataclass(frozen=True)
class KinematicModelCorrection:
    """Correcao versionada aplicada ao modelo cinemático de uma dependencia."""

    parameter: str
    upstream_value: float
    corrected_value: float
    reference: str

    def as_record(self) -> dict[str, object]:
        """Retorna uma representacao serializavel para o registro experimental."""

        return {
            "parameter": self.parameter,
            "upstream_value": self.upstream_value,
            "corrected_value": self.corrected_value,
            "reference": self.reference,
        }


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


_UR3E_OFFICIAL_WRIST_2_D_M = 0.08535
_UR3E_UAIBOT_1_2_7_WRIST_2_D_M = 0.10535
_UR3E_DH_TOLERANCE_M = 1e-10
_UR3E_DH_REFERENCE = (
    "UniversalRobots/Universal_Robots_ROS2_Description@jazzy:"
    "config/ur3e/default_kinematics.yaml"
)


def _correct_ur3e_uaibot_dh(robot: Any) -> tuple[KinematicModelCorrection, ...]:
    """Alinha o d5 do UAIbot 1.2.7 aos parametros oficiais do UR3e.

    O UAIbot 1.2.7 soma 20 mm ao quinto parametro ``d``. Essa diferenca nao e
    uma transformacao fixa de ferramenta: ela altera posicao e Jacobiano de forma
    dependente da configuracao. Por isso a correcao e aplicada no proprio elo DH,
    com validacao estrita do valor conhecido da dependencia.
    """

    links = getattr(robot, "links", None)
    if links is None or len(links) != 6:
        raise KinematicsError(
            "Modelo UAIbot UR3e deve expor exatamente seis elos DH."
        )
    wrist_2_link = links[4]
    try:
        upstream_value = float(wrist_2_link.d)
    except (AttributeError, TypeError, ValueError) as error:
        raise KinematicsError(
            "Quinto elo UAIbot UR3e nao expoe um parametro DH d valido."
        ) from error

    if np.isclose(
        upstream_value,
        _UR3E_OFFICIAL_WRIST_2_D_M,
        rtol=0.0,
        atol=_UR3E_DH_TOLERANCE_M,
    ):
        return ()
    if not np.isclose(
        upstream_value,
        _UR3E_UAIBOT_1_2_7_WRIST_2_D_M,
        rtol=0.0,
        atol=_UR3E_DH_TOLERANCE_M,
    ):
        raise KinematicsError(
            "Parametro DH d5 inesperado no UAIbot UR3e: "
            f"{upstream_value:.12g} m; esperados "
            f"{_UR3E_UAIBOT_1_2_7_WRIST_2_D_M:.5f} m (UAIbot 1.2.7) ou "
            f"{_UR3E_OFFICIAL_WRIST_2_D_M:.5f} m (oficial)."
        )
    if not hasattr(wrist_2_link, "_d"):
        raise KinematicsError(
            "Quinto elo UAIbot UR3e nao permite a correcao versionada de d5."
        )

    wrist_2_link._d = _UR3E_OFFICIAL_WRIST_2_D_M
    corrected_value = float(wrist_2_link.d)
    if not np.isclose(
        corrected_value,
        _UR3E_OFFICIAL_WRIST_2_D_M,
        rtol=0.0,
        atol=_UR3E_DH_TOLERANCE_M,
    ):
        raise KinematicsError("Falha ao aplicar a correcao DH do UR3e no UAIbot.")
    return (
        KinematicModelCorrection(
            parameter="wrist_2_d_m",
            upstream_value=upstream_value,
            corrected_value=corrected_value,
            reference=_UR3E_DH_REFERENCE,
        ),
    )


class UaibotKinematics:
    """Encapsula a API do UAIbot e explicita a ordem das juntas do modelo."""

    def __init__(
        self,
        robot: Any,
        model_joint_names: Sequence[str],
        eef_offset_xyz: Sequence[float],
        eef_offset_rpy: Sequence[float],
        mode: str = "auto",
        model_name: str = "custom",
        model_corrections: Sequence[KinematicModelCorrection] = (),
        requested_mode: str | None = None,
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
        self.requested_mode = mode if requested_mode is None else requested_mode
        self.model_name = str(model_name)
        self.model_corrections = tuple(model_corrections)
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
        if mode not in {"auto", "python", "c++"}:
            raise KinematicsError("Modo UAIbot deve ser auto, python ou c++.")
        if mode == "c++":
            raise KinematicsError(
                "Modo UAIbot c++ nao e suportado pelo adaptador UR3e corrigido; "
                "use auto ou python."
            )
        robot = factories[ur_type](
            name=f"{ur_type}_control_model",
            eef_frame_visible=False,
        )
        corrections = _correct_ur3e_uaibot_dh(robot)
        return cls(
            robot,
            model_joint_names,
            eef_offset_xyz,
            eef_offset_rpy,
            "python",
            model_name="ur3e_official_ros2_description",
            model_corrections=corrections,
            requested_mode=mode,
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
