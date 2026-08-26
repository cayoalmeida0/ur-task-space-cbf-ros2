"""Geometria de autocolisao UR3e/RG2 pertencente ao projeto."""

from dataclasses import dataclass
from dataclasses import replace
from typing import Any

import numpy as np


UAIBOT_FACTORY_GEOMETRY_SOURCE = (
    "UAIbot/UAIbotPy@1acb5ed637738aca4ea05945e6c065c3757bc13d:"
    "uaibot/robot/_create_ur_ur3e.py"
)
PROJECT_GEOMETRY_SOURCE = (
    "ur-task-space-cbf-ros2@0.6.3:"
    "ur_cbf_control/uaibot_collision_model.py#UR3E_RG2_PROJECT_PRIMITIVES"
)


class UaibotCollisionModelError(RuntimeError):
    """Indica divergencia entre a geometria esperada e o modelo em execucao."""


@dataclass(frozen=True)
class UaibotPrimitiveSpec:
    """Primitiva, pose no frame DH posterior a junta e dimensoes em metros."""

    link_index: int
    object_index: int
    primitive_type: str
    htm: tuple[tuple[float, float, float, float], ...]
    dimensions: tuple[float, ...]

    @property
    def identifier(self) -> str:
        return f"link_{self.link_index}_obj_{self.object_index}"


def _matrix(*rows: tuple[float, float, float, float]):
    return rows


UR3E_UAIBOT_PRIMITIVES = (
    UaibotPrimitiveSpec(0, 0, "Cylinder", _matrix(
        (1, 0, 0, 0), (0, 0, 1, -0.0469), (0, -1, 0, 0), (0, 0, 0, 1)
    ), (0.067, 0.21)),
    UaibotPrimitiveSpec(1, 0, "Cylinder", _matrix(
        (0, -1, 0, 0.2454), (1, 0, 0, 0), (0, 0, 1, 0.118), (0, 0, 0, 1)
    ), (0.052, 0.13)),
    UaibotPrimitiveSpec(1, 1, "Cylinder", _matrix(
        (0, 0, -1, 0.1254), (1, 0, 0, 0), (0, -1, 0, 0.12), (0, 0, 0, 1)
    ), (0.05, 0.2)),
    UaibotPrimitiveSpec(1, 2, "Cylinder", _matrix(
        (0, -1, 0, 0.0004), (1, 0, 0, 0), (0, 0, 1, 0.118), (0, 0, 0, 1)
    ), (0.05, 0.12)),
    UaibotPrimitiveSpec(2, 0, "Ball", _matrix(
        (0, 0, -1, 0.1886), (1, 0, 0, 0), (0, -1, 0, 0.05), (0, 0, 0, 1)
    ), (0.05,)),
    UaibotPrimitiveSpec(2, 1, "Cylinder", _matrix(
        (0, 0, -1, 0.1086), (1, 0, 0, 0), (0, -1, 0, 0.05), (0, 0, 0, 1)
    ), (0.04, 0.2)),
    UaibotPrimitiveSpec(2, 2, "Cylinder", _matrix(
        (0, -1, 0, -0.0014), (1, 0, 0, 0), (0, 0, 1, 0.045), (0, 0, 0, 1)
    ), (0.035, 0.09)),
    UaibotPrimitiveSpec(3, 0, "Cylinder", _matrix(
        (-1, 0, 0, 0), (0, 0, 1, 0.0039), (0, 1, 0, 0.0014), (0, 0, 0, 1)
    ), (0.035, 0.09)),
    UaibotPrimitiveSpec(3, 1, "Cylinder", _matrix(
        (-1, 0, 0, 0), (0, -1, 0, -0.0011), (0, 0, 1, 0.0414), (0, 0, 0, 1)
    ), (0.035, 0.045)),
    UaibotPrimitiveSpec(4, 0, "Cylinder", _matrix(
        (0, 1, 0, 0.0011), (0, 0, -1, 0.034), (-1, 0, 0, 0), (0, 0, 0, 1)
    ), (0.035, 0.025)),
    UaibotPrimitiveSpec(4, 1, "Cylinder", _matrix(
        (0, 1, 0, 0.0011), (1, 0, 0, 0.004), (0, 0, -1, -0.0025), (0, 0, 0, 1)
    ), (0.038, 0.098)),
    UaibotPrimitiveSpec(5, 0, "Cylinder", _matrix(
        (0, 1, 0, 0.0011), (1, 0, 0, 0.004), (0, 0, -1, -0.0231), (0, 0, 0, 1)
    ), (0.038, 0.046)),
    UaibotPrimitiveSpec(5, 1, "Cylinder", _matrix(
        (0, 1, 0, 0.0011), (0, 0, -1, -0.021), (-1, 0, 0, -0.0201), (0, 0, 0, 1)
    ), (0.01, 0.028)),
    UaibotPrimitiveSpec(5, 2, "Ball", _matrix(
        (0, 1, 0, 0.0011), (0, 0, -1, 0.004), (-1, 0, 0, 0.0279), (0, 0, 0, 1)
    ), (0.05,)),
    UaibotPrimitiveSpec(5, 3, "Box", _matrix(
        (0, 1, 0, 0.0011), (0, 0, -1, -0.006), (-1, 0, 0, 0.1079), (0, 0, 0, 1)
    ), (0.09, 0.07, 0.06)),
    UaibotPrimitiveSpec(5, 4, "Box", _matrix(
        (0.7071, 0.7071, 0, -0.0389), (0, 0, -1, -0.001),
        (-0.7071, 0.7071, 0, 0.1529), (0, 0, 0, 1)
    ), (0.075, 0.04, 0.035)),
    UaibotPrimitiveSpec(5, 5, "Box", _matrix(
        (-0.7071, 0.7071, 0, 0.0411), (0, 0, -1, -0.001),
        (-0.7071, -0.7071, 0, 0.1529), (0, 0, 0, 1)
    ), (0.075, 0.04, 0.035)),
    UaibotPrimitiveSpec(5, 6, "Cylinder", _matrix(
        (0, 1, 0, 0.0511), (1, 0, 0, -0.001), (0, 0, -1, 0.1979), (0, 0, 0, 1)
    ), (0.021, 0.04)),
    UaibotPrimitiveSpec(5, 7, "Cylinder", _matrix(
        (0, 1, 0, -0.0489), (1, 0, 0, -0.001), (0, 0, -1, 0.1979), (0, 0, 0, 1)
    ), (0.021, 0.04)),
)


def _translation(z: float):
    return _matrix(
        (1, 0, 0, 0),
        (0, 1, 0, 0),
        (0, 0, 1, z),
        (0, 0, 0, 1),
    )


def _adjust_primitive(
    spec: UaibotPrimitiveSpec,
    *,
    translation: tuple[float, float, float] | None = None,
    dimensions: tuple[float, ...] | None = None,
):
    """Copia uma primitiva alterando pose local e/ou dimensoes."""

    rows = [list(row) for row in spec.htm]
    if translation is not None:
        for axis, value in enumerate(translation):
            rows[axis][3] = float(value)
    return replace(
        spec,
        htm=tuple(tuple(row) for row in rows),
        dimensions=spec.dimensions if dimensions is None else dimensions,
    )


# Cada primitiva do braco e uma copia independente. Os ajustes experimentais
# devem ser feitos somente nesta tabela, preservando a tabela anterior como
# contrato imutavel da fabrica UAIbot fixada.
UR3E_RG2_PROJECT_PRIMITIVES = (
    replace(UR3E_UAIBOT_PRIMITIVES[0]),
    replace(UR3E_UAIBOT_PRIMITIVES[1]),
    replace(UR3E_UAIBOT_PRIMITIVES[2]),
    replace(UR3E_UAIBOT_PRIMITIVES[3]),
    # O corpo principal do antebraco usa o elbow_offset=27 mm. A esfera c21
    # fica no centro da junta e adota o elbow_radius=60 mm oficial.
    _adjust_primitive(
        UR3E_UAIBOT_PRIMITIVES[4],
        translation=(0.2132, 0.0, 0.027),
        dimensions=(0.060,),
    ),
    _adjust_primitive(
        UR3E_UAIBOT_PRIMITIVES[5],
        translation=(0.1086, 0.0, 0.027),
    ),
    # c23 cobre continuamente do elbow_offset ao centro de wrist_1_link.
    _adjust_primitive(
        UR3E_UAIBOT_PRIMITIVES[6],
        translation=(0.0, 0.0, 0.079025),
        dimensions=(0.040, 0.10405),
    ),
    # c31 e c41 sao centralizados nos corpos das juntas de punho; c32 cobre
    # todo o vao de 85,35 mm entre wrist_1_link e wrist_2_link. c31/c32 usam
    # o wrist_radius=45 mm oficial. c23/c41 preservam uma pequena folga entre
    # os elos nao adjacentes 2 e 4, evitando autocolisao estrutural falsa.
    _adjust_primitive(
        UR3E_UAIBOT_PRIMITIVES[7],
        translation=(0.0, 0.0, 0.0),
        dimensions=(0.045, 0.090),
    ),
    _adjust_primitive(
        UR3E_UAIBOT_PRIMITIVES[8],
        translation=(0.0, 0.0, 0.042675),
        dimensions=(0.045, 0.08535),
    ),
    _adjust_primitive(
        UR3E_UAIBOT_PRIMITIVES[9],
        translation=(0.0, 0.0, 0.0),
        dimensions=(0.038, 0.090),
    ),
    replace(UR3E_UAIBOT_PRIMITIVES[10]),
    replace(UR3E_UAIBOT_PRIMITIVES[11]),
    replace(UR3E_UAIBOT_PRIMITIVES[12]),
    # A capsula RG2 substitui os seis objetos da garra generica UAIbot.
    UaibotPrimitiveSpec(5, 2, "Cylinder", _translation(0.110), (0.090, 0.110)),
    UaibotPrimitiveSpec(5, 3, "Ball", _translation(0.055), (0.090,)),
    UaibotPrimitiveSpec(5, 4, "Ball", _translation(0.165), (0.090,)),
)


def _runtime_dimensions(primitive: Any, primitive_type: str) -> tuple[float, ...]:
    if primitive_type == "Ball":
        return (float(primitive.radius),)
    if primitive_type == "Cylinder":
        return (float(primitive.radius), float(primitive.height))
    if primitive_type == "Box":
        return (
            float(primitive.width),
            float(primitive.depth),
            float(primitive.height),
        )
    raise UaibotCollisionModelError(
        f"Tipo de primitiva UAIbot nao suportado: {primitive_type}."
    )


def _validate_collision_model(
    robot: Any,
    *,
    specs: tuple[UaibotPrimitiveSpec, ...],
    expected_counts: tuple[int, ...],
    label: str,
) -> None:
    """Valida estritamente um conjunto ordenado de objetos por elo."""

    if len(robot.links) != len(expected_counts):
        raise UaibotCollisionModelError(
            "Modelo de colisao UAIbot nao possui os seis elos esperados."
        )
    actual_counts = tuple(len(link.col_objects) for link in robot.links)
    if actual_counts != expected_counts:
        raise UaibotCollisionModelError(
            f"Contagem de primitivas {label} diverge: "
            f"obtida={actual_counts}; esperada={expected_counts}."
        )

    for spec in specs:
        try:
            primitive, attached_htm = robot.links[spec.link_index].col_objects[
                spec.object_index
            ]
        except Exception as error:
            raise UaibotCollisionModelError(
                f"Primitiva ausente: {spec.identifier}."
            ) from error
        actual_type = type(primitive).__name__
        if actual_type != spec.primitive_type:
            raise UaibotCollisionModelError(
                f"Tipo divergente em {spec.identifier}: "
                f"obtido={actual_type}; esperado={spec.primitive_type}."
            )
        actual_htm = np.asarray(attached_htm, dtype=float)
        expected_htm = np.asarray(spec.htm, dtype=float)
        if actual_htm.shape != (4, 4) or not np.allclose(
            actual_htm,
            expected_htm,
            rtol=0.0,
            atol=1e-10,
        ):
            raise UaibotCollisionModelError(
                f"Transformacao divergente em {spec.identifier}."
            )
        actual_dimensions = _runtime_dimensions(primitive, spec.primitive_type)
        if not np.allclose(
            actual_dimensions,
            spec.dimensions,
            rtol=0.0,
            atol=1e-12,
        ):
            raise UaibotCollisionModelError(
                f"Dimensoes divergentes em {spec.identifier}: "
                f"obtidas={actual_dimensions}; esperadas={spec.dimensions}."
            )


def validate_uaibot_ur3e_factory_model(robot: Any) -> None:
    """Recusa uma fabrica UAIbot diferente da dependencia fixada pelo projeto."""

    _validate_collision_model(
        robot,
        specs=UR3E_UAIBOT_PRIMITIVES,
        expected_counts=(1, 3, 3, 2, 2, 8),
        label="da fabrica UAIbot",
    )


def validate_ur3e_rg2_project_collision_model(robot: Any) -> None:
    """Confirma o modelo corrigido que deve coincidir com o Xacro visual."""

    _validate_collision_model(
        robot,
        specs=UR3E_RG2_PROJECT_PRIMITIVES,
        expected_counts=(1, 3, 3, 2, 2, 5),
        label="UR3e/RG2 do projeto",
    )


def _create_primitive(uaibot_module: Any, spec: UaibotPrimitiveSpec) -> Any:
    """Instancia uma primitiva simples usando a API publica do UAIbot."""

    try:
        primitive_class = getattr(uaibot_module, spec.primitive_type)
    except AttributeError as error:
        raise UaibotCollisionModelError(
            f"UAIbot nao expoe a primitiva {spec.primitive_type}."
        ) from error

    htm = np.asarray(spec.htm, dtype=float)
    common = {
        "htm": htm,
        "name": f"ur3e_rg2_{spec.identifier}",
        "color": (
            "#ff7300"
            if spec.link_index == 5 and spec.object_index >= 2
            else "#009fe3"
        ),
        "opacity": 0.3,
    }
    if spec.primitive_type == "Ball":
        return primitive_class(radius=spec.dimensions[0], **common)
    if spec.primitive_type == "Cylinder":
        return primitive_class(
            radius=spec.dimensions[0],
            height=spec.dimensions[1],
            **common,
        )
    if spec.primitive_type == "Box":
        return primitive_class(
            width=spec.dimensions[0],
            depth=spec.dimensions[1],
            height=spec.dimensions[2],
            **common,
        )
    raise UaibotCollisionModelError(
        f"Tipo de primitiva do projeto nao suportado: {spec.primitive_type}."
    )


def configure_ur3e_rg2_project_collision_model(
    robot: Any,
    uaibot_module: Any,
) -> None:
    """Substitui os objetos da fabrica pelo modelo versionado UR3e/RG2.

    A fabrica e validada antes da troca para que uma atualizacao da dependencia
    nao seja aceita silenciosamente. A escrita em ``_col_objects`` e necessaria
    porque o UAIbot 1.2.7 oferece anexacao publica, mas nao remocao publica.
    """

    validate_uaibot_ur3e_factory_model(robot)
    for link in robot.links:
        storage = getattr(link, "_col_objects", None)
        if not isinstance(storage, list) or not hasattr(link, "attach_col_object"):
            raise UaibotCollisionModelError(
                "Link UAIbot nao permite substituir objetos de colisao."
            )
        storage.clear()

    for spec in UR3E_RG2_PROJECT_PRIMITIVES:
        primitive = _create_primitive(uaibot_module, spec)
        attached_htm = np.asarray(spec.htm, dtype=float)
        robot.links[spec.link_index].attach_col_object(primitive, attached_htm)

    validate_ur3e_rg2_project_collision_model(robot)
