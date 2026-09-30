"""Geometria de autocolisao UR3e/RG2 pertencente ao projeto."""

from dataclasses import dataclass
from dataclasses import replace
import math
from typing import Any

import numpy as np


UAIBOT_FACTORY_GEOMETRY_SOURCE = (
    "UAIbot/UAIbotPy@1acb5ed637738aca4ea05945e6c065c3757bc13d:"
    "uaibot/robot/_create_ur_ur3e.py"
)
PROJECT_GEOMETRY_SOURCE = (
    "ur-task-space-cbf-ros2@0.6.39:"
    "ur_cbf_control/uaibot_collision_model.py#UR3E_RG2_PROJECT_PRIMITIVES"
)

# A RG2 aceita uma abertura nominal de 0 a 110 mm. Os quatro objetos laterais
# (duas caixas das falanges e dois cilindros das pontas) são deslocados
# simetricamente a partir da geometria UAIbot em 80 mm, que é a abertura usada
# como referência pelo cenário.
RG2_MIN_WIDTH_M = 0.0
RG2_MAX_WIDTH_M = 0.110
RG2_REFERENCE_WIDTH_M = 0.080
RG2_MOVING_OBJECT_INDICES = (4, 5, 6, 7)


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


def _with_translation_component(
    spec: UaibotPrimitiveSpec,
    axis: int,
    value: float,
):
    """Copia uma especificacao alterando uma componente da translacao DH."""

    if axis not in (0, 1, 2):
        raise ValueError("O eixo de translacao deve ser 0, 1 ou 2.")
    rows = [list(row) for row in spec.htm]
    rows[axis][3] = float(value)
    return replace(spec, htm=tuple(tuple(row) for row in rows))


# As primitivas do braco partem da fabrica UAIbot fixada. Os ajustes abaixo sao feitos
# nas coordenadas DH; em c31/c32, z_DH controla -y_URDF e y_DH controla z_URDF
# por causa de Rx(pi/2). Em c41/c42, y_DH controla -z_URDF por causa de
# Rx(-pi/2). A RG2 reutiliza os oito objetos originais do elo 5 do UAIbot, sem
# substituir a garra por uma capsula conservadora unica.
UR3E_RG2_PROJECT_PRIMITIVES = (
    replace(UR3E_UAIBOT_PRIMITIVES[0]),
    _with_translation_component(UR3E_UAIBOT_PRIMITIVES[1], 2, 0.115),
    *(replace(spec) for spec in UR3E_UAIBOT_PRIMITIVES[2:4]),
    _with_translation_component(
        _with_translation_component(UR3E_UAIBOT_PRIMITIVES[4], 0, 0.2132),
        2,
        0.0415,
    ),
    _with_translation_component(UR3E_UAIBOT_PRIMITIVES[5], 2, 0.027),
    _with_translation_component(
        _with_translation_component(UR3E_UAIBOT_PRIMITIVES[6], 0, 0.0011),
        2,
        0.025,
    ),
    _with_translation_component(
        _with_translation_component(UR3E_UAIBOT_PRIMITIVES[7], 1, -0.02),
        2,
        0.0,
    ),
    replace(
        _with_translation_component(UR3E_UAIBOT_PRIMITIVES[8], 2, 0.0),
        dimensions=(0.035, 0.0945),
    ),
    _with_translation_component(UR3E_UAIBOT_PRIMITIVES[9], 1, 0.027),
    _with_translation_component(
        _with_translation_component(
            _with_translation_component(UR3E_UAIBOT_PRIMITIVES[10], 0, 0.0),
            1,
            0.0,
        ),
        2,
        0.0,
    ),
    _with_translation_component(
        _with_translation_component(
            _with_translation_component(UR3E_UAIBOT_PRIMITIVES[11], 0, 0.0),
            1,
            0.0,
        ),
        2,
        -0.02,
    ),
    _with_translation_component(
        _with_translation_component(
            _with_translation_component(UR3E_UAIBOT_PRIMITIVES[12], 0, 0.0011),
            1,
            -0.026,
        ),
        2,
        -0.03,
    ),
    # Objetos 2..7: geometria original da garra UAIbot, preservada para a RG2.
    *(replace(spec) for spec in UR3E_UAIBOT_PRIMITIVES[13:]),
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
        expected_counts=(1, 3, 3, 2, 2, 8),
        label="UR3e/RG2 do projeto",
    )


def update_ur3e_rg2_gripper_width(
    robot: Any,
    width: float,
    *,
    reference_width: float = RG2_REFERENCE_WIDTH_M,
) -> None:
    """Atualiza as poses laterais da RG2 conforme a abertura comandada.

    A geometria UAIbot é anexada ao frame DH do último elo. Portanto, a
    abertura física da garra não aparece automaticamente no modelo interno.
    As primitivas centrais permanecem fixas e as quatro primitivas laterais
    são recalculadas a partir da pose de referência, movendo cada lado pela
    metade da variação da abertura. A operação mantém tipos e dimensões e é
    compatível com o cálculo de distância do UAIbot e com a CBF da mesa.
    """

    width = float(width)
    reference_width = float(reference_width)
    if not math.isfinite(width) or not (
        RG2_MIN_WIDTH_M <= width <= RG2_MAX_WIDTH_M
    ):
        raise UaibotCollisionModelError(
            "A largura da RG2 deve estar entre 0 e 0.110 m."
        )
    if not math.isfinite(reference_width) or not (
        RG2_MIN_WIDTH_M <= reference_width <= RG2_MAX_WIDTH_M
    ):
        raise UaibotCollisionModelError(
            "A largura de referência da RG2 é inválida."
        )

    try:
        storage = robot.links[5]._col_objects
    except (AttributeError, IndexError, TypeError) as error:
        raise UaibotCollisionModelError(
            "O modelo UR3e/RG2 não expõe o armazenamento do elo final."
        ) from error
    if not isinstance(storage, list) or len(storage) != 8:
        raise UaibotCollisionModelError(
            "O elo final da RG2 deve conter exatamente oito primitivas."
        )

    specs = {
        spec.object_index: spec
        for spec in UR3E_RG2_PROJECT_PRIMITIVES
        if spec.link_index == 5
    }
    width_delta = width - reference_width
    for object_index in RG2_MOVING_OBJECT_INDICES:
        spec = specs[object_index]
        try:
            item = storage[object_index]
            attached = np.asarray(item[1], dtype=float)
        except (AttributeError, IndexError, TypeError, ValueError) as error:
            raise UaibotCollisionModelError(
                f"Transformação ausente em link_5_obj_{object_index}."
            ) from error
        if attached.shape != (4, 4):
            raise UaibotCollisionModelError(
                f"Transformação inválida em link_5_obj_{object_index}."
            )

        updated = attached.copy()
        reference_x = float(spec.htm[0][3])
        # x negativo é o lado esquerdo e x positivo o lado direito no frame
        # final UAIbot. Ao fechar, ambos avançam na direção do centro.
        updated[0, 3] = reference_x + math.copysign(
            1.0,
            reference_x,
        ) * 0.5 * width_delta
        if isinstance(item, tuple):
            storage[object_index] = (item[0], updated)
        elif isinstance(item, list):
            storage[object_index] = [item[0], updated]
        else:
            raise UaibotCollisionModelError(
                f"Entrada inválida em link_5_obj_{object_index}."
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
