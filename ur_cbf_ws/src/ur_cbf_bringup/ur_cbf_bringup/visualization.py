"""Semantica das descricoes visuais usadas por RViz e Gazebo."""


def _boolean_text(value, name):
    normalized = str(value).strip().lower()
    if normalized not in ("true", "false"):
        raise ValueError(
            f"{name} deve ser true ou false; recebido: {normalized!r}."
        )
    return normalized == "true"


def resolve_cbf_visibility(show_volumes, show_volumes_gazebo):
    """Retorna os valores Xacro independentes para RViz e Gazebo."""

    rviz_visible = _boolean_text(show_volumes, "show_cbf_volumes")
    gazebo_enabled = _boolean_text(
        show_volumes_gazebo, "show_cbf_volumes_gazebo"
    )
    return (
        "true" if rviz_visible else "false",
        "true" if rviz_visible and gazebo_enabled else "false",
    )
