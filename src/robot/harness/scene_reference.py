"""Explicit static height priors plus measured robot FK; never live object truth."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def load_height_reference(scene_path=None):
    from robot.controller.PiperMuJoCo_controller import default_scene
    bundled = default_scene().resolve()
    path = Path(scene_path).resolve() if scene_path else bundled
    try:
        table = ET.parse(path).getroot().find("worldbody/geom[@name='table']")
        if table is None or table.get("type") != "box":
            raise ValueError("an axis-aligned world table box is required")
        if any(key in table.attrib for key in ("quat", "euler", "axisangle", "xyaxes", "zaxis")):
            raise ValueError("rotated table reference unsupported")
        position = np.fromstring(table.get("pos", "0 0 0"), sep=" ")
        size = np.fromstring(table.get("size", ""), sep=" ")
        if position.shape != (3,) or size.shape != (3,) or not np.isfinite([position, size]).all() or np.any(size <= 0):
            raise ValueError("invalid table geometry")
        result = {"available": True, "frame": "base", "table_top_z_m": float(position[2] + size[2]),
                  "source": "static scene table box; not a camera/object/contact measurement"}
        if path == bundled:
            result.update(bowl_height_above_table_approx_m=.05,
                          bowl_rim_z_approx_m=result["table_top_z_m"] + .05,
                          upright_grasp_tool_z_reference_m=result["table_top_z_m"] + .035,
                          object_reference="rough bundled bowl geometry and fixed upright-grasp example; assumes upright on table, not current object state")
        return result
    except (OSError, ET.ParseError, ValueError):
        return {"available": False, "source": "static table reference unavailable; do not assume Z=0"}


def height_feedback(reference, fk):
    result = dict(reference)
    if reference.get("available"):
        result.update(tool_z_base_m=float(fk["tool"][2]),
                      tool_height_above_table_m=float(fk["tool"][2] - reference["table_top_z_m"]),
                      robot_height_source="measured joints + Pinocchio FK at current observation",
                      tool_reference="nominal tool origin; not the lowest finger/robot point or guaranteed collision clearance")
    return result
