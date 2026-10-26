"""Explicit, offline asset conversion. Requires the optional harness-assets extra.

python examples/assets/prepare_piper_mujoco.py --robotwin-assets /path/to/RoboTwin/assets
No download or conversion is performed by cyr start or package import.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from copy import deepcopy
import yaml

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]


def numbers(values):
    return " ".join(f"{float(v):.10g}" for v in values)


def converted_meshes(path):
    import trimesh
    scene = trimesh.load(path, force="scene")
    result = []
    for node in scene.graph.nodes_geometry:
        transform, geometry = scene.graph[node]
        mesh = scene.geometry[geometry].copy()
        mesh.apply_transform(transform)
        result.append(mesh)
    if not result:
        raise ValueError(f"No meshes: {path}")
    return result


def build(output, robotwin_assets):
    import trimesh
    output.mkdir(parents=True, exist_ok=True)
    meshes = output / "meshes"
    meshes.mkdir(exist_ok=True)
    urdf = ROOT / "assets/sim/piper/piper_single.urdf"
    bowl_dir = robotwin_assets / "objects/002_bowl"
    source_files = [urdf, bowl_dir / "visual/base3.glb", bowl_dir / "collision/base3.glb", bowl_dir / "model_data3.json"]
    metadata = json.loads(source_files[-1].read_text())
    original_urdf = robotwin_assets / "embodiments/piper/piper.urdf"
    robot_config = robotwin_assets / "embodiments/piper/config.yml"
    source_files.extend([original_urdf, robot_config])
    config = yaml.safe_load(robot_config.read_text())
    scale = np.array(metadata["scale"])
    xml = ET.Element("mujoco", model="piper_three_bowls")
    ET.SubElement(xml, "compiler", angle="radian", eulerseq="XYZ", meshdir="meshes", autolimits="true")
    ET.SubElement(xml, "option", timestep="0.002", integrator="implicitfast", cone="elliptic", iterations="80")
    visual = ET.SubElement(xml, "visual")
    ET.SubElement(visual, "global", offwidth="640", offheight="480")
    asset = ET.SubElement(xml, "asset")
    world = ET.SubElement(xml, "worldbody")
    ET.SubElement(world, "light", pos="0 0 2", dir="0 0 -1", diffuse="0.8 0.8 0.8")
    ET.SubElement(world, "geom", name="table", type="box", pos="0.35 0 -0.025", size="0.65 0.55 0.025",
                  rgba="0.65 0.65 0.65 1", contype="2", conaffinity="3", friction="0.8 0.01 0.001")
    # Rebase RoboTwin's fixed head camera into this scene's base_link=world frame.
    head = next(c for c in config["static_camera_list"] if c["name"] == "head_camera")
    base_pose = np.array(config["robot_pose"][0])
    world_from_base = Rotation.from_quat(base_pose[[4, 5, 6, 3]]).as_matrix()
    eye = world_from_base.T @ (np.array(head["position"]) - base_pose[:3])
    forward = world_from_base.T @ np.array(head["forward"])
    left = world_from_base.T @ np.array(head["left"])
    forward /= np.linalg.norm(forward); left /= np.linalg.norm(left)
    up = np.cross(forward, left)
    ET.SubElement(world, "camera", name="head", pos=numbers(eye), xyaxes=numbers(np.r_[-left, up]), fovy="37")
    tree = ET.parse(urdf).getroot()
    original = ET.parse(original_urdf).getroot()
    # Preserve the two fixed joints and their visual origins independently.
    for tag, names in (("link", ("camera_mount", "camera")), ("joint", ("link6_to_camera_mount", "camera_joint"))):
        for name in names:
            element = deepcopy(original.find(f"{tag}[@name='{name}']"))
            if tag == "link":
                for mesh in element.findall("visual/geometry/mesh"):
                    mesh.set("filename", str(original_urdf.parent / mesh.get("filename")))
            tree.append(element)
    links = {link.attrib["name"]: link for link in tree.findall("link")}
    children = {}
    for joint in tree.findall("joint"):
        children.setdefault(joint.find("parent").attrib["link"], []).append(joint)
    actuators = ET.SubElement(xml, "actuator")

    def add_link(name, parent, joint=None):
        attributes = {"name": name}
        if joint is not None:
            origin = joint.find("origin")
            if origin is not None:
                attributes.update(pos=origin.get("xyz", "0 0 0"), euler=origin.get("rpy", "0 0 0"))
        body = ET.SubElement(parent, "body", **attributes)
        link = links[name]
        inertia = link.find("inertial")
        if inertia is not None:
            raw = inertia.find("inertia").attrib
            matrix = np.array([[float(raw[k]) for k in row] for row in (("ixx", "ixy", "ixz"), ("ixy", "iyy", "iyz"), ("ixz", "iyz", "izz"))])
            origin = inertia.find("origin")
            rotation = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")).as_matrix()
            matrix = rotation @ matrix @ rotation.T
            ET.SubElement(body, "inertial", pos=origin.get("xyz", "0 0 0"), mass=inertia.find("mass").get("value"),
                          fullinertia=numbers([matrix[0, 0], matrix[1, 1], matrix[2, 2], matrix[0, 1], matrix[0, 2], matrix[1, 2]]))
        if joint is not None and joint.get("type") != "fixed":
            limit = joint.find("limit")
            jname = joint.get("name")
            finger = joint.get("type") == "prismatic"
            limits = f'{limit.get("lower")} {limit.get("upper")}'
            ET.SubElement(body, "joint", name=jname, type="slide" if finger else "hinge", axis=joint.find("axis").get("xyz"),
                          range=limits, damping="1" if finger else "0.2", armature="0.001" if finger else "0.01")
            ET.SubElement(actuators, "position", name=jname, joint=jname, kp="500" if finger else "300",
                          kv="5" if finger else "20", ctrlrange=limits, forcerange="-10 10" if finger else "-60 60")
        vis = link.find("visual")
        if vis is not None:
            source = urdf.parent / vis.find("geometry/mesh").get("filename")
            source_files.append(source)
            mesh = trimesh.util.concatenate(converted_meshes(source))
            if name == "camera":
                mesh = mesh.simplify_quadric_decimation(face_count=20000)
            filename = f"{name}.stl"
            mesh.export(meshes / filename)
            ET.SubElement(asset, "mesh", name=name, file=filename)
            origin = vis.find("origin")
            camera_visual = name in ("camera_mount", "camera")
            ET.SubElement(body, "geom", name=f"robot_{name}", type="mesh", mesh=name, rgba="0.2 0.23 0.26 1",
                          pos=origin.get("xyz", "0 0 0"), euler=origin.get("rpy", "0 0 0"),
                          contype="0" if camera_visual else "1", conaffinity="0" if camera_visual else "2",
                          mass="0", friction="1.2 0.01 0.001", condim="4")
        if name == "flange":
            ET.SubElement(body, "site", name="flange", size="0.002", rgba="0 0 0 0")
        if name == "tool":
            ET.SubElement(body, "site", name="tool", size="0.002", rgba="0 0 0 0")
        if name == "camera":
            # RoboTwin attaches the SAPIEN camera directly to URDF link `camera`.
            # SAPIEN FLU -> MuJoCo OpenGL RUB: right=-Y, up=+Z, backward=-X.
            ET.SubElement(body, "camera", name="wrist", pos="0 0 0", xyaxes="0 -1 0 0 0 1", fovy="37")
        for child in children.get(name, []):
            add_link(child.find("child").get("link"), body, child)
    add_link("base_link", world)
    bowl_visual = trimesh.util.concatenate(converted_meshes(bowl_dir / "visual/base3.glb"))
    bowl_visual.apply_scale(scale)
    bowl_visual.export(meshes / "bowl_visual.stl")
    ET.SubElement(asset, "mesh", name="bowl_visual", file="bowl_visual.stl")
    parts = converted_meshes(bowl_dir / "collision/base3.glb")
    # Each GLB node is a convex decomposition piece; never collapse into one convex hull.
    for i, part in enumerate(parts):
        part.apply_scale(scale)
        part.convex_hull.export(meshes / f"bowl_collision_{i}.stl")
        ET.SubElement(asset, "mesh", name=f"bowl_collision_{i}", file=f"bowl_collision_{i}.stl")
    if len(parts) < 2:
        raise ValueError("Bowl collision asset lacks multiple convex pieces; refusing a filled cavity")
    for i, (x, y) in enumerate(((.42, -.12), (.46, .18), (.25, .23))):
        body = ET.SubElement(world, "body", name=f"bowl_{i}", pos=f"{x} {y} 0.08", quat="0.5 0.5 0.5 0.5")
        ET.SubElement(body, "freejoint", name=f"bowl_{i}_free")
        # Approximate bowl inertia, explicitly separate from visual mesh volume.
        ET.SubElement(body, "inertial", pos="0 0.025 0", mass="0.08", diaginertia="0.00015 0.00025 0.00015")
        ET.SubElement(body, "geom", name=f"bowl_{i}_visual", type="mesh", mesh="bowl_visual", contype="0", conaffinity="0",
                      group="1", rgba=("0.85 0.3 0.2 1", "0.2 0.6 0.85 1", "0.3 0.75 0.35 1")[i])
        for j in range(len(parts)):
            ET.SubElement(body, "geom", name=f"bowl_{i}_collision_{j}", type="mesh", mesh=f"bowl_collision_{j}",
                          contype="2", conaffinity="3", group="3", rgba="0 0 0 0", friction="1 0.01 0.001", condim="4")
    ET.indent(xml)
    ET.ElementTree(xml).write(output / "scene.xml", encoding="unicode")
    manifest = {"source": "TianxingChen/RoboTwin2.0: objects/002_bowl model_id=3; local RoboTwin asset copy",
                "collision_pieces": len(parts), "bowl_scale": scale.tolist(), "bowl_mass_kg": .08,
                "initial_bowl_positions_m": [[.42, -.12, .08], [.46, .18, .08], [.25, .23, .08]],
                "robot_collision": "one convex hull per visual link, approximate; robot self-collision disabled",
                "sources": {str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p.relative_to(robotwin_assets)):
                            hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files}}
    manifest["cameras"] = {"wrist_parent": "camera", "wrist_chain": ["link6_to_camera_mount", "camera_joint"],
        "source_axes": "SAPIEN FLU", "renderer_axes": "OpenGL RUB", "observation_axes": "OpenCV RDF",
        "fovy_degrees": 37, "source_camera_profile": "RoboTwin D435 320x240",
        "head_pose": "RoboTwin static_camera_list transformed by inverse robot_pose",
        "camera_visuals": "original origins; camera mesh simplified to 20000 faces; massless, non-colliding"}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Prepared {output / 'scene.xml'}; {len(parts)} convex bowl pieces")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robotwin-assets", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "assets/sim/piper_mujoco")
    args = parser.parse_args()
    build(args.out.resolve(), args.robotwin_assets.resolve())
