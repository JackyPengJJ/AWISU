"""Dimensioned installation envelope for the 18 V candidate, separate from X2 replay."""

import json
import math
import xml.etree.ElementTree as ET
import numpy as np
from .flight import ROOT


def config():
    return json.loads((ROOT / "configs/candidate18.json").read_text())


def installation():
    """BOM mass placements with parallel-axis inertia, not a measured CAD model."""
    from .final_design import calculate

    d = calculate()
    parts = []
    a = 0.6 / (2 * math.sqrt(2))
    rotors = [(a, a), (-a, a), (-a, -a), (a, -a)]

    def box(name, mass, pos, size, basis):
        m = mass / 1000
        dx, dy, dz = size
        parts.append(
            {
                "name": name,
                "mass_kg": m,
                "position_m": list(pos),
                "dimensions_m": list(size),
                "local_inertia": np.diag(
                    [
                        m * (dy * dy + dz * dz) / 12,
                        m * (dx * dx + dz * dz) / 12,
                        m * (dx * dx + dy * dy) / 12,
                    ]
                ).tolist(),
                "basis": basis,
            }
        )

    for name, mass, basis in d["base"]["mass_items"]:
        if any(s in name for s in ["MN3508", "ESC", "P14"]):
            size = (
                [0.04, 0.04, 0.03]
                if "MN3508" in name
                else [0.08, 0.025, 0.012]
                if "ESC" in name
                else [0.3556, 0.03, 0.003]
            )
            for i, (x, y) in enumerate(rotors):
                box(
                    f"{name} / {i}",
                    mass / 4,
                    [x, y, 0.05],
                    size,
                    basis + "; rotor-station placement allocation",
                )
        elif "Carbon frame" in name:
            for i, (x, y) in enumerate(rotors):
                box(
                    f"Frame arm allocation {i}",
                    mass * 0.8 / 4,
                    [x / 2, y / 2, 0],
                    [0.22, 0.05, 0.015],
                    "Installation mass distribution allocation",
                )
            box(
                "Central frame / landing allocation",
                mass * 0.2,
                [0, 0, -0.02],
                [0.18, 0.12, 0.18],
                basis,
            )
        elif "Propeller rings" in name:
            for i, (x, y) in enumerate(rotors):
                box(
                    f"Guard distribution {i}",
                    mass / 4,
                    [x, y, 0.05],
                    [0.3956, 0.3956, 0.02],
                    basis + "; bounding-box mass distribution",
                )
        elif "BFS-U3" in name:
            for side in [-1, 1]:
                box(
                    f"Camera body {side}",
                    mass / 2,
                    [0, side * 0.365, 0],
                    [0.029, 0.030, 0.029],
                    "FLIR dimensions; optical flange placement allocation",
                )
        elif "lenses" in name:
            for side in [-1, 1]:
                box(
                    f"Lens and installation {side}",
                    mass / 2,
                    [0, side * 0.3981, 0],
                    [0.042, 0.0362, 0.042],
                    "Opto Engineering dimensions; 110 g installed allocation",
                )
        elif "pitch servos" in name:
            for side in [-1, 1]:
                box(
                    f"Pitch support {side}",
                    mass / 2,
                    [0, side * 0.345, -0.02],
                    [0.025, 0.08, 0.04],
                    basis,
                )
        elif "VTC6" in name:
            box(name, mass, [0, 0, -0.09], [0.111, 0.038, 0.069], basis)
        elif "MID-360" in name:
            box(
                name,
                mass,
                [0, 0, 0.12],
                [0.07, 0.07, 0.06],
                basis + "; bounding installation envelope",
            )
        elif "OAK-D" in name:
            box(
                name,
                mass,
                [0.12, 0, 0.02],
                [0.025, 0.10, 0.03],
                basis + "; installation envelope",
            )
        elif "Orin" in name:
            box(name, mass, [0, 0, 0.025], [0.10, 0.08, 0.04], basis)
        else:
            box(
                name,
                mass,
                [0, 0, 0],
                [0.15, 0.10, 0.05],
                basis + "; central allocation, refine at CAD stage",
            )
    mass = sum(p["mass_kg"] for p in parts)
    com = sum(p["mass_kg"] * np.array(p["position_m"]) for p in parts) / mass
    J = np.zeros((3, 3))
    for p in parts:
        r = np.array(p["position_m"]) - com
        J += np.array(p["local_inertia"]) + p["mass_kg"] * (
            np.dot(r, r) * np.eye(3) - np.outer(r, r)
        )
    assert abs(mass - d["mass_g"] / 1000) < 1e-10
    assert np.linalg.eigvalsh(J).min() > 0
    return {
        "mass_kg": mass,
        "centre_of_mass_m": com.tolist(),
        "inertia_kgm2": J.tolist(),
        "parts": parts,
        "status": "dimensioned mass-allocation model; not weighed hardware or manufacturer CAD",
        "camera_body_source": "https://softwareservices.flir.com/BFS-U3-122S6/latest/Model/spec.html",
        "lens_source": "https://www.opto-e.com/en/products/en-12rt-series/RT-V1228-MPY2",
    }


def build_model(warehouse=False, open_ends=False):
    import mujoco

    cfg = config()
    mech = installation()
    flight = cfg["flight"]
    a = flight["motor_diagonal_m"] / (2 * math.sqrt(2))
    rotors = [(a, a), (-a, a), (-a, -a), (a, -a)]
    if warehouse:
        from .flight import create_model, load_config

        c = load_config()
        c["visual_assets"] = True
        _, xml = create_model(c)
        root = ET.fromstring(xml)
        world = root.find("worldbody")
        body = world.find("body[@name='uav']")
        for elem in list(body):
            body.remove(elem)
        root.find("sensor").clear()
        root.find("actuator").clear()
    else:
        root = ET.fromstring(
            '<mujoco model="awisu_candidate18"><option timestep=".005" gravity="0 0 -9.80665" integrator="RK4"/><visual><global offwidth="4096" offheight="3000"/><headlight ambient=".5 .5 .5" diffuse=".6 .6 .6"/></visual><worldbody><light pos="10 -4 9"/><geom name="floor" type="plane" size="100 10 .1" rgba=".85 .87 .89 1"/><body name="uav"/></worldbody><actuator/><sensor/></mujoco>'
        )
        world = root.find("worldbody")
        body = world.find("body")
    body.set("pos", "10 0 2")
    ET.SubElement(body, "freejoint")
    vals = np.asarray(mech["inertia_kgm2"])
    ET.SubElement(
        body,
        "inertial",
        mass=str(mech["mass_kg"]),
        pos=" ".join(map(str, mech["centre_of_mass_m"])),
        fullinertia=" ".join(
            map(
                str,
                [
                    vals[0, 0],
                    vals[1, 1],
                    vals[2, 2],
                    vals[0, 1],
                    vals[0, 2],
                    vals[1, 2],
                ],
            )
        ),
    )
    ET.SubElement(
        body,
        "geom",
        name="guard",
        type="cylinder",
        size=".5 .25",
        rgba="0 0 0 0",
        group="3",
    )

    def geom(name, kind, pos, size, rgba, **extra):
        return ET.SubElement(
            body,
            "geom",
            name=name,
            type=kind,
            pos=" ".join(map(str, pos)),
            size=" ".join(map(str, size)),
            rgba=rgba,
            contype="0",
            conaffinity="0",
            mass="0",
            group="2",
            **extra,
        )

    geom("central_plate", "box", [0, 0, 0], [0.09, 0.06, 0.012], ".11 .13 .15 1")
    geom("compute_stack", "box", [0, 0, 0.035], [0.05, 0.04, 0.02], ".25 .27 .29 1")
    geom("battery_6s2p", "box", [0, 0, -0.09], [0.0555, 0.019, 0.0345], ".15 .21 .3 1")
    geom("lidar_envelope", "cylinder", [0, 0, 0.12], [0.035, 0.03], ".3 .3 .31 1")
    geom(
        "stereo_envelope",
        "box",
        [0.12, 0, 0.02],
        [0.0125, 0.05, 0.015],
        ".15 .15 .16 1",
    )
    for i, (x, y) in enumerate(rotors):
        ET.SubElement(
            body,
            "geom",
            name=f"arm_{i}",
            type="capsule",
            fromto=f"0 0 0 {x} {y} .04",
            size=".012",
            rgba=".18 .19 .2 1",
            mass="0",
            contype="0",
            conaffinity="0",
            group="2",
        )
        geom(f"motor_{i}", "cylinder", [x, y, 0.05], [0.02, 0.015], ".25 .27 .29 1")
        geom(
            f"prop_{i}",
            "box",
            [x, y, 0.075],
            [0.1778, 0.012, 0.0015],
            ".07 .08 .09 1",
            euler=f"0 0 {i * 0.9}",
        )
        # Thin ring as 32 line segments; centre and radius match the design.
        for j in range(32):
            u, v = 2 * math.pi * j / 32, 2 * math.pi * (j + 1) / 32
            ET.SubElement(
                body,
                "geom",
                type="capsule",
                fromto=f"{x + 0.1978 * math.cos(u)} {y + 0.1978 * math.sin(u)} .06 {x + 0.1978 * math.cos(v)} {y + 0.1978 * math.sin(v)} .06",
                size=".003",
                rgba=".3 .32 .34 1",
                mass="0",
                contype="0",
                conaffinity="0",
                group="2",
            )
        ET.SubElement(
            body, "site", name=f"r{i}", pos=f"{x} {y} .05", size=".005", rgba="0 0 0 0"
        )
        ET.SubElement(
            root.find("actuator"),
            "general",
            site=f"r{i}",
            gear=f"0 0 1 0 0 {[1, -1, 1, -1][i] * 0.018}",
            ctrllimited="true",
            ctrlrange="0 16",
        )
    fy = math.degrees(2 * math.atan(3000 * 0.00345 / (2 * 12)))
    for name, side in [("left", 1), ("right", -1)]:
        ET.SubElement(
            body,
            "geom",
            type="capsule",
            fromto=f"0 0 -.015 0 {side * 0.345} -.015",
            size=".009",
            rgba=".3 .32 .34 1",
            mass="0",
            contype="0",
            conaffinity="0",
            group="2",
        )
        geom(
            f"{name}_camera_body",
            "box",
            [0, side * 0.365, 0],
            [0.0145, 0.015, 0.0145],
            ".17 .18 .19 1",
        )
        geom(
            f"{name}_lens",
            "cylinder",
            [0, side * 0.3981, 0],
            [0.021, 0.0181],
            ".08 .08 .09 1",
            quat=".70710678 .70710678 0 0",
        )
        geom(
            f"{name}_front_glass",
            "cylinder",
            [0, side * 0.4164, 0],
            [0.017, 0.0003],
            ".1 .2 .3 1",
            quat=".70710678 .70710678 0 0",
        )
        ET.SubElement(
            body,
            "camera",
            name=name,
            pos=f"0 {side * 0.4} 0",
            xyaxes=f"{side} 0 0 0 0 1",
            fovy=str(fy),
        )
    ET.SubElement(
        body,
        "camera",
        name="front_left",
        pos=".14 .04 .025",
        xyaxes="0 -1 0 .258819 0 .965926",
        fovy="50",
    )
    ET.SubElement(body, "site", name="imu", rgba="0 0 0 0")
    ET.SubElement(root.find("sensor"), "accelerometer", site="imu")
    ET.SubElement(root.find("sensor"), "gyro", site="imu")
    # Fixed head frames support explicit optical-pose snapshots. No servo
    # dynamics or settling performance is implied by changing these transforms.
    for name, side in [("left", 1), ("right", -1)]:
        mount = ET.SubElement(
            body, "body", name="head_" + name, pos=f"0 {side * 0.4} 0"
        )
        for item in list(body):
            if item is mount:
                continue
            if item.get("name", "").startswith(name + "_") or (
                item.tag == "camera" and item.get("name") == name
            ):
                pos = np.fromstring(item.get("pos", "0 0 0"), sep=" ")
                pos[1] -= side * 0.4
                item.set("pos", " ".join(map(str, pos)))
                body.remove(item)
                mount.append(item)
    global_visual = root.find("visual/global")
    global_visual.set("offwidth", "4096")
    global_visual.set("offheight", "3000")
    if warehouse:
        ET.SubElement(
            world,
            "geom",
            name="candidate_ceiling",
            type="box",
            pos="40 0 8.04",
            size="40 1.25 .04",
            rgba=".7 .72 .74 1",
        )
        for x in [] if open_ends else [-0.05, 80.05]:
            ET.SubElement(
                world,
                "geom",
                name="candidate_end_" + str(x),
                type="box",
                pos=f"{x} 0 4",
                size=".05 1.25 4",
                rgba=".75 .77 .79 1",
            )
    xml = ET.tostring(root).decode()
    model = mujoco.MjModel.from_xml_string(xml)
    return model, xml, mech
