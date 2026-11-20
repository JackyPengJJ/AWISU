"""Licensed real-robot visuals and metric warehouse staging, separate from dynamics."""

import json
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]


def decorate(xml):
    root = ET.fromstring(xml)
    ET.SubElement(root, "compiler", angle="radian")
    asset = ET.SubElement(root, "asset")
    world = root.find("worldbody")
    visual = root.find("visual")
    visual.find("global").set("offwidth", "4096")
    visual.find("global").set("offheight", "2160")
    ET.SubElement(
        visual,
        "headlight",
        ambient=".35 .35 .35",
        diffuse=".5 .5 .5",
        specular=".2 .2 .2",
    )
    ET.SubElement(visual, "quality", shadowsize="2048", offsamples="4")
    ET.SubElement(visual, "map", znear=".0005", zfar="10")
    ET.SubElement(
        asset,
        "texture",
        name="sky",
        type="skybox",
        builtin="gradient",
        rgb1=".025 .045 .075",
        rgb2=".16 .24 .32",
        width="512",
        height="3072",
    )
    ET.SubElement(
        asset,
        "texture",
        name="grid",
        type="2d",
        builtin="checker",
        rgb1=".045 .085 .13",
        rgb2=".09 .15 .21",
        mark="edge",
        markrgb=".17 .25 .32",
        width="1024",
        height="1024",
    )
    ET.SubElement(
        asset,
        "material",
        name="floor_mat",
        texture="grid",
        texrepeat="1 1",
        texuniform="true",
        reflectance=".12",
        shininess=".15",
    )
    floor = world.find("geom[@name='floor']")
    floor.set("material", "floor_mat")
    floor.set("rgba", "1 1 1 1")
    floor.set("size", "150 150 .1")
    for elem in list(world.findall("light")):
        world.remove(elem)
    ET.SubElement(
        world,
        "light",
        pos="20 -10 14",
        dir=".15 .25 -1",
        directional="true",
        diffuse=".8 .78 .73",
        specular=".25 .25 .25",
        castshadow="true",
    )
    ET.SubElement(
        world,
        "light",
        pos="30 10 12",
        dir="-.2 -.3 -1",
        directional="true",
        diffuse=".4 .48 .6",
        castshadow="false",
    )
    for name, rgba, spec in [
        ("steel_blue", ".12 .26 .38 1", ".35"),
        ("safety_orange", ".95 .31 .055 1", ".2"),
        ("zinc", ".44 .49 .52 1", ".5"),
        ("rubber", ".035 .04 .05 1", ".1"),
        ("wood", ".48 .32 .17 1", ".05"),
    ]:
        ET.SubElement(
            asset, "material", name=name, rgba=rgba, specular=spec, shininess=".35"
        )
    drone = ROOT / "assets/vendor/menagerie/skydio_x2/assets"
    ET.SubElement(
        asset,
        "texture",
        name="x2_tex",
        type="2d",
        file=str(drone / "X2_lowpoly_texture_SpinningProps_1024.png"),
    )
    ET.SubElement(
        asset,
        "material",
        name="x2_mat",
        texture="x2_tex",
        specular=".35",
        shininess=".4",
    )
    ET.SubElement(
        asset,
        "mesh",
        name="x2_mesh",
        file=str(ROOT / "assets/prepared/skydio_x2/x2_inspection.obj"),
        scale=".01 .01 .01",
    )
    body = world.find("body[@name='uav']")
    body.find("geom[@name='guard']").set("rgba", "0 0 0 0")
    body.find("geom[@name='guard']").set("group", "3")
    ET.SubElement(
        body,
        "geom",
        name="x2_visual",
        type="mesh",
        mesh="x2_mesh",
        material="x2_mat",
        quat="0 0 1 1",
        contype="0",
        conaffinity="0",
        mass="0",
        group="2",
    )
    for site in body.findall("site"):
        site.set("rgba", "0 0 0 0")
    for i, (x, y, z) in enumerate(
        [
            (-0.14, -0.18, 0.05),
            (-0.14, 0.18, 0.05),
            (0.14, 0.18, 0.08),
            (0.14, -0.18, 0.08),
        ]
    ):
        ET.SubElement(
            body,
            "geom",
            type="ellipsoid",
            size=".13 .014 .003",
            pos=f"{x} {y} {z + 0.005}",
            euler=f"0 0 {i * 0.7}",
            material="rubber",
            contype="0",
            conaffinity="0",
            mass="0",
        )
    # Added guards belong to the research configuration, not the stock manufacturer mesh.
    for x, y, z in [
        (-0.14, -0.18, 0.05),
        (-0.14, 0.18, 0.05),
        (0.14, 0.18, 0.08),
        (0.14, -0.18, 0.08),
    ]:
        for k in range(20):
            a, b = 2 * np.pi * np.array([k, k + 1]) / 20
            points = [
                x + 0.145 * np.cos(a),
                y + 0.145 * np.sin(a),
                z,
                x + 0.145 * np.cos(b),
                y + 0.145 * np.sin(b),
                z,
            ]
            ET.SubElement(
                body,
                "geom",
                type="capsule",
                size=".0025",
                fromto=" ".join(map(str, points)),
                material="rubber",
                contype="0",
                conaffinity="0",
                mass="0",
                group="2",
            )
    for camera in body.findall("camera"):
        camera.set("fovy", "24.65")
    for name, y in [("front_left", 0.04), ("front_right", -0.04)]:
        ET.SubElement(
            body,
            "camera",
            name=name,
            pos=f".10 {y} .04",
            xyaxes="0 -1 0 0 0 1",
            fovy="50",
        )
    # Research payload geometry; not a claim that stock X2 supports this payload/autopilot.
    ET.SubElement(
        body,
        "geom",
        type="box",
        size=".06 .045 .025",
        pos="0 0 -.035",
        material="rubber",
        contype="0",
        conaffinity="0",
        mass="0",
    )
    for side in [-1, 1]:
        ET.SubElement(
            body,
            "geom",
            type="box",
            size=".025 .015 .02",
            pos=f"0 {side * 0.075} -.025",
            material="zinc",
            contype="0",
            conaffinity="0",
            mass="0",
        )
        ET.SubElement(
            body,
            "geom",
            type="cylinder",
            size=".013 .008",
            pos=f"0 {side * 0.094} -.025",
            quat=".7071068 .7071068 0 0",
            material="rubber",
            contype="0",
            conaffinity="0",
            mass="0",
        )
    # Load original CC0 scanned cargo, reduced to 1800 faces with transferred UVs.
    parts = json.loads(
        (ROOT / "assets/prepared/cardboard_box_01/parts.json").read_text()
    )
    boxparts = []
    bounds = np.array([p["bounds"] for p in parts])
    lower = bounds[:, 0].min(axis=0) * [2, 2, 3]
    upper = bounds[:, 1].max(axis=0) * [2, 2, 3]
    mesh_offset = np.r_[-(lower[:2] + upper[:2]) / 2, -lower[2]]
    half_xy = (upper[:2] - lower[:2]) / 2
    carton_height = upper[2] - lower[2]
    cargo_layout = {}
    for i, part in enumerate(parts):
        ET.SubElement(
            asset,
            "mesh",
            name=f"cargo_mesh{i}",
            file=str(ROOT / part["mesh"]),
            scale="2 2 3",
        )
        ET.SubElement(
            asset,
            "texture",
            name=f"cargo_tex{i}",
            type="2d",
            file=str(ROOT / part["texture"]),
        )
        ET.SubElement(
            asset,
            "material",
            name=f"cargo_mat{i}",
            texture=f"cargo_tex{i}",
            specular=".03",
            shininess=".02",
        )
        boxparts.append((f"cargo_mesh{i}", f"cargo_mat{i}"))
    # Keep existing collision envelopes; replace only their flat visual appearance.
    rng = np.random.default_rng(512)
    for geom in list(world.findall("geom")):
        if geom.get("name") == "floor":
            continue
        kind = geom.get("rgba", "")
        x, y, z = map(float, geom.get("pos", "0 0 0").split())
        if kind == ".65 .48 .27 1":
            side = int(np.sign(y))
            layer = round((z - 0.7) / 1.6)
            slot = round((x - 1) / 2)
            item = 200 * int(side == 1) + layer * 40 + slot
            name = f"cargo_{item:04d}"
            shelf_top = layer * 1.6 + 0.035
            runner_height = 0.08
            deck_thickness = 0.035
            pallet_top = shelf_top + runner_height + deck_thickness
            yaw = np.pi / 2 + rng.uniform(-0.035, 0.035)
            R = Rotation.from_euler("z", yaw).as_matrix()
            centre = np.array([x, side * 1.67, pallet_top])
            cargo = ET.SubElement(
                world,
                "body",
                name=name,
                pos=" ".join(map(str, centre)),
                euler=f"0 0 {yaw}",
            )
            for mesh, mat in boxparts:
                ET.SubElement(
                    cargo,
                    "geom",
                    name=f"{name}_visual",
                    type="mesh",
                    mesh=mesh,
                    material=mat,
                    pos=" ".join(map(str, mesh_offset)),
                    contype="0",
                    conaffinity="0",
                    mass="0",
                    group="2",
                )
            # Collision box follows the visible carton bounds, rather than the old floating block.
            extent = np.abs(R[:2, :2]) @ half_xy
            geom.set("name", f"{name}_collision")
            geom.set("group", "3")
            geom.set("pos", f"{x} {side * 1.67} {pallet_top + carton_height / 2}")
            geom.set("size", f"{extent[0]} {extent[1]} {carton_height / 2}")
            for j, dx in enumerate([-0.40, 0, 0.40]):
                ET.SubElement(
                    world,
                    "geom",
                    name=f"{name}_runner{j}",
                    type="box",
                    size=f".07 .37 {runner_height / 2}",
                    pos=f"{x + dx} {side * 1.67} {shelf_top + runner_height / 2}",
                    material="wood",
                )
            for j, dx in enumerate([-0.44, -0.22, 0, 0.22, 0.44]):
                ET.SubElement(
                    world,
                    "geom",
                    name=f"{name}_deck{j}",
                    type="box",
                    size=f".09 .37 {deck_thickness / 2}",
                    pos=f"{x + dx} {side * 1.67} {pallet_top - deck_thickness / 2}",
                    material="wood",
                )
            cargo_layout[(side, layer, slot)] = {
                "body": cargo,
                "position": centre,
                "rotation": R,
                "pallet_top": pallet_top,
                "shelf_top": shelf_top,
                "height": carton_height,
                "half_xy": half_xy.tolist(),
            }
        elif kind == ".3 .4 .5 1":
            geom.set("name", f"rack_deck_{int(np.sign(y))}_{round(z / 1.6)}")
            if round(z / 1.6) == 5:
                geom.set("pos", f"{x} {y} 7.965")
            geom.set("material", "zinc")
            geom.attrib.pop("rgba", None)
        elif kind == ".4 .4 .4 1":
            geom.set("material", "steel_blue")
            geom.attrib.pop("rgba", None)
    for side in [-1, 1]:
        for level in range(6):
            for y in [side * 1.295, side * 2.05]:
                ET.SubElement(
                    world,
                    "geom",
                    type="box",
                    size="40 .045 .07",
                    pos=f"40 {y} {min(level * 1.6, 7.93)}",
                    material="safety_orange",
                )
        for x in np.arange(0, 80.1, 4):
            ET.SubElement(
                world,
                "geom",
                type="box",
                size=".09 .05 .02",
                pos=f"{x} {side * 1.30} .025",
                material="zinc",
                contype="0",
                conaffinity="0",
            )
            ET.SubElement(
                world,
                "geom",
                type="box",
                size=".035 .035 4",
                pos=f"{x} {side * 2.05} 4",
                material="steel_blue",
            )
            for z in [0.8, 2.4, 4.0, 5.6, 7.2]:
                ET.SubElement(
                    world,
                    "geom",
                    type="capsule",
                    size=".012",
                    fromto=f"{x} {side * 1.30} {z - 0.65} {x} {side * 2.05} {z + 0.65}",
                    material="zinc",
                    contype="0",
                    conaffinity="0",
                )
        # Aisle boundary strips preserve the requested checker floor appearance.
        ET.SubElement(
            world,
            "geom",
            type="box",
            size="40 .025 .002",
            pos=f"40 {side * 1.19} .003",
            rgba="1 .65 .05 1",
            contype="0",
            conaffinity="0",
        )
    from .barcodes import label_image
    from PIL import Image

    generated = ROOT / "assets/generated/labels"
    generated.mkdir(parents=True, exist_ok=True)
    truth = []
    for side_idx, side in enumerate([-1, 1]):
        for layer in range(5):
            for slot in range(40):
                code = f"W{side_idx * 200 + layer * 40 + slot:04d}"
                path = generated / f"{code}.png"
                if not path.exists():
                    Image.fromarray(label_image(code)).convert("RGB").save(path)
                ET.SubElement(
                    asset,
                    "texture",
                    name=f"label_tex_{code}",
                    type="2d",
                    file=str(path),
                )
                ET.SubElement(
                    asset,
                    "material",
                    name=f"label_mat_{code}",
                    texture=f"label_tex_{code}",
                    specular="0",
                    texuniform="false",
                )
                layout = cargo_layout[(side, layer, slot)]
                local = np.array(
                    [
                        -side * (half_xy[0] + 0.0005),
                        0,
                        0.8 + 1.6 * layer - layout["pallet_top"],
                    ]
                )
                orientation = np.array([[0, 0, -side], [-side, 0, 0], [0, 1, 0.0]])
                q = Rotation.from_matrix(orientation).as_quat()
                q = np.r_[q[3], q[:3]]
                ET.SubElement(
                    layout["body"],
                    "geom",
                    name=f"label_{code}",
                    type="box",
                    size=".05 .022 .0003",
                    pos=" ".join(map(str, local)),
                    quat=" ".join(map(str, q)),
                    material=f"label_mat_{code}",
                    contype="0",
                    conaffinity="0",
                    mass="0",
                    group="2",
                )
                pos = layout["position"] + layout["rotation"] @ local
                truth.append(
                    {
                        "id": code,
                        "side": side,
                        "layer": layer,
                        "slot": slot,
                        "position": pos.tolist(),
                    }
                )
    (generated.parent / "inventory_truth.json").write_text(json.dumps(truth, indent=2))
    support = []
    for (side, layer, slot), layout in cargo_layout.items():
        support.append(
            {
                "side": side,
                "layer": layer,
                "slot": slot,
                "shelf_top": layout["shelf_top"],
                "runner_bottom": layout["shelf_top"],
                "runner_top": layout["shelf_top"] + 0.08,
                "deck_bottom": layout["pallet_top"] - 0.035,
                "deck_top": layout["pallet_top"],
                "carton_bottom": layout["pallet_top"],
                "carton_top": layout["pallet_top"] + layout["height"],
            }
        )
    (generated.parent / "support_layout.json").write_text(json.dumps(support, indent=2))
    # True-scale scanned steel shelving as staging furniture beside launch, not distorted into 8 m racks.
    shelfparts = json.loads(
        (ROOT / "assets/prepared/steel_frame_shelves_03/parts.json").read_text()
    )
    staging = ET.SubElement(world, "body", pos="-3 3.5 0")
    for i, part in enumerate(shelfparts):
        ET.SubElement(
            asset, "mesh", name=f"staging_mesh{i}", file=str(ROOT / part["mesh"])
        )
        ET.SubElement(
            asset,
            "texture",
            name=f"staging_tex{i}",
            type="2d",
            file=str(ROOT / part["texture"]),
        )
        ET.SubElement(
            asset,
            "material",
            name=f"staging_mat{i}",
            texture=f"staging_tex{i}",
            specular=".15",
        )
        ET.SubElement(
            staging,
            "geom",
            type="mesh",
            mesh=f"staging_mesh{i}",
            material=f"staging_mat{i}",
            contype="0",
            conaffinity="0",
        )
    # Landing target rings and centre H.
    ET.SubElement(
        world,
        "geom",
        type="cylinder",
        size=".65 .003",
        pos="-2 0 .004",
        rgba=".12 .18 .23 1",
        contype="0",
        conaffinity="0",
    )
    for y in [-0.16, 0.16]:
        ET.SubElement(
            world,
            "geom",
            type="box",
            size=".25 .028 .002",
            pos=f"-2 {y} .009",
            rgba="1 .8 .15 1",
            contype="0",
            conaffinity="0",
        )
    ET.SubElement(
        world,
        "geom",
        type="box",
        size=".028 .16 .002",
        pos="-2 0 .009",
        rgba="1 .8 .15 1",
        contype="0",
        conaffinity="0",
    )
    return ET.tostring(root, encoding="unicode")
