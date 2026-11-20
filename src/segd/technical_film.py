"""Three-act technical film from licensed meshes and retained experiment records.

New views interpolate saved poses; no physics, inference or image decoding runs.
Comic Neue matches the report's panel headings. External films are style references
only. Outputs live separately from historical video evidence.
"""

import argparse
from functools import lru_cache
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image, ImageDraw, ImageFont

os.environ.setdefault("MUJOCO_GL", "osmesa")
from .flight import ROOT

W, H, FPS = 1920, 1080, 30
BG = "#F7F8FA"
INK = "#202B36"
GREY = "#637180"
BLUE = "#0072B2"
GREEN = "#007A5A"
ORANGE = "#C45100"
ACTS = [
    ("hardware", 24),
    ("method", 24),
    ("analysis", 8),
    ("replay", 72),
    ("results", 8),
]


@lru_cache(maxsize=40)
def font(size, bold=False):
    return ImageFont.truetype(
        str(
            ROOT
            / "assets/fonts"
            / ("ComicNeue-Bold.ttf" if bold else "ComicNeue-Regular.ttf")
        ),
        size,
    )


def text(d, xy, s, size=32, fill=INK, bold=False, anchor=None):
    d.text(xy, s, font=font(size, bold), fill=fill, anchor=anchor)


def base(chapter, title, subtitle=""):
    im = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(im)
    text(d, (48, 28), "AWISU", 27, BLUE, True)
    text(d, (1850, 28), chapter, 25, GREY, anchor="ra")
    text(d, (48, 72), title, 49, INK, True)
    if subtitle:
        text(d, (48, 133), subtitle, 28, GREY)
    text(
        d, (48, 1040), "Junjie Peng  ·  A0354063J  ·  EE5110 Technical Report", 23, GREY
    )
    return im


def tile(im, frame, rect, label=None):
    x, y, w, h = map(int, rect)
    img = Image.fromarray(frame) if isinstance(frame, np.ndarray) else frame
    im.paste(img.resize((w, h), Image.Resampling.LANCZOS), (x, y))
    if label:
        d = ImageDraw.Draw(im)
        box = d.textbbox((0, 0), label, font=font(25, True))
        tw = box[2]
        d.rounded_rectangle((x + 12, y + 12, x + tw + 36, y + 52), 8, fill=INK)
        text(d, (x + 24, y + 16), label, 25, "white", True)


@lru_cache(maxsize=12)
def photo(name, size):
    im = Image.open(ROOT / "assets/report_sources" / name).convert("RGB")
    arr = np.asarray(im)
    mask = np.any(arr < 235, axis=2)
    yy, xx = np.where(mask)
    if len(xx):
        im = im.crop(
            (
                max(0, int(xx.min()) - 8),
                max(0, int(yy.min()) - 8),
                min(im.width, int(xx.max()) + 9),
                min(im.height, int(yy.max()) + 9),
            )
        )
    im.thumbnail(size, Image.Resampling.LANCZOS)
    return im


def product_scene():
    import mujoco
    from .flight import create_model

    config = json.loads((ROOT / "results/final/inference/provenance.json").read_text())[
        "config"
    ]
    config["visual_assets"] = True
    unused, xml = create_model(config)
    del unused
    root = ET.fromstring(xml)
    world = root.find("worldbody")
    for elem in list(world):
        if elem.tag != "body" or elem.get("name") != "uav":
            world.remove(elem)
    world.find("body").set("pos", "0 0 .15")
    ET.SubElement(world, "light", pos="0 -1 2", dir="0 .2 -1", diffuse=".7 .7 .7")
    ET.SubElement(
        world, "geom", type="plane", size="100 100 .01", pos="0 0 .02", rgba="1 1 1 1"
    )
    head = root.find("visual/headlight")
    head.set("ambient", ".5 .5 .5")
    head.set("diffuse", ".7 .7 .7")
    m = mujoco.MjModel.from_xml_string(ET.tostring(root).decode())
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    opt = mujoco.MjvOption()
    opt.geomgroup[3] = 0
    opt.sitegroup[:] = 0
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0, 0, 0.18]
    cam.distance = 0.80
    cam.elevation = -30
    return m, d, opt, cam


def hardware_frame(frame, t):
    im = base(
        "01 / SYSTEM DESIGN",
        "AWISU: Warehouse Inventory with a Small UAV"
        if t < 6
        else "A small aircraft. A complete sensing stack.",
        "Proposed installation: 2.902 kg  ·  600 mm motor diagonal  ·  14 in propellers",
    )
    tile(im, frame, (48, 204, 1220, 686))
    d = ImageDraw.Draw(im)
    stage = min(2, int(t / 8))
    items = [
        [
            ("mn3508.jpg", "Propulsion", "MN3508 KV380 / P14"),
            ("geprc_vtc6.jpg", "Energy storage", "6S2P VTC6 / 6 Ah"),
        ],
        [
            ("pixhawk6c.jpg", "Flight control", "Pixhawk 6C / PX4"),
            ("carrier.jpg", "Onboard processing", "Orin Nano / compact carrier"),
        ],
        [
            ("rt_v1228.jpg", "Bilateral barcode views", "12 mm optics / 12.3 MP"),
            ("mn3508.jpg", "Navigation stack", "MID-360 + forward stereo"),
        ],
    ][stage]
    for j, (file, heading, detail) in enumerate(items):
        yy = 205 + j * 355
        d.rounded_rectangle(
            (1300, yy, 1872, yy + 325), 14, fill="white", outline="#D5DDE5", width=2
        )
        if stage == 2 and j == 1:
            text(d, (1586, yy + 100), "LiDAR + IMU", 39, BLUE, True, anchor="mm")
            text(
                d,
                (1586, yy + 155),
                "Visual / LiDAR / inertial fusion",
                28,
                GREY,
                anchor="mm",
            )
        else:
            pic = photo(file, (460, 185))
            im.paste(pic, (1586 - pic.width // 2, yy + 12))
        text(d, (1586, yy + 229), heading, 31, INK, True, anchor="mm")
        text(d, (1586, yy + 278), detail, 27, GREY, anchor="mm")
    text(d, (655, 934), "Simulation airframe / scanning payload", 27, GREY, anchor="mm")
    text(
        d,
        (655, 974),
        "Component choices at right describe the proposed custom platform.",
        25,
        GREY,
        anchor="mm",
    )
    return im


def arrow(d, a, b, color=BLUE, width=5):
    d.line([a, b], fill=color, width=width)
    u = np.array(b, dtype=float) - a
    u /= max(np.linalg.norm(u), 1)
    n = np.array([-u[1], u[0]])
    p = np.array(b)
    d.polygon(
        [tuple(p), tuple(p - 16 * u + 7 * n), tuple(p - 16 * u - 7 * n)], fill=color
    )


def method_frame(t):
    stage = min(2, int(t / 8))
    heads = [
        "Discover free space and rack faces.",
        "Sweep both sides, one level at a time.",
        "Resolve a missed label locally.",
    ]
    subs = [
        "Proposed navigation: LiDAR + IMU + forward images / local geometry",
        "Proposed acquisition: whole-label admission / corroborated reads / spatial tracks",
        "Keep the confirmed side; brake, approach the failed side and resume the same boundary.",
    ]
    im = base("02 / METHOD", heads[stage], subs[stage])
    d = ImageDraw.Draw(im)
    # Top view, 12 m local interval, spatially separate from the full-aisle route.
    x0, x1 = 95, 1170
    y0, y1 = 340, 660
    text(d, (630, 228), "Local aisle view / schematic", 31, INK, True, anchor="mm")
    for yy, side in [(290, "LEFT RACK"), (730, "RIGHT RACK")]:
        text(d, (94, yy - 42), side, 24, GREY)
        for k in range(12):
            x = x0 + k * 89
            d.rounded_rectangle(
                (x, yy, x + 70, yy + 72), 5, fill="#DFCEB8", outline="#AD9273", width=2
            )
            d.rectangle((x + 20, yy + 22, x + 50, yy + 43), fill="white")
            for z in range(22, 50, 4):
                d.line((x + z, yy + 25, x + z, yy + 40), fill=INK, width=2)
    d.line((x0, 535, x1, 535), fill="#CCD5DD", width=3)
    u = (t % 8) / 8
    cx = int(x0 + 90 + u * 850)
    cy = 535
    if stage == 2:
        cx = 640
        cy = 535 + int(55 * np.sin(np.pi * u) ** 2)
        d.line(
            [(490, 535), (640, 535), (640, 590), (640, 535), (800, 535)],
            fill=ORANGE,
            width=4,
        )
        d.rounded_rectangle((630, 730, 700, 802), 5, outline=ORANGE, width=5)
    if stage == 0:
        span = int(80 + u * 700)
        d.polygon(
            [(cx, cy), (min(x1, cx + span), 370), (min(x1, cx + span), 700)],
            fill="#DFEAF2",
        )
        for xx in range(cx, min(x1, cx + span), 42):
            d.ellipse((xx, 350, xx + 5, 355), fill=BLUE)
            d.ellipse((xx, 711, xx + 5, 716), fill=BLUE)
    else:
        d.polygon([(cx, cy), (cx - 100, 365), (cx + 100, 365)], fill="#DFEEE8")
        d.polygon([(cx, cy), (cx - 100, 726), (cx + 100, 726)], fill="#E0EAF4")
        for yy in (290, 730):
            for k in range(
                min(12, int((cx - x0) / 89) + (1 if stage == 2 and yy == 290 else 0))
            ):
                d.rounded_rectangle(
                    (x0 + k * 89, yy, x0 + k * 89 + 70, yy + 72),
                    5,
                    outline=GREEN,
                    width=4,
                )
    # Stylised plan symbol; explicitly schematic, not an installation drawing.
    for dx, dy in [(-22, -22), (-22, 22), (22, -22), (22, 22)]:
        d.line((cx, cy, cx + dx, cy + dy), fill=INK, width=7)
        d.ellipse(
            (cx + dx - 12, cy + dy - 12, cx + dx + 12, cy + dy + 12),
            outline=INK,
            width=3,
        )
    arrow(d, (cx + 35, cy), (cx + 92, cy), INK, 4)
    d.line((1260, 204, 1260, 950), fill="#D7DFE6", width=2)
    if stage == 0:
        lines = [
            ("Observe", "MID-360 / stereo / IMU"),
            ("Estimate", "Time-aligned rack-relative state"),
            ("Admit motion", "Known-free stopping corridor"),
        ]
    elif stage == 1:
        lines = [
            ("Associate", "Side + level + spatial instance"),
            ("Confirm", "Consistent whole-label reads"),
            ("Complete", "No unchecked boundary / pending item"),
        ]
    else:
        lines = [
            ("Preserve", "Opposite-side confirmations"),
            ("Recover", "Up to 0.10 m lateral approach"),
            ("Resume", "Same unfinished coverage boundary"),
        ]
    for j, (head, detail) in enumerate(lines):
        yy = 280 + j * 205
        text(d, (1320, yy), head, 35, BLUE, True)
        text(d, (1320, yy + 58), detail, 25, GREY)
    # Route projected on the rack plane; no map reconstruction is claimed.
    text(d, (95, 864), "Five-level coverage", 27, INK, True)
    for k in range(5):
        yy = 976 - k * 20
        aa, bb = (440, 1130) if k % 2 == 0 else (1130, 440)
        d.line((aa, yy, bb, yy), fill=BLUE, width=3)
        if k < 4:
            d.line((bb, yy, bb, yy - 20), fill=ORANGE, width=3)
    text(d, (1320, 940), "Proposed workflow / animated schematic", 24, GREY)
    return im


def analysis_frame(t, design):
    im = base(
        "03 / DESIGN ANALYSIS",
        "Tie image quality to flight time and energy.",
        "Whole-label geometry, acquisition phase and recovery actions share one budget.",
    )
    d = ImageDraw.Draw(im)
    safe = design["followup"]["coverage"]["minimum_safe_width_m"]
    required = 0.85 * 0.604
    # Large readable spatial ruler rather than a dense paper screenshot.
    text(d, (540, 286), "Complete-label travel", 39, INK, True, anchor="mm")
    x0, scale = 130, 1500
    d.rounded_rectangle(
        (x0, 376, x0 + int(safe * scale), 466),
        8,
        fill="#DEEEE6",
        outline=GREEN,
        width=3,
    )
    progress = min(1, t / 3)
    d.rectangle(
        (x0 + 12, 390, x0 + 12 + int(required * scale * progress), 452),
        fill="#D4E7F5",
        outline=BLUE,
        width=2,
    )
    text(
        d,
        (540, 530),
        f"{safe:.3f} m available / {required:.3f} m required",
        32,
        GREEN,
        True,
        anchor="mm",
    )
    text(d, (540, 602), "0.85 m/s × 0.604 s = 0.513 m", 35, BLUE, True, anchor="mm")
    text(
        d,
        (540, 672),
        "250 × 40 mm worked label / 3.04 pixels per module",
        28,
        GREY,
        anchor="mm",
    )
    d.line((1070, 230, 1070, 890), fill="#D7DFE6", width=2)
    for j, (number, label) in enumerate(
        [
            ("2.902 kg", "Installed-mass estimate"),
            ("73.14 Wh", "Adverse case / two local recoveries"),
            ("91.43 Wh", "Delivered pack energy needed incl. reserve"),
        ]
    ):
        yy = 280 + j * 195
        text(d, (1460, yy), number, 48, BLUE, True, anchor="mm")
        text(d, (1460, yy + 64), label, 26, GREY, anchor="mm")
    text(
        d,
        (960, 930),
        "Energy feasibility requires pack discharge qualification above the loaded 19.8 V cutoff.",
        29,
        GREY,
        anchor="mm",
    )
    return im


def data_inputs():
    run = ROOT / "results/final/inference"
    rows = np.genfromtxt(
        run / "trajectory.csv", delimiter=",", names=True, dtype=None, encoding="utf8"
    )
    inventory = json.loads((run / "inventory.json").read_text())
    first = np.sort([x["first_seen_s"] for x in inventory])
    return (
        rows,
        first,
        json.loads((run / "metrics.json").read_text()),
        json.loads((ROOT / "results/final/baseline/metrics.json").read_text()),
    )


def replay_frame(views, t, row, rows, index, first, duration):
    phase = str(row["phase"])
    title = {
        "takeoff": "Lift off and enter the aisle.",
        "approach": "Enter the aisle.",
        "scan": "Continuous scanning on both sides.",
        "climb": "Climb directly to the next level.",
        "return": "Return after the fifth scan.",
        "descend": "Descend through the mapped route.",
        "land": "Land and close the flight record.",
    }.get(phase, "Recorded flight replay.")
    im = base(
        "04 / RETAINED EXPERIMENT",
        title,
        f"Saved 2.65 kg reference plant  ·  t = {float(row['t']):.1f} s  ·  {501.02 / duration:.2f}× replay",
    )
    d = ImageDraw.Draw(im)
    tile(im, views[0], (48, 180, 1296, 729), "EXTERNAL FOLLOW")
    for k, label in enumerate(
        ["LEFT BARCODE VIEW", "RIGHT BARCODE VIEW", "FORWARD NAVIGATION VIEW"]
    ):
        tile(im, views[k + 1], (1380, 180 + k * 248, 480, 240), label)

    def point(x, z):
        return (round(340 + (x + 3) / 86 * 740), round(1010 - z / 8 * 85))

    for z in [0.8, 2.4, 4, 5.6, 7.2]:
        d.line([point(0, z), point(80, z)], fill="#D6DEE6", width=2)
    past = rows[: index + 1 : 20]
    if len(past) > 1:
        d.line([point(float(r["x"]), float(r["z"])) for r in past], fill=BLUE, width=3)
    px, py = point(row["x"], row["z"])
    d.ellipse((px - 6, py - 6, px + 6, py + 6), fill=ORANGE)
    text(d, (48, 940), "Recorded route", 27, INK, True)
    text(d, (48, 980), "Position / reference", 24, GREY)
    count = int(np.searchsorted(first, float(row["t"]), side="right"))
    err = float(
        np.sqrt(
            sum(
                (row[a] - row[b]) ** 2
                for a, b in [("x", "rx"), ("y", "ry"), ("z", "rz")]
            )
        )
    )
    text(d, (1150, 940), f"{count:03d} / 400 IDs", 32, BLUE, True)
    text(d, (1500, 940), f"{err:.3f} m error", 32, INK, True)
    text(d, (1150, 984), "Counts: saved decoded images", 24, GREY)
    text(d, (1500, 984), "Views: saved-pose re-render", 24, GREY)
    return im


def result_frame(t, rows, metrics, baseline):
    gain = 100 * (1 - metrics["position_rmse_m"] / baseline["position_rmse_m"])
    im = base(
        "05 / RESULTS",
        "Control and image evidence strengthen the design.",
        "Independent full-route comparison / saved-image barcode verification",
    )
    d = ImageDraw.Draw(im)
    stats = [
        (370, f"{gain:.1f}%", "Tracking RMSE reduction"),
        (960, "400 / 400", "Correct identities after crop checks"),
    ]
    for x, n, s in stats:
        text(d, (x, 300), n, 64, BLUE, True, anchor="mm")
        text(d, (x, 380), s, 29, GREY, anchor="mm")
    text(
        d,
        (370, 435),
        f"{baseline['position_rmse_m']:.4f} to {metrics['position_rmse_m']:.4f} m",
        27,
        GREY,
        anchor="mm",
    )
    item = json.loads((ROOT / "results/final/inference/inventory.json").read_text())[0]
    crop = Image.open(
        ROOT / "results/final/inference/evidence" / item["evidence"]
    ).convert("RGB")
    im.paste(crop, (1550 - crop.width // 2, 230))
    text(d, (1550, 206), "Saved pixel evidence", 28, INK, True, anchor="mm")
    text(d, (1550, 452), "Decoded ID: " + item["barcode"], 27, GREY, anchor="mm")
    errors = np.sqrt(
        sum(
            (rows[a] - rows[b]) ** 2 for a, b in [("x", "rx"), ("y", "ry"), ("z", "rz")]
        )
    )
    x0, y0 = 150, 790
    ww, hh = 1620, 220
    d.line((x0, y0 - hh, x0, y0, x0 + ww, y0), fill=GREY, width=2)
    end = max(2, int(len(rows) * min(1, (t + 1) / 5)))
    idx = np.arange(0, end, 12)
    pts = [
        (x0 + float(rows[i]["t"]) / 501.02 * ww, y0 - float(errors[i]) / 0.2 * hh)
        for i in idx
    ]
    if len(pts) > 1:
        d.line(pts, fill=BLUE, width=2)
    for v in [0, 0.05, 0.10, 0.15, 0.20]:
        text(d, (130, y0 - v / 0.2 * hh), f"{v:.2f}", 22, GREY, anchor="rm")
    for v in [0, 100, 200, 300, 400, 500]:
        text(d, (x0 + v / 501.02 * ww, y0 + 20), str(v), 22, GREY, anchor="mt")
    text(d, (960, 845), "Time (s)", 28, GREY, anchor="mm")
    text(d, (150, 525), "Position error (m)", 26, INK, True)
    text(
        d,
        (960, 936),
        "Design analysis: custom 2.902 kg system.  Experiment: retained 2.65 kg plant and prescribed route.",
        28,
        GREY,
        anchor="mm",
    )
    text(
        d,
        (960, 988),
        "Assets: MuJoCo Menagerie / Skydio X2; Poly Haven cargo. Product images: manufacturers.",
        24,
        GREY,
        anchor="mm",
    )
    return im


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def render_part(out, act, start=0, stop=None, sample=False):
    import imageio.v2 as iio

    os.environ.setdefault("MUJOCO_GL", "osmesa")
    import mujoco
    from scipy.spatial.transform import Rotation, Slerp

    rows, first, metrics, baseline = data_inputs()
    design = json.loads((ROOT / "reports/design_calculations.json").read_text())
    durations = dict(ACTS)
    duration = durations[act]
    n = duration * FPS
    stop = min(n, stop if stop is not None else n)
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    target = out / f"{act}_{start:05d}_{stop:05d}.mp4"
    if target.exists():
        raise FileExistsError(target)
    renderer = small = None
    cam = None
    if act == "hardware":
        model, data, opt, cam = product_scene()
        renderer = mujoco.Renderer(model, 720, 1280)
    elif act == "replay":
        from .flight import create_model

        c = json.loads((ROOT / "results/final/inference/provenance.json").read_text())[
            "config"
        ]
        c["visual_assets"] = True
        model, _ = create_model(c)
        data = mujoco.MjData(model)
        renderer = mujoco.Renderer(model, 720, 1280)
        small = mujoco.Renderer(model, 240, 480)
        original = model.geom_group.copy()
        cam = mujoco.MjvCamera()
        xyz = np.column_stack([rows[k] for k in ["x", "y", "z"]])
        qs = np.column_stack([rows[k] for k in ["qx", "qy", "qz", "qw"]])
        slerp = Slerp(rows["t"], Rotation.from_quat(qs))
    output_indices = [start] if sample else range(start, stop)
    writer = (
        None
        if sample
        else iio.get_writer(
            str(target),
            fps=FPS,
            codec="libx264",
            quality=None,
            macro_block_size=1,
            ffmpeg_params=[
                "-crf",
                "18",
                "-preset",
                "fast",
                "-threads",
                "2",
                "-g",
                "30",
                "-movflags",
                "+frag_keyframe+empty_moov+default_base_moof",
            ],
        )
    )
    begin = time.monotonic()
    try:
        for i in output_indices:
            t = i / FPS
            if act == "hardware":
                cam.azimuth = 135 + 360 * t / duration
                cam.elevation = -30 + 5 * np.sin(2 * np.pi * t / duration)
                renderer.update_scene(data, camera=cam, scene_option=opt)
                frame = hardware_frame(renderer.render().copy(), t)
            elif act == "method":
                frame = method_frame(t)
            elif act == "analysis":
                frame = analysis_frame(t, design)
            elif act == "results":
                frame = result_frame(t, rows, metrics, baseline)
            else:
                rt = float(rows["t"][0] + (rows["t"][-1] - rows["t"][0]) * i / (n - 1))
                idx = int(np.searchsorted(rows["t"], rt, side="right") - 1)
                idx = min(max(idx, 0), len(rows) - 1)
                pos = np.array([np.interp(rt, rows["t"], xyz[:, k]) for k in range(3)])
                q = slerp([rt]).as_quat()[0]
                data.qpos[:3] = pos
                data.qpos[3:7] = q[[3, 0, 1, 2]]
                mujoco.mj_forward(model, data)
                cam.lookat[:] = pos
                cam.azimuth = 170 + 10 * np.sin(t / 9)
                cam.elevation = -23 - 3 * np.sin(t / 13)
                cam.distance = 2.6 + 0.15 * np.sin(t / 7)
                model.geom_group[:] = original
                model.geom_group[
                    (original == 2)
                    & (np.linalg.norm(data.geom_xpos - pos, axis=1) > 13)
                ] = 4
                renderer.update_scene(data, camera=cam)
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
                views = [renderer.render().copy()]
                for camera in ["left", "right", "front_left"]:
                    small.update_scene(data, camera=camera)
                    small.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
                    views.append(small.render().copy())
                rr = rows[idx].copy()
                for k, v in zip(["x", "y", "z"], pos):
                    rr[k] = v
                rr["t"] = rt
                frame = replay_frame(views, t, rr, rows, idx, first, duration)
            if writer:
                writer.append_data(np.asarray(frame))
            if sample or i % 240 == 0:
                frame.save(out / f"{act}_{i:05d}.jpg")
            if i % 120 == 0:
                print(f"{act} {i}/{n} {time.monotonic() - begin:.1f}s", flush=True)
    finally:
        if writer:
            writer.close()
        if renderer:
            renderer.close()
        if small:
            small.close()
    return out / f"{act}_{start:05d}.jpg" if sample else target


def assemble(out):
    import imageio_ffmpeg

    out = Path(out)
    parts = []
    for act, duration in ACTS:
        clips = sorted(out.glob(f"{act}_*_*.mp4"))
        spans = [tuple(map(int, p.stem.split("_")[-2:])) for p in clips]
        cursor = 0
        for a, b in spans:
            if a != cursor:
                raise ValueError((act, "gap/overlap", cursor, a))
            cursor = b
        if cursor != duration * FPS:
            raise ValueError((act, cursor, duration * FPS))
        parts.extend(clips)
    concat = out / "concat.txt"
    concat.write_text("".join("file '" + p.name + "'\n" for p in parts))
    target = out / "awisu_technical_film.mp4"
    if target.exists():
        raise FileExistsError(target)
    cmd = [
        imageio_ffmpeg.get_ffmpeg_exe(),
        "-v",
        "error",
        "-f",
        "concat",
        "-safe",
        "0",
        "-i",
        str(concat),
        "-c",
        "copy",
        "-video_track_timescale",
        str(FPS),
        "-movflags",
        "+faststart",
        str(target),
    ]
    subprocess.run(cmd, check=True)
    records = {
        str(p.relative_to(ROOT)): sha(p)
        for p in [
            ROOT / "results/final/inference/trajectory.csv",
            ROOT / "results/final/inference/metrics.json",
            ROOT / "results/final/inference/inventory.json",
            ROOT / "results/final/inference/evidence/0000.jpg",
            ROOT / "models/ppo_residual.zip",
        ]
    }
    provenance = {
        "chapters": [{"name": a, "seconds": b} for a, b in ACTS],
        "duration_s": sum(b for _, b in ACTS),
        "fps": FPS,
        "size": [W, H],
        "font": "Comic Neue Regular / Bold",
        "all_replay_views_native_fps": FPS,
        "native_view_sizes": {"external": [1280, 720], "left_right_front": [480, 240]},
        "full_simulation_duration_s": float(data_inputs()[0][-1]["t"]),
        "pose_interpolation": "linear position, quaternion SLERP at 30 output frames/s",
        "new_inference": False,
        "new_decode_test": False,
        "replay_speed": float(data_inputs()[0][-1]["t"]) / dict(ACTS)["replay"],
        "instantaneous_error_display": "Interpolated display position compared with the preceding saved reference sample; scientific aggregate metrics are read unchanged from metrics.json.",
        "source_code_sha256": sha(Path(__file__)),
        "font_sha256": {
            p.name: sha(p) for p in (ROOT / "assets/fonts").glob("ComicNeue-*.ttf")
        },
        "design_calculations_sha256": sha(ROOT / "reports/design_calculations.json"),
        "replay_views": ["external", "left", "right", "front_left"],
        "source_sha256": records,
        "output_sha256": sha(target),
        "interrupted_render_recovery": json.loads(
            (out / "recovery/recovery.json").read_text()
        )
        if (out / "recovery/recovery.json").exists()
        else None,
        "design_vs_experiment": "Hardware and method are proposed design; replay uses retained 2.65 kg prescribed-route plant.",
        "style_reference": "https://showlab.github.io/Show-Harness/",
        "external_reference_footage_used": False,
        "product_photo_adjustment": "White background margins trimmed; product pixels retained, then display resized.",
        "product_mesh": "Retained simulation aircraft with project guards and fixed scanning payload; licensed Menagerie Skydio X2 appearance. Proposed custom component selections are separate.",
    }
    (out / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    return target


def build(out, jobs=4):
    """Render independent ranges in subprocesses, then verify and concatenate."""
    from concurrent.futures import ThreadPoolExecutor, as_completed

    out = Path(out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    plans = [(a, 0, b * FPS) for a, b in ACTS if a != "replay"]
    n = dict(ACTS)["replay"] * FPS
    plans.extend(("replay", i, min(i + 120, n)) for i in range(0, n, 120))

    def work(plan):
        a, b, c = plan
        env = {
            **os.environ,
            "PYTHONPATH": str(ROOT / "src"),
            "MUJOCO_GL": "osmesa",
            "LP_NUM_THREADS": "2",
            "OPENBLAS_NUM_THREADS": "1",
            "OMP_NUM_THREADS": "1",
        }
        with (out / f"{a}_{b:05d}.log").open("w") as log:
            subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "segd.technical_film",
                    "--out",
                    str(out),
                    "--act",
                    a,
                    "--start",
                    str(b),
                    "--stop",
                    str(c),
                ],
                cwd=ROOT,
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                check=True,
            )
        print("Completed", plan, flush=True)

    with ThreadPoolExecutor(max_workers=jobs) as pool:
        for f in as_completed([pool.submit(work, p) for p in plans]):
            f.result()
    return assemble(out)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--act", choices=[a for a, _ in ACTS])
    p.add_argument("--start", type=int, default=0)
    p.add_argument("--stop", type=int)
    p.add_argument("--sample", action="store_true")
    p.add_argument("--assemble", action="store_true")
    p.add_argument("--build", action="store_true")
    p.add_argument("--jobs", type=int, default=4)
    a = p.parse_args()
    if a.build:
        if not 1 <= a.jobs <= 4:
            p.error("--jobs must be between 1 and 4")
        result = build(a.out, a.jobs)
    elif a.assemble:
        result = assemble(a.out)
    elif a.act:
        result = render_part(a.out, a.act, a.start, a.stop, a.sample)
    else:
        p.error("Choose --build, --assemble or --act")
    print(result)


if __name__ == "__main__":
    main()
