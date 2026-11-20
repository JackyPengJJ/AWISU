import os

os.environ["MUJOCO_GL"] = "osmesa"
from pathlib import Path
import xml.etree.ElementTree as E
import mujoco
from PIL import Image

r = Path(__file__).resolve().parents[1]
p = r / "assets/vendor/menagerie/skydio_x2/x2.xml"
root = E.parse(p).getroot()
root.find("compiler").set("assetdir", str(p.parent / "assets"))
v = E.SubElement(root, "visual")
E.SubElement(v, "global", offwidth="1600", offheight="1200")
E.SubElement(
    v, "headlight", diffuse=".7 .7 .7", ambient=".45 .45 .45", specular=".2 .2 .2"
)
root.find("asset").find("mesh").set(
    "file", str(r / "assets/prepared/skydio_x2/x2_inspection.obj")
)
root.find("asset").find("mesh").set("name", "X2_lowpoly")
b = root.find("worldbody/body")
for i, (x, y, z) in enumerate(
    [(-0.14, -0.18, 0.05), (-0.14, 0.18, 0.05), (0.14, 0.18, 0.08), (0.14, -0.18, 0.08)]
):
    E.SubElement(
        b,
        "geom",
        type="ellipsoid",
        size=".13 .014 .003",
        pos=f"{x} {y} {z + 0.005}",
        euler=f"0 0 {i * 40}",
        rgba=".08 .09 .10 1",
        contype="0",
        conaffinity="0",
        mass="0",
    )
w = root.find("worldbody")
E.SubElement(w, "light", pos="0 -1 2", dir="0 .2 -1", diffuse=".7 .7 .7")
E.SubElement(w, "geom", type="plane", size="100 100 .01", pos="0 0 .02", rgba="1 1 1 1")
m = mujoco.MjModel.from_xml_string(E.tostring(root).decode())
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)
c = mujoco.MjvCamera()
c.lookat[:] = [0, 0, 0.15]
c.distance = 0.88
c.azimuth = 135
c.elevation = -35
opt = mujoco.MjvOption()
opt.geomgroup[3] = 0
opt.sitegroup[:] = 0
ren = mujoco.Renderer(m, 1200, 1600)
ren.update_scene(d, camera=c, scene_option=opt)
Image.fromarray(ren.render()).save(
    r / "assets/report_sources/skydio_reference_render.png"
)
ren.close()
