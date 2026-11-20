"""Export the report overview and component-role illustration as SVG/PDF/PNG."""

from pathlib import Path
import base64, json, os, html

R = Path(__file__).resolve().parents[1]
O = Path(os.environ.get("SEGD_FIGURE_OUT", str(R / "reports/figures")))
O.mkdir(parents=True, exist_ok=True)
# Cairo/fontconfig must see the same Times files as XeLaTeX.
font_dir = Path(os.environ.get("AWISU_FONT_DIR", str(R / "assets/fonts")))
fc = O / "fonts.conf"
fc.write_text(
    f"<fontconfig><include>/etc/fonts/fonts.conf</include><dir>{R}/assets/fonts</dir><dir>{font_dir}</dir><cachedir>/tmp/segd-font-cache</cachedir></fontconfig>"
)
os.environ["FONTCONFIG_FILE"] = str(fc)
import cairosvg

B = "#246388"
G = "#29745e"
A = "#b36b28"
K = "#252b31"
L = "#dbe2e7"


def svg_text(text):
    return "".join(
        '<tspan font-family="Times New Roman">' + html.escape(ch) + "</tspan>"
        if ch in "≥≤≠→±"
        else html.escape(ch)
        for ch in text
    )


class SVG:
    def __init__(self, h):
        self.h = h
        self.s = [
            f'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" width="480pt" height="{h}pt" viewBox="0 0 480 {h}"><rect width="480" height="{h}" fill="white"/><defs><marker id="a" markerWidth="7" markerHeight="7" refX="6" refY="3" orient="auto"><path d="M0 0L6 3L0 6Z" fill="{B}" stroke="none"/></marker></defs><g font-family="Comic Neue" font-size="12" fill="{K}">'
        ]

    def add(self, s):
        self.s.append(s)

    def t(self, x, y, s, b=False, c=K):
        self.add(
            f'<text x="{x}" y="{y}" fill="{c}" font-weight="{"bold" if b else "normal"}">{svg_text(s)}</text>'
        )

    def tc(self, x, y, s, c=K):
        self.add(
            f'<text x="{x}" y="{y}" fill="{c}" text-anchor="middle">{svg_text(s)}</text>'
        )

    def rect(self, x, y, w, h, fill="white", stroke=L):
        self.add(
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{fill}" stroke="{stroke}"/>'
        )

    def path(self, d, c=B, w=1, arrow=False, dash=False):
        self.add(
            f'<path d="{d}" fill="none" stroke="{c}" stroke-width="{w}"'
            + (' marker-end="url(#a)"' if arrow else "")
            + (' stroke-dasharray="3 3"' if dash else "")
            + "/>"
        )

    def img(self, p, x, y, w, h):
        data = base64.b64encode(Path(p).read_bytes()).decode()
        mime = "jpeg" if str(p).endswith("jpg") else "png"
        self.add(
            f'<image x="{x}" y="{y}" width="{w}" height="{h}" preserveAspectRatio="xMidYMid meet" xlink:href="data:image/{mime};base64,{data}"/>'
        )

    def save(self, name):
        data = "".join(self.s) + "</g></svg>"
        (O / f"{name}.svg").write_text(data)
        cairosvg.svg2pdf(bytestring=data.encode(), write_to=str(O / f"{name}.pdf"))
        cairosvg.svg2png(
            bytestring=data.encode(), write_to=str(O / f"{name}.png"), scale=2
        )


# Unmodified crops retained as the source images for the overview.
f = SVG(298)
for x, y, w, h, col in [
    (0, 0, 236, 161, B),
    (244, 0, 236, 161, G),
    (0, 167, 236, 131, B),
    (244, 167, 236, 131, A),
]:
    f.add(
        f'<rect x="{x + 0.8}" y="{y + 0.8}" width="{w - 1.6}" height="{h - 1.6}" rx="5" fill="white" stroke="{col}" stroke-width=".9"/>'
    )
f.add('<g transform="translate(8 4) scale(.96)">')
f.add(
    '<text x="116" y="12" text-anchor="middle" font-weight="bold">(a) Bilateral acquisition</text>'
)
f.add(
    '<text x="371" y="12" text-anchor="middle" font-weight="bold">(b) Whole-label admission</text>'
)
f.img(R / "assets/report_sources/mission_scene.png", 0, 25, 232, 119)
f.img(R / "assets/report_sources/mission_left.png", 255, 27, 105, 52)
f.img(R / "assets/report_sources/mission_right.png", 375, 27, 105, 52)
f.tc(307.5, 94, "Left rack", c=B)
f.tc(427.5, 94, "Right rack", c=G)
# ROI brackets annotate native images without altering their pixels.
for x, y, col in [(310, 36, B), (394, 46, G)]:
    f.path(f"M{x} {y}h-3v11h3M{x + 14} {y}h3v11h-3", col, 1.2)
# Panel sequence is conveyed by numbering; no cross-panel decorative arrows.
f.tc(371, 115, "Four corners inside valid FOV")
f.tc(371, 132, "At least 2.5 px / module")
f.tc(371, 149, "Pose + exposure within limits")
f.tc(116, 158, "Recorded scene; proposed processing", c=K)

f.add(
    '<text x="116" y="183" text-anchor="middle" font-weight="bold">(c) Spatial coverage record</text>'
)
f.add(
    '<text x="371" y="183" text-anchor="middle" font-weight="bold">(d) Confirm or recover</text>'
)
# Two rack banks; cell states represent projected cargo instances, not a measured map.
for bank, x0, col in [("L", 22, B), ("R", 132, G)]:
    f.t(x0, 199, bank, c=col)
    for row in range(5):
        for j in range(8):
            fill = "#d8e8e1" if row > 1 else "#eef1f3"
            if row == 1 and j < 4:
                fill = "#d8e8e1"
            if bank == "R" and row == 1 and j == 3:
                fill = "#eac59f"
            f.rect(x0 + j * 12, 204 + row * 11, 10, 9, fill, "white")
f.path("M120 252H17V241H120V230H17", B, 1.1, True)
f.path("M17 230V219H120V208H17", B, 1, True, True)
f.t(0, 273, "5 levels × 2 sides; instance ≠ SKU")
f.tc(371, 218, "Pair reads by instance")
f.tc(371, 236, "Agree: retain ID + image pair", c=G)
f.tc(371, 254, "Miss: adjust the local view", c=A)
f.tc(371, 272, "Resume after confirmation", c=A)

for x, col, label in [
    (0, "#eef1f3", "Unchecked"),
    (99, "#d8e8e1", "Confirmed"),
    (190, "#eac59f", "Retry"),
]:
    f.rect(x, 282, 7, 7, col, col)
    f.t(x + 11, 290, label)

f.add("</g>")
f.save("framework")
# Manufacturer reference photograph, with explicit substitution callouts.
f = SVG(286)
f.add(
    '<text x="142.5" y="12" text-anchor="middle" font-weight="bold">(a) Reference aircraft</text>'
)
f.add(
    '<text x="390" y="12" text-anchor="middle" font-weight="bold">(b) Component choices</text>'
)
f.img(R / "assets/report_sources/skydio_reference_render.png", 0, 22, 285, 248)
for x, y, n, yy in [(221, 127, "1", 54), (145, 142, "2", 133), (135, 165, "3", 216)]:
    f.add(
        f'<circle cx="{x}" cy="{y}" r="9" fill="white" stroke="{B}" stroke-width="1.3"/>'
    )
    f.t(x - 3, y + 4, n, True, B)
    f.path(f"M{x + 9} {y}H290V{yy}H307", B, 1, True)
for y, file, num, title, desc in [
    (27, "mn3508.jpg", "1", "Propulsion", "MN3508 KV380"),
    (106, "pixhawk6c.jpg", "2", "Flight control", "Pixhawk 6C"),
    (185, "rt_v1228.jpg", "3", "Side optics", "12 mm lens + camera"),
]:
    f.img(R / "assets/report_sources" / file, 311, y, 56, 62)
    f.t(375, y + 13, title, True)
    f.t(375, y + 31, desc if num != "3" else "12 mm lens")
    f.t(375, y + 48, {"1": "P14 × 4.8", "2": "PX4 cascade", "3": "+ pitch mount"}[num])
f.t(0, 283, "Skydio X2 reference mesh; callouts identify proposed component roles.")
f.save("hardware")
# GitHub Octicon, MIT-licensed glyph geometry (GitHub Primer); isolated brand mark.
icon = "M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 5.47 7.59.4.07.55-.17.55-.38 0-.19-.01-.82-.01-1.49-2.01.37-2.53-.49-2.69-.94-.09-.23-.48-.94-.82-1.13-.28-.15-.68-.52-.01-.53.63-.01 1.08.58 1.23.82.72 1.21 1.87.87 2.33.66.07-.52.28-.87.51-1.07-1.78-.2-3.64-.89-3.64-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82a7.65 7.65 0 0 1 4 0c1.53-1.04 2.2-.82 2.2-.82.44 1.1.16 1.92.08 2.12.51.56.82 1.27.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73.54 1.48 0 1.07-.01 1.93-.01 2.2 0 .21.15.46.55.38A8.013 8.013 0 0 0 16 8c0-4.42-3.58-8-8-8Z"
cairosvg.svg2pdf(
    bytestring=f'<svg xmlns="http://www.w3.org/2000/svg" width="12pt" height="12pt" viewBox="0 0 16 16"><path d="{icon}"/></svg>'.encode(),
    write_to=str(O / "github.pdf"),
)

(O / "framework.data.json").write_text(
    json.dumps(
        {
            "source_images": [
                "assets/report_sources/mission_" + n + ".png"
                for n in ["scene", "left", "right"]
            ],
            "blocks": ["acquire", "admit", "associate", "recover"],
            "map_is_schematic": True,
        },
        indent=2,
    )
    + "\n"
)
(O / "hardware.data.json").write_text(
    json.dumps(
        {
            "render": "assets/report_sources/skydio_reference_render.png",
            "render_script": "scripts/render_report_asset.py",
            "mesh": "assets/prepared/skydio_x2/x2_inspection.obj",
            "meaning": "Reference mesh with proposed component-role callouts",
        },
        indent=2,
    )
    + "\n"
)
