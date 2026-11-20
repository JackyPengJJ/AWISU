"""Evidence-led vector figures; scientific-visualization v1.4 workflow.

Typography follows the course specification, not a venue certification.
Raw trajectories and images remain unchanged; no replicate uncertainty is invented.
"""

import hashlib
import json
import os
import numpy as np
from .flight import ROOT
from pathlib import Path

BLUE, ORANGE, GREEN, GREY = "#0072B2", "#C45100", "#007A5A", "#59636E"
LIGHT = "#EAF1F6"


def tracking_error(rows):
    return np.sqrt(
        sum(
            (rows[a] - rows[b]) ** 2 for a, b in [("x", "rx"), ("y", "ry"), ("z", "rz")]
        )
    )


def error_envelope(rows, width=1.0):
    """Fixed-width min/max bins preserve excursions; these are not confidence intervals."""
    err = tracking_error(rows)
    bins = np.floor(rows["t"] / width).astype(int)
    ids = np.unique(bins)
    return {
        "time_s": ((ids + 0.5) * width).tolist(),
        "min_m": [float(err[bins == i].min()) for i in ids],
        "max_m": [float(err[bins == i].max()) for i in ids],
    }


def make_figures(results, out, d):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager, patches
    from PIL import Image

    for folder in {
        ROOT / "assets/fonts",
        Path(os.environ.get("AWISU_FONT_DIR", str(ROOT / "assets/fonts"))),
    }:
        for f in folder.glob("*.[tT][tT][fF]"):
            font_manager.fontManager.addfont(str(f))
    style = {
        "font.family": ["Comic Neue", "Times New Roman"],
        "font.size": 12.2,
        "axes.titlesize": 12.2,
        "axes.labelsize": 12.2,
        "legend.fontsize": 12.2,
        "xtick.labelsize": 12.2,
        "ytick.labelsize": 12.2,
        "pdf.fonttype": 42,
        "svg.fonttype": "none",
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.linewidth": 0.65,
        "lines.linewidth": 1.4,
        "axes.axisbelow": True,
    }
    with plt.rc_context(style):
        return _draw(results, out, d, plt, patches, Image, matplotlib.__version__)


def _draw(results, out, d, plt, patches, Image, version):
    out.mkdir(parents=True, exist_ok=True)
    paths = []
    manifest = []

    def title(ax, letter, text):
        ax.set_title(f"({letter})  {text}", loc="center", weight="bold", pad=12)

    def clean(ax):
        ax.grid(axis="y", color="#DCE2E7", lw=0.5)
        ax.tick_params(length=3, width=0.65)

    def save(fig, name, description, data):
        for ext in ("pdf", "svg", "png"):
            fig.savefig(out / f"{name}.{ext}", dpi=300, facecolor="white")
        (out / f"{name}.data.json").write_text(json.dumps(data, indent=2) + "\n")
        manifest.append(
            {
                "figure": name,
                "description": description,
                "size_inches": list(fig.get_size_inches()),
                "font_pt": 12.2,
                "formats": ["pdf", "svg", "png"],
                "raster_dpi": 300,
                "data": f"{name}.data.json",
            }
        )
        plt.close(fig)
        paths.append(str(out / f"{name}.pdf"))

    def arrow(ax, a, b, color=BLUE, style="->"):
        ax.annotate(
            "", xy=b, xytext=a, arrowprops=dict(arrowstyle=style, color=color, lw=1.2)
        )

    def box(ax, x, y, w, h, heading, sub="", color=BLUE):
        ax.add_patch(
            patches.FancyBboxPatch(
                (x, y),
                w,
                h,
                boxstyle="round,pad=.008,rounding_size=.012",
                facecolor="white",
                edgecolor=color,
                lw=1.1,
            )
        )
        ax.text(
            x + w / 2,
            y + h * 0.68 if sub else y + h / 2,
            heading,
            ha="center",
            va="center",
            weight="bold",
        )
        if sub:
            ax.text(x + w / 2, y + h * 0.27, sub, ha="center", va="center", color=GREY)

    fig = plt.figure(figsize=(6.8, 4.8))
    ax = fig.add_axes([0.13, 0.55, 0.82, 0.34])
    b = d["base"]
    r = d["cases"][0]
    title(ax, "a", "Matched propulsion at 22.2 V")
    ax.plot(
        b["motor_test_thrust_g"],
        b["motor_test_power_w"],
        "o-",
        color=BLUE,
        label="P14 × 4.8",
        ms=4,
    )
    ax.plot(
        [820, 1200, 1500, 1700, 1880],
        [79.92, 135.42, 210.9, 250.86, 295.26],
        "s--",
        color=ORANGE,
        label="P15 × 5",
        ms=4,
    )
    hover = (r["mass_g"] / 4, r["hover_propulsion_w"] / 4)
    ax.plot(*hover, "D", mfc="white", mec="black", ms=6, zorder=5)
    ax.annotate(
        f"Hover: {hover[0]:.1f} g, {hover[1]:.1f} W",
        xy=hover,
        xytext=(1050, 42),
        arrowprops={"arrowstyle": "-", "color": GREY},
        fontsize=11.5,
    )
    ax.set(
        xlabel="Thrust per rotor (g)",
        ylabel="Power per rotor (W)",
        xlim=(650, 1950),
        ylim=(0, 330),
        xticks=[700, 1000, 1300, 1600, 1900],
    )
    ax.legend(frameon=False, loc="upper left", ncol=2, fontsize=11.5)
    clean(ax)
    ax = fig.add_axes([0.23, 0.11, 0.48, 0.22])
    title(ax, "b", "Installed-mass headroom")
    ax.title.set_x(0.65)
    labels = ["5 Ah LiPo", "6 Ah VTC6", "6 Ah P30B"]
    mass = [3.030, d["mass_g"] / 1000, 2.891]
    margins = (3 - np.array(mass)) * 1000
    ax.axvline(0, color=GREY, lw=0.9)
    ax.barh(range(3), margins, height=0.42, color=[ORANGE, BLUE, "#AAB6BF"])
    for i, value in enumerate(margins):
        ax.annotate(
            f"{value:+.1f}",
            xy=(value, i),
            xytext=(-5 if value < 0 else 5, 0),
            textcoords="offset points",
            ha="right" if value < 0 else "left",
            va="center",
            fontsize=11.5,
        )
    ax.set(
        yticks=range(3),
        yticklabels=labels,
        xlim=(-65, 145),
        ylim=(2.6, -0.6),
        xlabel="Margin to 3 kg (g)",
        xticks=[-50, 0, 50, 100],
    )
    ax.spines["left"].set_visible(False)
    ax.tick_params(axis="y", length=0)
    ax.grid(axis="x", color="#E4E8EC", lw=0.5)
    for i, status in enumerate(
        ["Over mass limit", "Selected; 30 A", "Current unspecified"]
    ):
        ax.text(
            1.10,
            i,
            status,
            transform=ax.get_yaxis_transform(),
            va="center",
            fontsize=10.5,
            color=BLUE if i == 1 else GREY,
        )
    save(
        fig,
        "02_propulsion",
        "Matched manufacturer samples and installed-mass headroom; positive margins lie below the 3 kg limit. Pack-current status is reported separately.",
        {
            "p14_thrust_g": b["motor_test_thrust_g"],
            "p14_power_w": b["motor_test_power_w"],
            "hover_point": hover,
            "candidate_labels": labels,
            "candidate_mass_kg": mass,
            "mass_headroom_g": margins.tolist(),
            "p15_thrust_g": [820, 1200, 1500, 1700, 1880],
            "p15_power_w": [79.92, 135.42, 210.9, 250.86, 295.26],
        },
    )

    fig, axes = plt.subplots(1, 2, figsize=(6.8, 3.7))
    fig.subplots_adjust(left=0.11, right=0.97, top=0.85, bottom=0.19, wspace=0.55)
    ax = axes[0]
    title(ax, "a", "Lateral clearance")
    bank = np.linspace(0, 20, 201)
    extent = 0.5 * np.cos(np.deg2rad(bank)) + 0.25 * np.sin(np.deg2rad(bank))
    centred = 1.25 - extent - 0.15 - 0.15 - 0.05
    ax.plot(bank, centred, color=BLUE, label="Centred")
    ax.plot(bank, centred - 0.10, color=ORANGE, ls="--", label="0.10 m recovery")
    ax.axhline(0.25, color=GREY, ls=":", lw=1, label="Admission target")
    ax.axvline(12, color=GREY, ls="--", lw=0.8, ymax=0.60)
    ax.set(
        xlabel="Bank angle (°)",
        ylabel="Residual clearance (m)",
        xlim=(0, 20),
        ylim=(0.20, 0.52),
        xticks=[0, 6, 12, 18],
    )
    ax.legend(frameon=False, loc="upper right", fontsize=10.5)
    clean(ax)
    ax = axes[1]
    title(ax, "b", "Vertical LiDAR field")
    distance = np.linspace(0, 2, 201)
    lower = distance * np.tan(np.deg2rad(-7))
    upper = distance * np.tan(np.deg2rad(52))
    ax.fill_between(distance, lower, upper, color="#E1EDF4")
    ax.fill_between(distance, -0.5, lower, color="#F6EAE0")
    ax.plot(distance, upper, color=BLUE, label="+52° upper ray")
    ax.plot(distance, lower, color=ORANGE, ls="--", label="−7° lower ray")
    ax.axhline(0, color=GREY, lw=0.7, ls=":")
    ax.scatter([0], [0], color=BLUE, s=25, zorder=5)
    ax.set(
        xlabel="Horizontal range (m)",
        ylabel="Height from sensor (m)",
        xlim=(0, 2),
        ylim=(-0.5, 2.7),
        xticks=[0, 1, 2],
    )
    ax.legend(frameon=False, loc="upper left", fontsize=10.5)
    ax.text(1.25, 0.65, "Nominal field", ha="center", color=BLUE)
    ax.text(1.0, -0.40, "Lower blind zone", ha="center", color=ORANGE, fontsize=10.5)
    clean(ax)
    save(
        fig,
        "04_observability",
        "Analytical lateral clearance and LiDAR elevation envelope. Descent into the lower blind zone requires a previously mapped free column.",
        {
            "bank_angle_deg": bank.tolist(),
            "centred_clearance_m": centred.tolist(),
            "recovery_clearance_m": (centred - 0.1).tolist(),
            "range_m": distance.tolist(),
            "lower_height_m": lower.tolist(),
            "upper_height_m": upper.tolist(),
            "aisle_width_m": 2.5,
            "offset_limit_m": 0.1,
            "lidar_elevation_deg": [-7, 52],
        },
    )

    # Unwrap the rack face: each coloured footprint admits the complete label,
    # including the same corner, pose and position bounds used in the calculator.
    c = d["followup"]["coverage"]
    z = np.asarray(c["height_centres_m"])
    widths = np.asarray(c["safe_width_by_pose_m"])
    envelope = widths.max(axis=0)
    required = 0.85 * 0.604
    fig = plt.figure(figsize=(6.8, 4.8))
    gs = fig.add_gridspec(
        2,
        2,
        left=0.12,
        right=0.97,
        top=0.88,
        bottom=0.09,
        height_ratios=[2.25, 1],
        width_ratios=[1.1, 1],
        hspace=0.8,
        wspace=0.42,
    )
    critical = int(envelope.argmin())
    zc = z[critical]
    ax = fig.add_subplot(gs[0, 0])
    title(ax, "a", "Three pitches, one band")
    ax.set_facecolor("#F4F5F6")
    for pitch, w, color in zip([-28, 0, 28], widths, [BLUE, GREEN, ORANGE]):
        valid = w > 0
        ax.fill_betweenx(
            z,
            -w / 2,
            w / 2,
            where=valid,
            facecolor=color,
            alpha=0.20,
            edgecolor=color,
            lw=1.2,
        )
        zi = float(np.median(z[valid]))
        ax.text(
            0.50,
            zi,
            f"{pitch:+d}°" if pitch else "0°",
            va="center",
            ha="left",
            color=color,
            fontsize=11.5,
        )
        ax.add_patch(
            patches.Rectangle(
                (-0.05, zi - 0.022),
                0.1,
                0.044,
                fc="white",
                ec=color,
                lw=0.8,
                zorder=5,
            )
        )
        for xx in np.linspace(-0.045, 0.045, 18):
            ax.plot(
                [xx, xx], [zi - 0.014, zi + 0.014], color="#252C33", lw=0.5, zorder=6
            )
    ax.axvline(0, color=GREY, lw=0.6, ls=":")
    ax.axhline(zc, color=GREY, lw=0.8, ls="--")
    ax.set(
        xlim=(-0.6, 0.7),
        ylim=(-0.8, 0.8),
        xticks=[-0.5, 0, 0.5],
        yticks=[-0.8, -0.4, 0, 0.4, 0.8],
        ylabel="Label-centre height (m)",
        xlabel="Along-track offset (m)",
    )
    ax.tick_params(length=2)
    ax = fig.add_subplot(gs[0, 1])
    title(ax, "b", "Keep all four corners")
    ax.set(xlim=(0, 1), ylim=(0, 1))
    ax.axis("off")
    ax.add_patch(
        patches.Rectangle((0.04, 0.30), 0.90, 0.61, fc="#F4F5F6", ec=GREY, lw=1)
    )
    ax.add_patch(
        patches.Rectangle((0.08, 0.34), 0.82, 0.53, fc="white", ec=BLUE, ls="--", lw=1)
    )

    def tag(x, y, w=0.56):
        ax.add_patch(patches.Rectangle((x, y), w, 0.10, fc="white", ec=GREY, lw=0.8))
        for k, xx in enumerate(np.linspace(x + 0.04, x + w - 0.04, 24)):
            ax.plot(
                [xx, xx],
                [y + 0.018, y + 0.082],
                color="#252C33",
                lw=0.65 if k % 3 else 1.2,
            )

    tag(0.24, 0.45)
    tag(0.46, 0.72)
    for x, y in [(0.24, 0.45), (0.80, 0.45), (0.24, 0.55), (0.80, 0.55)]:
        ax.plot(x, y, "o", ms=3, color=GREEN)
    ax.plot([0.94, 0.94], [0.67, 0.88], color=ORANGE, lw=2)
    ax.text(0.50, 0.60, "Complete label", ha="center", color=GREEN, fontsize=11.5)
    ax.text(
        0.50, 0.97, "Clipped label: reject", ha="center", color=ORANGE, fontsize=11.5
    )
    ax.text(0.5, 0.18, "100 × 44 mm incl. quiet zones", ha="center", fontsize=11.5)
    ax.text(
        0.5,
        0.07,
        "Pose / range / position margins",
        ha="center",
        fontsize=11.5,
        color=GREY,
    )
    ax = fig.add_subplot(gs[1, :])
    title(ax, "c", "Worst-height travel budget")
    ax.set(xlim=(-0.42, 0.42), ylim=(-0.1, 1.2))
    ax.axis("off")
    safe = float(envelope.min())
    half = safe / 2
    ax.add_patch(
        patches.Rectangle((-half, 0.28), safe, 0.38, fc="#E3EEE9", ec=GREEN, lw=1)
    )
    ax.add_patch(
        patches.Rectangle(
            (-required / 2, 0.28), required, 0.38, fc="#DCEAF4", ec=BLUE, lw=1
        )
    )
    ax.text(
        0,
        0.47,
        f"{required:.3f} m needed at 0.85 m/s",
        ha="center",
        va="center",
        fontsize=11.5,
        color=BLUE,
    )
    ax.annotate(
        "",
        xy=(-half, 0.93),
        xytext=(half, 0.93),
        arrowprops=dict(arrowstyle="|-|", color=GREEN, lw=1),
    )
    ax.text(
        0,
        1.04,
        f"{safe:.3f} m available at the limiting height",
        ha="center",
        va="bottom",
        fontsize=11.5,
        color=GREEN,
    )
    ax.text(
        0,
        0.06,
        f"{1000 * (safe - required):.0f} mm total slack  ·  0.604 s two-observation horizon",
        ha="center",
        va="top",
        fontsize=11.5,
        color=GREY,
    )
    save(
        fig,
        "05_imaging",
        "Rack-plane admissible label-centre regions at three camera pitches, after finite-label and uncertainty erosion. The common dashed height is the limiting transition; the travel budget uses the best admissible pitch.",
        {
            "height_centres_m": c["height_centres_m"],
            "safe_width_by_pose_m": c["safe_width_by_pose_m"],
            "best_pose_envelope_m": envelope.tolist(),
            "required_interval_m": required,
            "minimum_interval_m": safe,
            "limiting_height_m": float(zc),
            "inset": "schematic corner containment; drawn barcode bars carry no encoded evidence",
            "label_width_m": 0.1,
            "label_height_m": 0.044,
            "exposure_times_s": [0.14, 0.19, 0.34, 0.39, 0.54, 0.59],
            "cycle_s": 0.60,
            "confirmation_horizon_s": 0.604,
        },
    )

    screen = json.loads((results / "candidate18/battery_screen.json").read_text())
    source = np.genfromtxt(
        ROOT / "docs/audit/vtc6_discharge.csv", delimiter=",", names=True
    )
    fig, axs = plt.subplots(2, 1, figsize=(6.8, 5.4))
    fig.subplots_adjust(left=0.14, right=0.96, bottom=0.10, top=0.89, hspace=0.9)
    ax = axs[0]
    title(ax, "a", "Typical cell discharge at 23°C")
    for amps, color in [(10, BLUE), (15, ORANGE)]:
        ax.plot(
            source["capacity_mah"],
            source[f"voltage_{amps}a_v"],
            "o-",
            ms=3,
            color=color,
            label=f"{amps} A / cell",
        )
    for cutoff, style in [(3.3, ":"), (3.0, "--")]:
        ax.axhline(cutoff, color=GREY, ls=style, lw=1)
        ax.text(
            100,
            cutoff,
            f"{6 * cutoff:.1f} V pack",
            ha="left",
            va="bottom",
            fontsize=10,
        )
    ax.set(
        xlabel="Discharged capacity (mAh / cell)",
        ylabel="Loaded voltage (V)",
        xlim=(0, 2800),
        ylim=(2.85, 3.95),
    )
    ax.legend(frameon=False, ncol=2, fontsize=10, loc="upper right")
    clean(ax)
    ax = axs[1]
    title(ax, "b", "Reserve after the adverse mission")
    plotted = []
    for n, color in [(2, BLUE), (8, GREEN), (20, ORANGE)]:
        rows = [
            x
            for x in screen["results"]
            if x["cutoff_v"] == 18
            and abs(x["capacity_scale"] - 3000 / 3120) < 1e-8
            and x["recoveries"] == n
        ]
        ax.plot(
            [x["interconnect_ohm"] * 1000 for x in rows],
            [x["reserve_fraction"] * 100 for x in rows],
            "o-",
            ms=3,
            color=color,
            label=f"{n} recoveries",
        )
        plotted.extend(rows)
    ax.axhline(20, color=GREY, ls="--", lw=1)
    ax.text(
        29.5,
        20.3,
        "20% requirement",
        ha="right",
        va="bottom",
        fontsize=10,
        bbox=dict(facecolor="white", edgecolor="none", pad=1),
    )
    ax.set(
        xlabel="Assembly connection resistance (mΩ)",
        ylabel="Delivered-energy reserve (%)",
        xlim=(0, 30),
        ylim=(-1, 33),
        xticks=[0, 10, 20, 30],
    )
    ax.legend(frameon=False, ncol=3, fontsize=10, loc="upper right")
    clean(ax)
    save(
        fig,
        "07_energy",
        "Digitised typical cell curves and coupled pack/load reserve sensitivity. The voltage shift is 40 mV/cell and capacity-axis scale is 3000/3120; no measured pack discharge is claimed.",
        {
            "curve_source": "docs/audit/vtc6_source.json",
            "cell_trace": {n: source[n].tolist() for n in source.dtype.names},
            "screening": plotted,
        },
    )

    run = results / "final/inference"
    rows = np.genfromtxt(
        run / "trajectory.csv", delimiter=",", names=True, dtype=None, encoding="utf8"
    )
    base = np.genfromtxt(
        results / "final/baseline/trajectory.csv",
        delimiter=",",
        names=True,
        dtype=None,
        encoding="utf8",
    )
    sel = json.loads((results / "final/selection.json").read_text())
    met = json.loads((run / "metrics.json").read_text())
    bm = json.loads((results / "final/baseline/metrics.json").read_text())
    assert (
        abs(float(np.sqrt(np.mean(tracking_error(rows) ** 2))) - met["position_rmse_m"])
        < 1e-8
    )
    fig = plt.figure(figsize=(6.8, 4.2))
    gs = fig.add_gridspec(
        2, 2, left=0.12, right=0.98, top=0.90, bottom=0.13, wspace=0.65, hspace=1.0
    )
    ax = fig.add_subplot(gs[:, 0])
    title(ax, "a", "Checkpoint selection")
    names = list(sel["scores_rmse_m"])
    scores = list(sel["scores_rmse_m"].values())
    for i, (name, v) in enumerate(zip(names, scores)):
        ax.plot([0, v], [i, i], color="#DCE2E7", lw=2)
        ax.plot(
            v,
            i,
            "D" if name == sel["selected"] else "o",
            color=ORANGE if name == sel["selected"] else BLUE,
            ms=5,
        )
        ax.text(v + 0.004, i, f"{v:.3f}", va="center")
    ax.set(
        yticks=range(len(names)),
        yticklabels=[
            "Base",
            "s0 best",
            "s0 final",
            "s1 best",
            "s1 final",
            "s2 best",
            "s2 final",
        ],
        xlim=(0, 0.22),
        xlabel="Selection RMSE (m)",
    )
    ax.invert_yaxis()
    ax.axvline(scores[0], color=GREY, ls=":", lw=1)
    ax.set_xticks([0, 0.1, 0.2])
    ax = fig.add_subplot(gs[0, 1])
    title(ax, "b", "Full-route error")
    data = {}
    for rr, color, label, ls in [(base, GREY, "Base", "--"), (rows, BLUE, "PPO", "-")]:
        e = error_envelope(rr)
        data[label] = e
        ax.fill_between(e["time_s"], e["min_m"], e["max_m"], color=color, alpha=0.18)
        ax.plot(e["time_s"], e["max_m"], color=color, ls=ls, lw=0.8, label=label)
    ax.set(
        xlim=(0, 502),
        ylim=(0, max(tracking_error(rows).max(), tracking_error(base).max()) * 1.08),
        xlabel="Time (s)",
        ylabel="Error (m)",
    )
    ax.legend(frameon=False, ncol=2, loc="upper right", handlelength=1)
    clean(ax)
    ax = fig.add_subplot(gs[1, 1])
    title(ax, "c", "Saved pixel evidence")
    inv = json.loads((run / "inventory.json").read_text())
    ax.imshow(
        Image.open(run / "evidence" / inv[0]["evidence"]), interpolation="nearest"
    )
    ax.axis("off")
    ax.text(
        0.5,
        -0.16,
        f"{met['read_items']}/{met['expected_items']} IDs · {met['sensor_frames_processed']} images",
        transform=ax.transAxes,
        ha="center",
    )
    save(
        fig,
        "08_experiment",
        "All selection outcomes; fixed 1 s min/max error envelopes from one complete route per controller; untouched barcode crop. No confidence intervals.",
        {
            "selection": sel,
            "full_route_error_bins": data,
            "rmse_m": {"Base": bm["position_rmse_m"], "PPO": met["position_rmse_m"]},
            "barcode_crop": inv[0]["evidence"],
            "raw_rows": {"Base": len(base), "PPO": len(rows)},
            "full_routes_per_controller": 1,
        },
    )
    sources = [
        "results/final/inference/trajectory.csv",
        "results/final/baseline/trajectory.csv",
        "results/final/selection.json",
        "results/final/inference/metrics.json",
        "results/final/inference/inventory.json",
    ]
    (out / "manifest.json").write_text(
        json.dumps(
            {
                "style_basis": "scientific-visualization v1.4; general conference conventions, not venue certification",
                "font": "Comic Neue 12.2 pt; Times fallback for mathematical symbols",
                "matplotlib": version,
                "figures": manifest,
                "input_sha256": {
                    p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest()
                    for p in sources
                },
            },
            indent=2,
        )
        + "\n"
    )
    return paths
