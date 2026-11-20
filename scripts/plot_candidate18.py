"""Plot saved screening outputs; no generated or smoothed data."""

import json
from pathlib import Path
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parents[1]
out = root / "results/candidate18"
plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "font.size": 11.5,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "pdf.fonttype": 42,
    }
)
r = json.loads((out / "battery_screen.json").read_text())
fig, axs = plt.subplots(1, 3, figsize=(14, 4.4), layout="constrained")
for n, col in [(2, "#0072B2"), (8, "#009E73"), (20, "#D55E00")]:
    rows = [
        x
        for x in r["results"]
        if x["cutoff_v"] == 18
        and abs(x["capacity_scale"] - 3000 / 3120) < 1e-8
        and x["recoveries"] == n
    ]
    axs[0].plot(
        [x["interconnect_ohm"] * 1000 for x in rows],
        [100 * x["reserve_fraction"] for x in rows],
        marker="o",
        label=f"{n} recoveries",
        color=col,
    )
axs[0].axhline(20, color=".4", ls="--", lw=1, label="Design reserve")
axs[0].set(
    xlabel="Interconnect resistance (mΩ)",
    ylabel="Remaining delivered energy (%)",
    title="(a) Loaded battery model, 23°C",
)
axs[0].legend(frameon=False, fontsize=10)
a = np.genfromtxt(out / "control_seed0.csv", delimiter=",", names=True)
axs[1].plot(
    a["time_s"], a["ref_x"] - a["ref_x"][0], ls="--", color=".4", label="Reference x"
)
axs[1].plot(a["time_s"], a["x"] - a["x"][0], color="#0072B2", label="Actual x")
axs[1].plot(a["time_s"], a["z"] - a["z"][0], color="#009E73", label="Actual z")
axs[1].set(
    xlabel="Time (s)", ylabel="Displacement (m)", title="(b) Separate rigid-body test"
)
axs[1].legend(frameon=False, fontsize=10)
axs[2].plot(a["time_s"], a["current_a"], color="#0072B2", label="Modelled bus current")
axs[2].axhline(30, color="#D55E00", ls="--", label="Shared current limit")
axs[2].set(
    xlabel="Time (s)",
    ylabel="Current at 18 V (A)",
    ylim=(18, 32),
    title="(c) Voltage-limited power model",
)
axs[2].legend(frameon=False, fontsize=10)
for ax in axs:
    ax.grid(axis="y", alpha=0.2)
for ext in ["pdf", "png"]:
    fig.savefig(out / f"candidate18_validation.{ext}", dpi=180)
