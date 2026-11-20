# AWISU

**Autonomous Warehouse Inventory with a Small UAV**  
Junjie Peng · EE5110 Technical Report

[Code](https://github.com/JackyPengJJ/AWISU) · [Report](submission/EE5110_A0354063J_JunjiePeng_Report.pdf) · [Video](https://youtu.be/ZQHlchVMqs8)

AWISU is a small-UAV design for barcode inventory in an unknown, GPS-denied warehouse aisle. The report compares hardware, navigation and control alternatives, derives imaging and mission-resource budgets, and evaluates a reference controller and barcode pipeline in MuJoCo.

## System and results

The proposed system uses bilateral cameras, lidar–inertial–visual navigation, a five-level continuous scan and immediate local recovery for missed labels. Its estimated installed mass is 2.9016 kg. The two-recovery scenario requires 73.14 Wh under the adverse power model; maintaining a 20% reserve requires 91.43 Wh delivered before the specified battery cutoff. Pack qualification remains an installation test.

The preserved experiment uses a separate 2.65 kg reference plant, fixed side cameras and a prescribed route. The selected residual controller reduced full-route position RMSE from 0.1148 m to 0.1038 m. Saved-image consistency checks retained 400 correct IDs from 1420 processed images. These records evaluate tracking and decoding; the report separately develops the proposed unknown-environment navigation and hardware design.

The final film introduces the hardware and method, then replays the continuous flight with external, bilateral and forward views. English narration and an optional subtitle track are included in the local submission video.

## Repository layout

```text
submission/             Final named report and narrated video
src/segd/               Simulation, decoding, engineering models and export code
configs/                Reference plant, collision and training configurations
models/                 Released residual-controller checkpoint
assets/                 Scene assets, report inputs and attribution
results/final/          Preserved full-route and selection records
results/training/       Recorded learning and validation curves
results/barcode_study/  Synthetic imaging-study measurements
reports/                Resolved LaTeX, Markdown, Word and final figure exports
docs/design/            Camera, label, mission and battery comparison inputs
docs/latex/             Report layout template
docs/video/             Narration script, subtitles and source provenance
scripts/                Figure, evidence and submission utilities
tests/                  Numerical, geometry and implementation tests
```

`results/final/inference/evidence/` contains the saved pixel crops underlying the reported decoding checks, including rejected candidates. These are scientific evidence, not draft illustrations. `reports/figures/` contains only the seven selected report figures and the two link icons. Old reports, process videos, review screenshots, TeX caches and local environments are excluded.

## Installation

Use Python 3.11 or newer, and run commands from this repository root:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -e .
# Needed for policy inference or training:
python -m pip install -e '.[train]'
```

On Linux, headless rendering requires OSMesa/EGL and Cairo (for example, `libosmesa6`, `libegl1`, `libgl1`, `libcairo2`). The default dependencies are declared in `pyproject.toml`; `requirements.recorded.txt` records the original experiment environment rather than a portable installation command.

```bash
python -m segd.cli doctor
python -m unittest discover -s tests -v
python -m segd.cli calculate --out runs/design_calculations.json
```

Training-dependent tests are skipped when the optional dependencies are absent.

## Reference experiment

```bash
# Write new inference output; retained results are not overwritten.
MUJOCO_GL=osmesa python -m segd.cli simulate \
  --config configs/realistic.json \
  --policy models/ppo_residual.zip --out runs/inference

# Render an existing recorded trajectory.
MUJOCO_GL=osmesa python -m segd.cli replay \
  --run results/final/inference --out runs/replay --seconds 30

# Optional: reproduce training in a new directory.
MUJOCO_GL=osmesa python -m segd.cli train \
  --seed 2 --steps 1000000 --out runs/training_seed2
```

The checkpoint is the seed-2 validation-best model at 500,000 steps; the original run completed one million steps. The observation order and residual-action bounds are defined in `src/segd/flight.py` and `training.py`. Full-route comparison and checkpoint selection use separate records.

## Report and figures

LaTeX is the authoritative layout. Editable prose is in `docs/report_source.md`; references and source conditions are in `docs/sources.json`. The engineering entry point is `src/segd/final_design.py`. `optical_model.py` implements finite-label projection and action budgets; `design_options.py` compares optics, labels and batteries. Calculations are not replacements for preserved experiment records.

To rebuild, install XeLaTeX with `fontspec`, `unicode-math`, `amsmath`, `siunitx`, `booktabs`, `longtable`, `graphicx`, `caption`, `geometry`, `titlesec`, `xurl`, `hyperref` and `fancyhdr`. Supply your licensed Times New Roman fonts as described in [assets/fonts/README.md](assets/fonts/README.md).

```bash
export AWISU_FONT_DIR=/path/to/your/times-new-roman
# Optional if xelatex is not on PATH:
export XELATEX=/path/to/xelatex
MPLBACKEND=Agg python -m segd.cli report --results results --out runs/report
```

This writes a complete PDF, LaTeX, Markdown, Word and selected figures into the new output directory and checks the 20-page limit, references and preserved evidence hashes. To compile the already resolved `reports/report.tex`, run XeLaTeX from `reports/` after setting its local font path. The packaged resolved manuscript uses a relative font path; the build command resolves your configured font directory.

## Verification and publication

```bash
python scripts/verify_release.py
```

The release verifier checks preserved data/model hashes, reported RMSE, referenced evidence files, final PDF naming and the configured video URL. The GitHub workflow runs the numerical and implementation tests. Rendering and TeX compilation are local checks, not part of CI.

The local final MP4 exceeds the usual Git file limit and is excluded by `.gitignore`; use the YouTube link when publishing this repository. The source repository is https://github.com/JackyPengJJ/AWISU. It is private; access is limited to authorized GitHub accounts. The report's Code link points to this repository. The original course PDF does not prescribe filenames; the names in `submission/` were confirmed with the author.

See [NOTICE.md](NOTICE.md) for asset, font, narration and AI-assistance attribution. Select a software license before offering the original code for licensed reuse.
