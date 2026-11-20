# Development

Run commands from the repository root. Use Python 3.11+ and install development tools with `python -m pip install -e '.[dev]'`.

```bash
ruff format src scripts tests
ruff check --select F821,F822,F823 src scripts tests
python -m unittest discover -s tests -v
python scripts/verify_release.py
```

Write new experiments to `runs/`. Preserve the saved model, trajectories and barcode evidence under `models/` and `results/`; these support the published report numbers. Update design inputs and report calculations together. Check figures at final print size and rebuild the PDF before changing report claims.

The optional `train` dependencies are required for controller training and policy inference. Rendering needs an OpenGL/EGL/OSMesa backend. Report compilation needs XeLaTeX and locally licensed Times New Roman fonts.
