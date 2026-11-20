"""Compare the released policy with the baseline under matched proxy conditions."""

import argparse, json
from pathlib import Path
from segd.flight import ROOT, load_config, simulate

p = argparse.ArgumentParser()
p.add_argument("--out", required=True)
p.add_argument("--full", action="store_true")
a = p.parse_args()
out = Path(a.out)
out.mkdir(parents=True, exist_ok=False)
results = []
for seed, wind in [(701, 0.0), (702, 1.0)]:
    for name, policy in [
        ("baseline", None),
        ("policy", str(ROOT / "models/ppo_residual.zip")),
    ]:
        c = load_config(ROOT / "configs/realistic.json")
        c.update(state_source="proxy", seed=seed, wind_n=wind)
        metrics = simulate(c, out / f"{name}_{seed}", short=not a.full, policy=policy)
        results.append({"controller": name, "seed": seed, "wind_n": wind, **metrics})
(out / "comparison.json").write_text(json.dumps(results, indent=2))
print(json.dumps(results, indent=2))
