# Released controller

`ppo_residual.zip` is the validation-best PPO checkpoint from training seed 2, saved at 500,000 environment steps. The complete training run reached 1,000,000 steps.

The policy uses a 25-element observation and outputs three bounded acceleration residuals. It supplements the cascaded flight controller and requires the observation layout defined by `segd.flight.observation`. It does not generate the inventory mission route or decode camera images.

Training curves are stored in `results/training/seed2/`; paired selection results are stored in `results/final/paired_results.json`. The model SHA-256 is recorded in `results/final/inference/provenance.json`.
