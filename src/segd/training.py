"""PPO residual training on a delayed-state proxy; NOT complete sensor navigation."""

import json
import hashlib
import time
from collections import deque
from pathlib import Path
import numpy as np
import mujoco
import gymnasium as gym
from gymnasium.spaces import Box
from .flight import (
    ROOT,
    load_config,
    create_model,
    Controller,
    state,
    observation,
    provenance,
    snapshot_source,
    COLLISION,
    collision_clearance,
)


class ResidualEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, randomize=True):
        self.randomize = randomize
        calibration = ROOT / "configs/navigation_calibration.npz"
        calibration_meta = ROOT / "configs/navigation_calibration.json"
        self.calibration_valid = calibration_meta.exists() and json.loads(
            calibration_meta.read_text()
        ).get("use_for_new_training", False)
        self.calibration = (
            np.load(calibration)
            if calibration.exists() and self.calibration_valid
            else None
        )
        self.observation_space = Box(-np.inf, np.inf, (25,), np.float32)
        self.action_space = Box(-1, 1, (3,), np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        rng = self.np_random
        self.c = load_config(ROOT / "configs/realistic.json")
        if self.randomize:
            self.c["mass_kg"] *= rng.uniform(0.85, 1.15)
            self.c["motor_tau_s"] *= rng.uniform(0.5, 2)
            self.c["max_motor_thrust_n"] *= rng.uniform(0.9, 1.1)
        self.model, _ = create_model(self.c)
        self.data = mujoco.MjData(self.model)
        self.data.qpos[:3] = [10, 0, 4]
        mujoco.mj_forward(self.model, self.data)
        self.ctrl = Controller(self.c)
        self.steps = 0
        self.previous = np.zeros(3)
        self.delay = int(rng.integers(1, 7)) if self.randomize else 1
        self.queue = deque(maxlen=self.delay)
        self.estimate = None
        self.wind = rng.uniform(-1, 1, 2) if self.randomize else np.zeros(2)
        self.calibration_offset = (
            int(rng.integers(0, len(self.calibration["position_error"])))
            if self.calibration is not None and self.randomize
            else 0
        )
        self.phase = rng.uniform(0, 2 * np.pi)
        self.ref = self.reference(0)
        self._measure()
        return self._obs(), {}

    def reference(self, t):
        # Shared 6-DoF plant, local sinusoidal tracking segment for efficient episodes.
        p = np.array(
            [
                10 + 1.2 * np.sin(0.35 * t),
                0.12 * np.sin(0.5 * t),
                4 + 0.2 * np.sin(0.4 * t),
            ]
        )
        v = np.array(
            [0.42 * np.cos(0.35 * t), 0.06 * np.cos(0.5 * t), 0.08 * np.cos(0.4 * t)]
        )
        a = np.array(
            [
                -0.147 * np.sin(0.35 * t),
                -0.03 * np.sin(0.5 * t),
                -0.032 * np.sin(0.4 * t),
            ]
        )
        return p, v, a

    def _measure(self):
        p, v, R, w = state(self.data)
        self.queue.append((p, v))
        pe, ve = self.queue[0]
        if self.randomize:
            pe = pe + self.np_random.normal(0, 0.015, 3)
            ve = ve + self.np_random.normal(0, 0.025, 3)
        if self.calibration is not None:
            i = (self.calibration_offset + self.steps) % len(
                self.calibration["position_error"]
            )
            pe = pe + self.calibration["position_error"][i]
            ve = ve + self.calibration["velocity_error"][i]
        self.estimate = (pe, ve, R, w)

    def _obs(self):
        return observation(*self.estimate, self.ref, self.ctrl.motor, self.c)

    def step(self, action):
        action = np.clip(np.asarray(action, float), -1, 1)
        target = self.ctrl.command(*self.estimate, self.ref, action)
        for _ in range(4):
            p, v, R, w = state(self.data)
            self.data.xfrc_applied[1, :3] = (
                np.r_[self.wind * np.sin(0.7 * self.data.time + self.phase), 0]
                - 0.15 * v
            )
            self.data.ctrl[:] = self.ctrl.advance_motor(target)
            mujoco.mj_step(self.model, self.data)
        self.steps += 1
        self.ref = self.reference(self.data.time)
        self._measure()
        p, v, R, w = state(self.data)
        error = np.linalg.norm(p - self.ref[0])
        vel = np.linalg.norm(v - self.ref[1])
        collided = any(contact.dist < -0.002 for contact in self.data.contact)
        terminated = bool(
            collided
            or collision_clearance(p, R) < COLLISION["training_clearance_margin_m"]
            or error > 2
        )
        reward = (
            -4 * error**2
            - 0.2 * vel**2
            - 0.01 * np.sum(action**2)
            - 0.03 * np.sum((action - self.previous) ** 2)
        )
        if terminated:
            reward -= 50
        self.previous = action.copy()
        return (
            self._obs(),
            float(reward),
            terminated,
            self.steps >= 1000,
            {"error_m": error, "collision": collided},
        )


def train(args):
    review = ROOT / "configs/environment_review.json"
    if review.exists() and not json.loads(review.read_text()).get(
        "training_authorized_on_this_environment", False
    ):
        raise RuntimeError(
            "Training paused: environment validation is required in configs/environment_review.json"
        )
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import (
        BaseCallback,
        CallbackList,
        EvalCallback,
        CheckpointCallback,
    )
    from stable_baselines3.common.monitor import Monitor

    torch.set_num_threads(1)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    snapshot_source(out)
    (out / "provenance.json").write_text(
        json.dumps(
            {
                **provenance(load_config(ROOT / "configs/realistic.json")),
                "seed": args.seed,
                "requested_steps": args.steps,
                "scope": "proxy-state residual controller training; synthetic delayed/noisy observations; no full-perception validation",
                "navigation_calibration": json.loads(
                    (ROOT / "configs/navigation_calibration.json").read_text()
                )
                if (ROOT / "configs/navigation_calibration.json").exists()
                else None,
                "resume": args.resume,
            },
            indent=2,
        )
    )
    env = Monitor(ResidualEnv(), str(out / "training.csv"))
    val = Monitor(ResidualEnv(randomize=False), str(out / "validation.csv"))
    val.reset(seed=777)

    class WallLimit(BaseCallback):
        def __init__(self):
            super().__init__()
            self.started = time.monotonic()
            self.target = 0

        def _on_training_start(self):
            self.target = self.model.num_timesteps + args.steps

        def _on_step(self):
            return (
                time.monotonic() - self.started < args.hours * 3600
                and self.model.num_timesteps < self.target
            )

    callbacks = CallbackList(
        [
            WallLimit(),
            CheckpointCallback(save_freq=250000, save_path=str(out / "checkpoints")),
            EvalCallback(
                val,
                best_model_save_path=str(out / "best"),
                log_path=str(out / "validation"),
                eval_freq=min(250000, max(2048, args.steps // 4)),
                n_eval_episodes=3,
                deterministic=True,
            ),
        ]
    )
    if args.resume:
        model = PPO.load(args.resume, env=env, device="cpu")
    else:
        model = PPO(
            "MlpPolicy",
            env,
            seed=args.seed,
            device="cpu",
            policy_kwargs={"net_arch": [128, 128]},
            n_steps=2048,
            batch_size=128,
            n_epochs=10,
            learning_rate=3e-4,
            verbose=1,
        )
    t = time.monotonic()
    initial = model.num_timesteps
    try:
        model.learn(
            total_timesteps=args.steps,
            callback=callbacks,
            reset_num_timesteps=not bool(args.resume),
        )
    finally:
        model.save(str(out / "final"))
        env.close()
        val.close()
    result = {
        "seed": args.seed,
        "actual_total_steps": model.num_timesteps,
        "new_steps": model.num_timesteps - initial,
        "wall_s": time.monotonic() - t,
        "model_sha256": hashlib.sha256((out / "final.zip").read_bytes()).hexdigest(),
        "scope": "approved-scene residual control with synthetic delayed/noisy state; synthetic delayed/noisy observations; no full-perception validation",
    }
    (out / "metrics.json").write_text(json.dumps(result, indent=2))
    return result
