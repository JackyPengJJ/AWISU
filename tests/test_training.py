import importlib.util
import unittest
import numpy as np


@unittest.skipUnless(
    importlib.util.find_spec("stable_baselines3"),
    "optional training dependencies missing",
)
class TrainingTests(unittest.TestCase):
    def test_gym_interface_and_checkpoint_observation(self):
        from segd.training import ResidualEnv
        from stable_baselines3.common.env_checker import check_env

        env = ResidualEnv()
        check_env(env, warn=True)
        obs, _ = env.reset(seed=123)
        self.assertEqual(obs.shape, (25,))
        for _ in range(20):
            obs, reward, terminated, truncated, info = env.step(np.zeros(3))
            self.assertTrue(np.all(np.isfinite(obs)))
            self.assertIsInstance(terminated, bool)
        env.close()
