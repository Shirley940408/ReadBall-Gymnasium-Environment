import gymnasium as gym
from gymnasium import spaces
import numpy as np

class CreateRedBallEnv(gym.Env):
    metadata = {
        "render_modes": ["human"],
        "render_fps": 10
    }
    def __init__(self, render_mode=None):
        super().__init__()
        self.render_mode = render_mode
        self.observation_space = spaces.Discrete(640)
        self.action_space = spaces.Discrete(640)
        self.step_count = 0

    def reset(self, seed=None, options=None):
        self.step_count = 0
        observation = 320  # center pixel
        return observation, {}

    def step(self, action):
        self.step_count += 1
        observation = np.random.randint(0, 640)
        reward = 0
        terminated = self.step_count == 100
        truncated = False
        info = {}
        return observation, reward, terminated, truncated, info

    def render(self):
        if self.render_mode == "human":
            pass

    def close(self):
        pass
