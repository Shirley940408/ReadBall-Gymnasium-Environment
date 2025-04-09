# null agent
import gymnasium as gym
import aisd_examples

env = gym.make("aisd_examples/CreateRedBall-v0", render_mode="human")
obs, info = env.reset()

for _ in range(100):
    action = env.action_space.sample()
    obs, reward, term, trunc, info = env.step(action)
    if term or trunc:
        obs, info = env.reset()

env.close()