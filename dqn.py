import gymnasium as gym
import aisd_examples  # Ensure your env is registered
from stable_baselines3 import DQN
from stable_baselines3.common.env_util import make_vec_env
import matplotlib.pyplot as plt

# Create vectorized environment
env = make_vec_env("aisd_examples/CreateRedBall-v0", n_envs=1)

# Initialize DQN model
model = DQN("MlpPolicy", env, verbose=1)

# Manual training loop to log episode rewards
obs = env.reset()
episode_rewards = []
total_reward = 0

for i in range(1000):  # number of steps or use total_timesteps
    action, _ = model.predict(obs)
    obs, reward, done, info = env.step(action)
    total_reward += reward[0]

    if done:
        episode_rewards.append(total_reward)
        total_reward = 0
        obs = env.reset()
    else:
        obs = obs

# Save model
model.save("dqn_redball_model")

# Plot reward curve
plt.plot(episode_rewards)
plt.title("DQN Returns Over Episodes (Training)")
plt.xlabel("Episode")
plt.ylabel("Total Reward")
plt.grid()
plt.savefig("dqn_returns.png")
plt.show()
