import gymnasium as gym
import aisd_examples
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
import matplotlib.pyplot as plt

# Create vectorized environment
env = make_vec_env("aisd_examples/CreateRedBall-v0", n_envs=1)

# Create the PPO agent
model = PPO("MlpPolicy", env, verbose=1)

# Custom reward logging
episode_rewards = []
obs = env.reset()
total_reward = 0

# Train manually with model.learn() in chunks
for i in range(1000):
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
model.save("ppo_redball_model")

# Plot
plt.plot(episode_rewards)
plt.title("PPO Returns Over Episodes (Training)")
plt.xlabel("Episode")
plt.ylabel("Total Reward")
plt.grid()
plt.savefig("ppo_returns.png")
plt.show()
