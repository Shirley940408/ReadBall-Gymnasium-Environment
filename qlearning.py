# qlearning agent
import gymnasium as gym
import aisd_examples  # ensures environment is registered
import numpy as np
import matplotlib.pyplot as plt

env = gym.make("aisd_examples/CreateRedBall-v0")

# Q-learning parameters
alpha = 0.1        # learning rate
gamma = 0.99       # discount factor
epsilon = 0.1      # exploration probability
num_episodes = 100  # number of training episodes

# Environment dimensions
obs_space = env.observation_space.n
action_space = env.action_space.n

# Initialize Q-table
Q = np.zeros((obs_space, action_space))

episode_returns = []

for episode in range(num_episodes):
    state, _ = env.reset()
    total_reward = 0

    done = False
    while not done:
        # Epsilon-greedy action selection
        if np.random.rand() < epsilon:
            action = np.random.randint(action_space)
        else:
            action = np.argmax(Q[state])

        next_state, reward, terminated, truncated, _ = env.step(action)
        done = terminated or truncated

        # Q-learning update rule
        Q[state, action] = Q[state, action] + alpha * (
            reward + gamma * np.max(Q[next_state]) - Q[state, action]
        )

        total_reward += reward
        state = next_state

    episode_returns.append(total_reward)
    print(f"Episode {episode + 1}: Total return = {total_reward}")

env.close()

# Plotting
plt.plot(episode_returns)
plt.xlabel("Episode")
plt.ylabel("Return")
plt.title("Original Hyperparameters")
plt.grid(True)
plt.savefig("qlearning_returns.png")
plt.show()
