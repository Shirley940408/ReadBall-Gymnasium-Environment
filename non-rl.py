import gymnasium as gym
import aisd_examples
import matplotlib.pyplot as plt

# Create environment
env = gym.make("aisd_examples/CreateRedBall-v0")

def compute_action(observation, is_stopped):
    if is_stopped:
        # 🚧 Obstacle detected — spin left in place
        print("[AVOID] Obstacle ahead! Turning left.")
        return 200  # sharp left turn (no forward motion)

    if observation is None:
        # 🔍 No ball — explore: move forward with slight right twist
        return 380
    else:
        # 🎯 Ball detected — move forward and adjust direction
        if abs(observation - 320) < 30:
            return 320  # perfectly centered → go straight
        elif observation < 320:
            return 260  # ball left → turn left while moving
        else:
            return 380  # ball right → turn right while moving

# Run agent
episode_rewards = []
num_episodes = 100

for episode in range(num_episodes):
    obs, _ = env.reset()
    terminated = False
    truncated = False
    total_reward = 0
    step_count = 0

    while not (terminated or truncated):
        # Check robot stop status from RedBall node
        is_stopped = env.unwrapped.redball.create3_is_stopped

        # Decide action
        action = compute_action(obs, is_stopped)

        # Step
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        step_count += 1

    episode_rewards.append(total_reward)
    print(f"[Episode {episode+1}] Total Reward = {total_reward}")

# Plot the episode rewards
plt.plot(episode_rewards)
plt.title("Non-RL Agent: Tracking, Exploration, and Obstacle Avoidance")
plt.xlabel("Episode")
plt.ylabel("Total Reward")
plt.grid()
plt.savefig("non_rl_returns.png")
plt.show()
