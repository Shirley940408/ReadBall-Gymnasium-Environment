# CST8509 Assignment 2 – Red Ball Tracking with Gymnasium and ROS 2

This project builds a Gymnasium-compatible environment integrated with ROS 2 to control a simulated iRobot Create3 robot. The goal is to develop agents (both reinforcement learning and non-RL) that can detect and track a red ball in a simulated environment.

## 🧠 Project Structure

```
Assn2/ 
├── aisd_examples/                  # Custom Gymnasium environment package
│   ├── setup.py
│   └── aisd_examples/
│       ├── __init__.py
│       └── envs/
│           ├── __init__.py
│           ├── create_red_ball.py   # Custom CreateRedBall-v0 environment
│           └── blocks_world.py      # Unused in this assignment
├── null.py                         # Basic agent, does nothing useful
├── non_rl.py                       # Heuristic agent (no learning)
├── qlearning.py                    # Q-learning implementation
├── dqn.py                          # DQN using Stable-Baselines3
├── ppo.py                          # PPO using Stable-Baselines3
├── evaluate.py                     # Evaluation script for trained RL agents
└── [generated plots].png           # Output graphs (e.g., qlearning_returns.png)
```


---

## 📦 Prerequisites

- Python 3.10+
- ROS 2 Humble
- Gazebo simulation with the iRobot Create3 plugin
- Ubuntu 22.04 recommended

---

## 📚 Required Python Packages

Install Python dependencies (preferably in a virtualenv):

```bash
pip install --upgrade pip
pip install gymnasium stable-baselines3[extra] matplotlib opencv-python cv_bridge
sudo apt install ros-humble-cv-bridge
```
---

## 🔧 Setup Instructions
> 1. Clone the repo

```
git clone https://github.com/your-username/redball-tracking.git
cd redball-tracking
```

> 2. Install the Gymnasium environment
```
cd aisd_examples
pip install -e .
```

> 3. Launch your simulation in ROS 2 (before running any agent)

Make sure to:

- Launch the simulation

- Confirm that /custom_ns/camera1/image_raw is publishing images


> 4. Test that the red ball is visible

Use rqt or a custom viewer to confirm the ball is in the robot’s view.
---

## ▶️ How to Run Agents
### 🚀 Non-RL Agent (Heuristic-Based)
```
python3 non_rl.py
```
This agent:

- Moves forward with slight twist

- Stops and centers the red ball when detected

- Avoids obstacles based on /stop_status

### 🧠 Q-Learning Agent
```
python3 qlearning.py
```
Trains a Q-table from scratch. Outputs a plot: qlearning_returns.png

### 🤖 PPO / DQN (Deep RL)
Train:
```
python3 ppo.py
# or
python3 dqn.py
```
Evaluate saved model (optional):
```
python3 evaluate.py
```
---
## 🔁 Environment Behavior
- Each episode lasts 100 steps

- Reward is highest when the red ball is centered (x ≈ 320)

- Agents receive camera input and publish Twist messages to move
---
## 🤝 Integration with ROS 2

- Image input: /custom_ns/camera1/image_raw

- Velocity output: /cmd_vel

- Stop status feedback: /stop_status

- Uses rclpy, cv_bridge, and sensor_msgs.msg.Image to process camera data and control motion

- Real-time feedback loop using rclpy.spin_once(...) inside each step
---
## 📈 Evaluation Metrics
Each agent logs per-episode total rewards:

- Higher reward = better tracking and centering of red ball

- Non-RL agent used as baseline

- PPO, DQN, and Q-learning compared via plotted graphs
---

## 📸 Sample Results
> Plots like ppo_returns.png, dqn_returns.png, and non_rl_returns.png visualize agent performance.

![Demo of red ball tracking](Rviz.gif)
---
## 📚 License

MIT License


## 🙌 Acknowledgements

    ROS 2 Humble

    Stable-Baselines3

    Gymnasium (OpenAI Gym v0.26+)

    iRobot Create3 ROS 2 simulation tools

## ✍️ Author

Shuo Yan – CST8509 Assignment 2