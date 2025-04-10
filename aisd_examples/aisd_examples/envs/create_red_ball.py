import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from irobot_create_msgs.msg import StopStatus

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
        # init rclpy
        if not rclpy.ok():
            rclpy.init()
        self.redball = RedBall()

    def reset(self, seed=None, options=None):
        self.step_count = 0
        return self.redball.redball_position, {}

    def step(self, action):
        # print(f"[STEP {self.step_count}] Previous redball_position = {self.redball.redball_position}")
        self.step_count += 1

        twist = Twist()

        if self.step_count <= 1:
            # 🚗 Phase 1: Move straight ahead (no turning)
            twist.linear.x = 0.0  # move forward
            twist.angular.z = 3.14
            print("[PHASE 1] Driving straight to explore")
        else:
            # 🔁 Phase 2: Begin controlled turning using action
            angle = (action - 320) / 320 * (np.pi / 2)
            twist.linear.x = 0.1  # still move forward slightly
            twist.angular.z = float(angle)*10

        self.redball.twist_publisher.publish(twist)

        # Allow time for movement and sensor feedback
        for _ in range(30):
            rclpy.spin_once(self.redball, timeout_sec=0.1)

        # Get observation and reward
        observation = self.redball.redball_position or 320
        reward = self.reward(observation)
        terminated = self.step_count >= 100
        truncated = False
        info = {}

        return observation, reward, terminated, truncated, info

    def reward(self, redball_position):
        if redball_position is None:
            return -1.0  # ❌ Ball not seen — bad
        distance = abs(redball_position - 320)
        return -distance / 320  # Normalized from -1.0 (worst) to 0.0 (best)
        # redball_position = 320 → reward = 0.0 ✅

        # redball_position = 160 → reward = -0.5

        # redball_position = 0 or 640 → reward = -1.0 ❌

        # None → reward = -1.0 ❌

    def render(self):
        pass

    def close(self):
        self.redball.destroy_node()
        rclpy.shutdown()


class RedBall(Node):
  """
  A Node to analyse red balls in images and publish the results
  """
  def __init__(self):
    super().__init__('redball')
    # A converter between ROS and OpenCV images
    self.br = CvBridge()
    self.subscription = self.create_subscription(
      Image,
      '/custom_ns/camera1/image_raw',
      self.listener_callback,
      10)
    self.target_publisher = self.create_publisher(Image, 'target_redball', 10)
    self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
    self.create3_is_stopped = True  # default
    self.stop_sub = self.create_subscription(
        StopStatus,
        '/stop_status',
        self.stop_callback,
        10
    )
    self.redball_position = 320  # default to center

  def stop_callback(self, msg):
    self.create3_is_stopped = msg.is_stopped

  def listener_callback(self, msg):
    # print("[CALLBACK] Image received!")

    # Convert ROS Image to OpenCV
    frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Optional: Save image for debugging
    cv2.imwrite("frame_debug.jpg", frame)  # View with `xdg-open frame_debug.jpg`

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # ✅ Correct red detection with wraparound (two ranges)
# Narrower range for sharper red (you can tune further!)
    lower_red1 = (0, 150, 150)
    upper_red1 = (5, 255, 255)

    lower_red2 = (170, 150, 150)
    upper_red2 = (180, 255, 255)

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Blur & clean the mask
    blurred_mask = cv2.GaussianBlur(mask, (9, 9), 3, 3)
    eroded = cv2.erode(blurred_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
    dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8)))

    # Detect circles
    circles = cv2.HoughCircles(dilated, cv2.HOUGH_GRADIENT, 1, 150,
                                param1=100, param2=20, minRadius=2, maxRadius=2000)

    if circles is not None:
        x_center = int(circles[0][0][0])
        self.redball_position = x_center
        print(f"[DETECTED] Red ball at x={x_center}")

        # Optional: draw and publish for visualization
        output = cv2.circle(frame, (x_center, int(circles[0][0][1])), int(circles[0][0][2]), (0, 255, 0), 3)
        self.target_publisher.publish(self.br.cv2_to_imgmsg(output, encoding="bgr8"))
    else:
        print("[DETECTED] No red ball")


    frame = self.br.imgmsg_to_cv2(msg)

    # convert image to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # red ball mask
    lower_red1 = (0, 100, 100)
    upper_red1 = (10, 255, 255)

    lower_red2 = (160, 100, 100)
    upper_red2 = (180, 255, 255)
    # blur and morphology

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # detect circles
    circles = cv2.HoughCircles(dilated, cv2.HOUGH_GRADIENT, 1, 150, param1=100, param2=20, minRadius=2, maxRadius=2000)
    if circles is not None:
        x_center = int(circles[0][0][0])
        self.redball_position = x_center  # ✅ This must be set!
        print(f"[DETECTED] Red ball at x={x_center}")
    else:
        print("[DETECTED] No ball")
