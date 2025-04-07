from setuptools import setup

setup(
    name="aisd_examples",
    version="0.1",
    install_requires=["gymnasium", "numpy", "rclpy", "sensor_msgs", "irobot_create_msgs"],
    packages=["aisd_examples", "aisd_examples.envs"],
    package_dir={"aisd_examples": "aisd_examples"},
)
