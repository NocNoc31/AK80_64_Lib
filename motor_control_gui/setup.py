from setuptools import setup, find_packages

setup(
    name="motor_control_gui",
    version="0.1.0",
    packages=find_packages(),
    install_requires=["rclpy", "pyyaml"],
    author="Your Name",
    description="A reusable GUI library for motor control with ROS 2 integration",
    python_requires=">=3.8",
)