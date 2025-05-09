from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import threading
from .gui.motor_gui import MotorGUI
from .ros.param_client import ParameterClient
from .ros.status_subscriber import StatusSubscriber
from .utils.logging import setup_logger
import rclpy

class ControlNode(Node):
    def __init__(self, node_name: str, motors_config: list, service_configs: dict):
        """Khởi tạo node điều khiển với GUI."""
        super().__init__(node_name)
        setup_logger()
        self.param_client = ParameterClient(self, service_configs)
        self.status_subscriber = StatusSubscriber(self)
        self.gui = MotorGUI(motors_config, self.param_client, self.status_subscriber)
        try:
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self)
            self.spin_method = "executor"
            self.get_logger().info("Sử dụng SingleThreadedExecutor để quay")
        except Exception as e:
            self.get_logger().error(f"Không thể khởi tạo Executor: {e}. Quay lại rclpy.spin")
            self.executor = None
            self.spin_method = "spin"
        self.spin_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.spin_thread.start()

    def spin_ros(self):
        """Quay node ROS."""
        try:
            if self.spin_method == "executor" and self.executor is not None:
                self.executor.spin()
            else:
                rclpy.spin(self)
        except KeyboardInterrupt:
            pass

    def destroy_node(self):
        """Hủy node."""
        super().destroy_node()
        if self.executor is not None:
            self.executor.shutdown()