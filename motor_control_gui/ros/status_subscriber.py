from rclpy.node import Node
from std_msgs.msg import Float32, Bool

class StatusSubscriber:
    def __init__(self, node: Node):
        """Khởi tạo subscriber cho trạng thái."""
        self.node = node
        self.subscribers = {}

    def subscribe(self, node_name: str, callback):
        """Đăng ký lắng nghe trạng thái từ các topic."""
        if node_name not in self.subscribers:
            base_topic = f"/{node_name}_control_node"
            self.subscribers[f"{node_name}_position"] = self.node.create_subscription(
                Float32, f"{base_topic}_position",
                lambda msg: callback(f"{node_name}_position", msg.data), 10
            )
            self.subscribers[f"{node_name}_velocity"] = self.node.create_subscription(
                Float32, f"{base_topic}_vel_actual",
                lambda msg: callback(f"{node_name}_velocity", msg.data), 10
            )
            self.subscribers[f"{node_name}_reached"] = self.node.create_subscription(
                Bool, f"{base_topic}_target_reached",
                lambda msg: callback(f"{node_name}_reached", msg.data), 10
            )