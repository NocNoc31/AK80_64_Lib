import queue
import threading
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters

class ParameterClient:
    def __init__(self, node: Node, service_configs: dict):
        """Khởi tạo client cho dịch vụ SetParameters.
        service_configs: {motor_name: service_name}
        """
        self.node = node
        self.clients = {name: node.create_client(SetParameters, srv) for name, srv in service_configs.items()}
        self.queues = {name: queue.Queue() for name in service_configs}
        self.result_queue = queue.Queue()
        self.threads = {name: threading.Thread(target=lambda n=name: self.process_commands(n), daemon=True) for name in service_configs}
        for thread in self.threads.values():
            thread.start()

    def set_parameters(self, node_name: str, parameters: list) -> bool:
        """Gửi tham số đến node được chỉ định."""
        if node_name not in self.clients:
            self.node.get_logger().error(f"Tên node không hợp lệ: {node_name}")
            return False
        self.queues[node_name].put((node_name, parameters))
        try:
            for attempt in range(20):
                try:
                    result_node, result = self.result_queue.get(timeout=1.5)
                    if result_node == node_name:
                        return result
                except queue.Empty:
                    self.node.get_logger().info(f"Thử lại {attempt + 1}/20 cho {node_name}: Hàng đợi kết quả trống")
            self.node.get_logger().error(f"Hết thời gian chờ kết quả cho {node_name}")
            return False
        except Exception as e:
            self.node.get_logger().error(f"Lỗi không mong muốn khi chờ kết quả cho {node_name}: {e}")
            return False

    def process_commands(self, node_name: str):
        """Xử lý lệnh từ hàng đợi của node."""
        while True:
            try:
                _, parameters = self.queues[node_name].get()
                client = self.clients[node_name]
                request = SetParameters.Request()
                request.parameters = parameters
                future = client.call_async(request)
                future.add_done_callback(
                    lambda f: self.result_queue.put(
                        (node_name, all(result.successful for result in f.result().results) if f.result() else False)
                    )
                )
                self.queues[node_name].task_done()
            except Exception as e:
                self.node.get_logger().error(f"Lỗi xử lý lệnh cho {node_name}: {e}")
                self.result_queue.put((node_name, False))