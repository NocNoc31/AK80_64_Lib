
import queue
import threading
from time import time
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters

class ParameterClient:
    def __init__(self, node: Node, service_configs: dict):
        """Khởi tạo client cho dịch vụ SetParameters.
        service_configs: {motor_name: service_name}
        """
        self.node = node
        self.clients = {name: node.create_client(SetParameters, srv) for name, srv in service_configs.items()}
        self.queues = {name: queue.Queue(maxsize=100) for name in service_configs}
        self.result_queues = {name: queue.Queue(maxsize=100) for name in service_configs}
        self.last_log_time = {name: 0 for name in service_configs}
        self.threads = {name: threading.Thread(target=lambda n=name: self.process_commands(n), daemon=True) for name in service_configs}
        
        # Khởi tạo hai giá trị True trong hàng đợi kết quả cho mỗi động cơ
        for name in service_configs:
            for _ in range(2):
                self.result_queues[name].put((name, True))
        for thread in self.threads.values():
            thread.start()
        # Khởi động giám sát luồng
        self.node.create_timer(1.0, self.monitor_threads)

    def set_parameters(self, node_name: str, parameters: list) -> bool:
        """Gửi tham số đến node được chỉ định."""
        if node_name not in self.clients:
            self.node.get_logger().error(f"Tên node không hợp lệ: {node_name}")
            return False
        client = self.clients[node_name]
        if not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error(f"Dịch vụ {client.srv_name} không khả dụng")
            return False
        try:
            self.queues[node_name].put((node_name, parameters), timeout=1.0)
            result_node, result = self.result_queues[node_name].get(timeout=1.0)
            if result_node == node_name:
                return result
            return False
        except queue.Full:
            self.node.get_logger().error(f"Hàng đợi lệnh đầy cho {node_name}")
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
                    lambda f: self.result_queues[node_name].put(
                        (node_name, all(result.successful for result in f.result().results) if f.result() else False),
                        timeout=1.0
                    )
                )
                self.queues[node_name].task_done()
            except queue.Full:
                if time() - self.last_log_time[node_name] > 1.0:
                    self.node.get_logger().error(f"Hàng đợi kết quả đầy cho {node_name}")
                    self.last_log_time[node_name] = time()
                self.result_queues[node_name].put((node_name, False))
            except Exception as e:
                if time() - self.last_log_time[node_name] > 1.0:
                    self.node.get_logger().error(f"Lỗi xử lý lệnh cho {node_name}: {e}")
                    self.last_log_time[node_name] = time()
                self.result_queues[node_name].put((node_name, False))

    def monitor_threads(self):
        """Giám sát và khởi động lại các luồng bị dừng."""
        for name, thread in self.threads.items():
            if not thread.is_alive():
                self.node.get_logger().error(f"Luồng cho {name} đã dừng. Khởi động lại...")
                self.threads[name] = threading.Thread(target=lambda n=name: self.process_commands(n), daemon=True)
                self.threads[name].start()