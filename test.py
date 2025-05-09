import rclpy
from motor_control_gui import ControlNode, setup_logger
from std_msgs.msg import Float32, Bool

def main():
    rclpy.init()
    setup_logger(level="INFO")

    motors_config = [
        {"name": "motor1", "id": "0x68", "range": [0, 90], "default_speed": 10000, "default_accel": 1000},
        {"name": "motor2", "id": "0x69", "range": [0, 90], "default_speed": 10000, "default_accel": 1000},
    ]
    service_configs = {
        "motor1": "/motor1_control_node/set_parameters",
        "motor2": "/motor2_control_node/set_parameters",
    }

    node = ControlNode("motor_gui_node", motors_config, service_configs)
    node.gui.mainloop()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()