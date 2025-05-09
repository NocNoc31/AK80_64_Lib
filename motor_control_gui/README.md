Motor Control GUI
A reusable Python library for controlling and monitoring motors via a graphical user interface (GUI) integrated with ROS 2. This library provides an intuitive interface to send motor commands, monitor real-time status, and load parameters from YAML files, making it ideal for robotics and automation projects.
Table of Contents

Features
Installation
Usage
Configuration
Goals
Requirements
Contributing
License

Features

Graphical User Interface (GUI):
Displays control panels for each motor, showing ID, target positions, speed, and acceleration.
Allows incremental position adjustments (+10°/-10°) within configured ranges.
Provides buttons to send commands, stop all motors, and load parameters from YAML files.


Real-Time Status Monitoring:
Updates position (degrees), velocity (RPM), and target-reached status via ROS 2 topics.
Displays status in the GUI with formatted numeric or boolean values.


ROS 2 Integration:
Sends motor parameters (target positions, speed, acceleration) via SetParameters service.
Subscribes to ROS 2 topics for real-time motor status updates.


Parameter Loading:
Loads motor parameters from YAML files, supporting multiple target positions, speed, and acceleration.


Stop All Functionality:
Commands all motors to return to HOME position (0.0°) and disables them after confirmation.
Includes timeout handling with error notifications.


Modular Design:
Organized into sub-packages (gui, ros, utils) for easy maintenance and reuse.
Thread-safe implementation using queues and events to prevent conflicts.



Installation

Clone or Download the Repository:
git clone https://github.com/yourusername/motor_control_gui.git
cd motor_control_gui


Install Dependencies:Ensure you have Python 3.8+ and ROS 2 (Humble or later) installed. Then install the library:
pip install .

This installs required packages: rclpy and pyyaml.

Set Up ROS 2 Environment:Source your ROS 2 installation before running the library:
source /opt/ros/<distro>/setup.bash



Usage
The library provides a ControlNode class to integrate the GUI with ROS 2. Below is an example of how to use it:
Example Script (test.py)
import rclpy
from motor_control_gui import ControlNode, setup_logger

def main():
    rclpy.init()
    setup_logger(level="INFO")
    
    # Define motor configurations
    motors_config = [
        {"name": "motor1", "id": "0x68", "range": [0, 90], "default_speed": 10000, "default_accel": 1000},
        {"name": "motor2", "id": "0x69", "range": [0, 90], "default_speed": 10000, "default_accel": 1000},
    ]
    
    # Define ROS 2 service names
    service_configs = {
        "motor1": "/motor1_control_node/set_parameters",
        "motor2": "/motor2_control_node/set_parameters",
    }
    
    # Create and run the control node
    node = ControlNode("motor_gui_node", motors_config, service_configs)
    node.gui.mainloop()
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

Running the Example

Ensure ROS 2 nodes for motor control are running (publishing topics and providing SetParameters services).
Run the script:source /opt/ros/<distro>/setup.bash
python3 test.py


A GUI window will appear, allowing you to:
Enter target positions (comma-separated degrees).
Adjust positions with +10°/-10° buttons.
Set speed (pulses per second) and acceleration (pulses per second²).
Send commands to motors.
Load parameters from a YAML file.
Stop all motors and return to HOME.



Example YAML Configuration (config.yaml)
motor1:
  target_positions: [0.0, 40.0, 0.0, 60.0, 20.0]
  speed: 10000
  accel: 1000
motor2:
  target_positions: [0.0, 60.0, 0.0]
  speed: 12000
  accel: 1500


Use the "Load YAML" button to load this file into the GUI.

Configuration

Motors Config: A list of dictionaries specifying motor details:
name: Unique motor identifier (e.g., "motor1").
id: Motor ID (e.g., "0x68").
range: Position range in degrees (e.g., [0, 90]).
default_speed: Default speed in pulses per second (e.g., 10000).
default_accel: Default acceleration in pulses per second² (e.g., 1000).


Service Configs: A dictionary mapping motor names to ROS 2 service names for SetParameters.
Logging: Configure logging level (DEBUG, INFO, etc.) using setup_logger(level="INFO").

Goals

Ease of Use: Provide an intuitive GUI for controlling motors without deep ROS 2 knowledge.
Reusability: Modular design allows integration into various ROS 2 projects, not limited to motors.
Reliability: Thread-safe implementation and error handling ensure robust operation.
Extensibility: Easily add new motors or features by updating configurations or extending modules.

Requirements

Python: 3.8 or higher
ROS 2: Humble or later
Dependencies:
rclpy: ROS 2 Python client library
pyyaml: For parsing YAML configuration files


Operating System: Tested on Ubuntu (22.04 or later) with ROS 2 installed.









Giao Diện Điều Khiển Motor
Một thư viện Python có thể tái sử dụng để điều khiển và giám sát motor thông qua giao diện người dùng đồ họa (GUI) tích hợp với ROS 2. Thư viện cung cấp giao diện trực quan để gửi lệnh motor, giám sát trạng thái thời gian thực và nạp tham số từ file YAML, lý tưởng cho các dự án robot và tự động hóa.
Mục Lục

Tính Năng
Cài Đặt
Cách Sử Dụng
Cấu Hình
Mục Tiêu
Yêu Cầu
Đóng Góp
Giấy Phép

Tính Năng

Giao Diện Người Dùng Đồ Họa (GUI):
Hiển thị bảng điều khiển cho từng motor, bao gồm ID, vị trí mục tiêu, tốc độ và gia tốc.
Cho phép điều chỉnh vị trí tăng dần (+10°/-10°) trong phạm vi được cấu hình.
Cung cấp các nút để gửi lệnh, dừng tất cả motor và nạp tham số từ file YAML.


Giám Sát Trạng Thái Thời Gian Thực:
Cập nhật vị trí (độ), tốc độ (RPM) và trạng thái đạt mục tiêu thông qua các topic ROS 2.
Hiển thị trạng thái trên GUI với định dạng số hoặc boolean.


Tích Hợp ROS 2:
Gửi tham số motor (vị trí mục tiêu, tốc độ, gia tốc) qua dịch vụ SetParameters.
Đăng ký topic ROS 2 để cập nhật trạng thái motor thời gian thực.


Nạp Tham Số:
Nạp tham số motor từ file YAML, hỗ trợ nhiều vị trí mục tiêu, tốc độ và gia tốc.


Chức Năng Dừng Tất Cả:
Ra lệnh cho tất cả motor về vị trí HOME (0.0°) và tắt sau khi xác nhận.
Xử lý thời gian chờ với thông báo lỗi nếu có sự cố.


Thiết Kế Mô-đun:
Được tổ chức thành các gói con (gui, ros, utils) để dễ bảo trì và tái sử dụng.
Triển khai an toàn luồng sử dụng hàng đợi và sự kiện để tránh xung đột.



Cài Đặt

Tải hoặc Sao Chép Repository:
git clone https://github.com/yourusername/motor_control_gui.git
cd motor_control_gui


Cài Đặt Phụ Thuộc:Đảm bảo bạn có Python 3.8+ và ROS 2 (Humble trở lên). Sau đó cài đặt thư viện:
pip install .

Lệnh này sẽ cài đặt các gói yêu cầu: rclpy và pyyaml.

Thiết Lập Môi Trường ROS 2:Nạp môi trường ROS 2 trước khi chạy thư viện:
source /opt/ros/<distro>/setup.bash



Cách Sử Dụng
Thư viện cung cấp lớp ControlNode để tích hợp GUI với ROS 2. Dưới đây là ví dụ cách sử dụng:
Ví Dụ Script (test.py)
import rclpy
from motor_control_gui import ControlNode, setup_logger

def main():
    rclpy.init()
    setup_logger(level="INFO")
    
    # Cấu hình motor
    motors_config = [
        {"name": "motor1", "id": "0x68", "range": [0, 90], "default_speed": 10000, "default_accel": 1000},
        {"name": "motor2", "id": "0x69", "range": [0, 90], "default_speed": 10000, "default_accel": 1000},
    ]
    
    # Cấu hình dịch vụ ROS 2
    service_configs = {
        "motor1": "/motor1_control_node/set_parameters",
        "motor2": "/motor2_control_node/set_parameters",
    }
    
    # Tạo và chạy node điều khiển
    node = ControlNode("motor_gui_node", motors_config, service_configs)
    node.gui.mainloop()
    
    # Dọn dẹp
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

Chạy Ví Dụ

Đảm bảo các node ROS 2 cho điều khiển motor đang chạy (xuất bản topic và cung cấp dịch vụ SetParameters).
Chạy script:source /opt/ros/<distro>/setup.bash
python3 test.py


Một cửa sổ GUI sẽ xuất hiện, cho phép bạn:
Nhập vị trí mục tiêu (các độ phân cách bằng dấu phẩy).
Điều chỉnh vị trí với nút +10°/-10°.
Đặt tốc độ (pulse/giây) và gia tốc (pulse/giây²).
Gửi lệnh đến motor.
Nạp tham số từ file YAML.
Dừng tất cả motor và đưa về vị trí HOME.



Ví Dụ Cấu Hình YAML (config.yaml)
motor1:
  target_positions: [0.0, 40.0, 0.0, 60.0, 20.0]
  speed: 10000
  accel: 1000
motor2:
  target_positions: [0.0, 60.0, 0.0]
  speed: 12000
  accel: 1500


Sử dụng nút "Nạp YAML" để nạp file này vào GUI.

Cấu Hình

Cấu Hình Motor: Danh sách các từ điển chỉ định chi tiết motor:
name: Tên định danh duy nhất của motor (ví dụ: "motor1").
id: ID motor (ví dụ: "0x68").
range: Phạm vi vị trí tính bằng độ (ví dụ: [0, 90]).
default_speed: Tốc độ mặc định (pulse/giây, ví dụ: 10000).
default_accel: Gia tốc mặc định (pulse/giây², ví dụ: 1000).


Cấu Hình Dịch Vụ: Từ điển ánh xạ tên motor với tên dịch vụ ROS 2 cho SetParameters.
Ghi Log: Cấu hình mức độ ghi log (DEBUG, INFO, v.v.) bằng setup_logger(level="INFO").

Mục Tiêu

Dễ Sử Dụng: Cung cấp GUI trực quan để điều khiển motor mà không cần hiểu sâu về ROS 2.
Tái Sử Dụng: Thiết kế mô-đun cho phép tích hợp vào nhiều dự án ROS 2, không giới hạn ở motor.
Đáng Tin Cậy: Triển khai an toàn luồng và xử lý lỗi đảm bảo hoạt động ổn định.
Dễ Mở Rộng: Dễ dàng thêm motor mới hoặc tính năng bằng cách cập nhật cấu hình hoặc mở rộng module.

Yêu Cầu

Python: 3.8 trở lên
ROS 2: Humble trở lên
Phụ Thuộc:
rclpy: Thư viện client Python cho ROS 2
pyyaml: Để phân tích file cấu hình YAML


Hệ Điều Hành: Đã thử nghiệm trên Ubuntu (22.04 trở lên) với ROS 2 cài đặt.


