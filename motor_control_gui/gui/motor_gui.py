import tkinter as tk
from tkinter import ttk, messagebox, StringVar, filedialog
import threading
from .styles import configure_styles
from ..utils.config_loader import load_config
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class MotorGUI(tk.Tk):
    def __init__(self, motors_config, param_client, status_subscriber):
        super().__init__()
        self.motors_config = motors_config  # Danh sách cấu hình motor: [{"name": "motor1", "id": "0x68", "range": [0, 90], ...}]
        self.param_client = param_client
        self.status_subscriber = status_subscriber
        self.title("Giao diện điều khiển Motor")
        self.geometry("1000x700")
        self.configure(bg='#E6F3FA')
        self.style = ttk.Style()
        configure_styles(self.style)
        self.entries = {}
        self.status_labels = {}
        self.reached_status = {motor["name"]: False for motor in motors_config}
        self.reached_event = threading.Event()
        self.create_widgets()
        self.subscribe_to_status()

    def create_widgets(self):
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=0)

        for idx, motor in enumerate(self.motors_config):
            name = motor["name"]
            frame = ttk.LabelFrame(self, text=f"{name.capitalize()} (ID: {motor['id']})", padding=15)
            frame.grid(row=0, column=idx, padx=20, pady=10, sticky="nsew")
            for i in range(4):
                frame.grid_columnconfigure(i, weight=1)

            ttk.Label(frame, text="Vị trí mục tiêu (độ, cách nhau bởi dấu phẩy):").grid(row=0, column=0, sticky="w", pady=5, columnspan=2)
            self.entries.setdefault(name, {})['targets_str'] = StringVar()
            ttk.Entry(frame, textvariable=self.entries[name]['targets_str'], width=25).grid(row=0, column=2, padx=5, pady=5, columnspan=2)

            ttk.Label(frame, text="Tốc độ (pps):").grid(row=1, column=0, sticky="w", pady=5)
            self.entries[name]['speed'] = ttk.Entry(frame, width=12)
            self.entries[name]['speed'].grid(row=1, column=1, padx=5, pady=5)
            self.entries[name]['speed'].insert(0, str(motor.get("default_speed", 10000)))

            ttk.Label(frame, text="Gia tốc (pps²):").grid(row=2, column=0, sticky="w", pady=5)
            self.entries[name]['accel'] = ttk.Entry(frame, width=12)
            self.entries[name]['accel'].grid(row=2, column=1, padx=5, pady=5)
            self.entries[name]['accel'].insert(0, str(motor.get("default_accel", 1000)))

            ttk.Button(frame, text="+10 độ", command=lambda m=name: self.adjust_target(m, 10)).grid(row=1, column=2, pady=5, sticky="ew")
            ttk.Button(frame, text="-10 độ", command=lambda m=name: self.adjust_target(m, -10)).grid(row=1, column=3, pady=5, sticky="ew")

            ttk.Button(frame, text="Gửi lệnh", command=lambda m=name: self.send_command(m, self.get_parameters(m))).grid(row=3, column=0, columnspan=4, pady=10)

            ttk.Label(frame, text="Vị trí (độ):").grid(row=4, column=0, sticky="w", pady=5)
            self.status_labels.setdefault(name, {})['position'] = ttk.Label(frame, text="0.0")
            self.status_labels[name]['position'].grid(row=4, column=1, columnspan=3, sticky="w", pady=5)

            ttk.Label(frame, text="Tốc độ (rpm):").grid(row=5, column=0, sticky="w", pady=5)
            self.status_labels[name]['velocity'] = ttk.Label(frame, text="0.0")
            self.status_labels[name]['velocity'].grid(row=5, column=1, columnspan=3, sticky="w", pady=5)

            ttk.Label(frame, text="Đạt mục tiêu:").grid(row=6, column=0, sticky="w", pady=5)
            self.status_labels[name]['reached'] = ttk.Label(frame, text="False")
            self.status_labels[name]['reached'].grid(row=6, column=1, columnspan=3, sticky="w", pady=5)

        ttk.Button(self, text="Tải YAML", command=self.load_yaml_params).grid(row=1, column=0, pady=10, sticky="ew")
        ttk.Button(self, text="Dừng tất cả", style='Stop.TButton', command=self.stop_all).grid(row=1, column=1, pady=10, sticky="ew")

    def load_yaml_params(self):
        file_path = filedialog.askopenfilename(filetypes=[("YAML files", "*.yaml *.yml")])
        if file_path:
            try:
                config = load_config(file_path)
                for motor in self.motors_config:
                    name = motor["name"]
                    if name in config:
                        params = config[name]
                        if 'target_positions' in params:
                            targets = [float(t) for t in ([params['target_positions']] if isinstance(params['target_positions'], (int, float)) else params['target_positions'])]
                            motor_config = next(m for m in self.motors_config if m["name"] == name)
                            if any(t < motor_config["range"][0] or t > motor_config["range"][1] for t in targets):
                                raise ValueError(f"Vị trí mục tiêu {targets} ngoài phạm vi {motor_config['range']} cho {name}")
                            self.entries[name]['targets_str'].set(','.join(map(str, targets)))
                        if 'speed' in params:
                            self.entries[name]['speed'].delete(0, tk.END)
                            self.entries[name]['speed'].insert(0, str(params['speed']))
                        if 'accel' in params:
                            self.entries[name]['accel'].delete(0, tk.END)
                            self.entries[name]['accel'].insert(0, str(params['accel']))
                        params = self.get_parameters(name)
                        if params:
                            self.param_client.set_parameters(name, params)
                self.after(0, lambda: messagebox.showinfo("Thành công", f"Đã tải tham số từ {file_path}"))
            except Exception as e:
                self.after(0, lambda: messagebox.showerror("Lỗi", f"Không thể tải YAML: {e}"))

    def stop_all(self):
        self.reached_status = {motor["name"]: False for motor in self.motors_config}
        self.reached_event.clear()
        for motor in self.motors_config:
            name = motor["name"]
            params = [
                Parameter(name="target_positions", value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=[0.0])),
                Parameter(name="speed", value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=10000)),
                Parameter(name="accel", value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=1000))
            ]
            self.param_client.set_parameters(name, params)

        def check_reached():
            import time
            timeout = 30.0
            start_time = time.time()
            while time.time() - start_time < timeout:
                if self.reached_event.wait(timeout=0.1):
                    for motor in self.motors_config:
                        params = [Parameter(name="speed", value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=0))]
                        self.param_client.set_parameters(motor["name"], params)
                    self.after(0, lambda: messagebox.showinfo("Thành công", "Đã về HOME, động cơ tắt"))
                    return
                time.sleep(0.1)
            failed_motors = [m for m, reached in self.reached_status.items() if not reached]
            self.after(0, lambda: messagebox.showerror("Lỗi", f"Hết thời gian: {', '.join(failed_motors)} không về được HOME"))

        threading.Thread(target=check_reached, daemon=True).start()

    def adjust_target(self, motor, step):
        current_targets_str = self.entries[motor]['targets_str'].get()
        current_targets = [float(t.strip()) for t in current_targets_str.split(',') if t.strip()] or [0.0]
        new_target = current_targets[-1] + step
        motor_config = next(m for m in self.motors_config if m["name"] == motor)
        if motor_config["range"][0] <= new_target <= motor_config["range"][1]:
            self.entries[motor]['targets_str'].set(str(new_target))
            params = self._create_target_parameter(motor, [new_target])
            if params:
                self.param_client.set_parameters(motor, params)
        else:
            self.after(0, lambda: messagebox.showwarning("Cảnh báo", f"Vị trí {new_target} ngoài phạm vi {motor_config['range']}"))

    def _create_target_parameter(self, motor, targets):
        if targets:
            return [Parameter(name="target_positions", value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=targets))]
        return None

    def get_parameters(self, motor):
        params = []
        targets_str = self.entries[motor]['targets_str'].get().strip()
        if targets_str:
            try:
                targets = [float(t.strip()) for t in targets_str.split(',')]
                param_val = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=targets)
                params.append(Parameter(name="target_positions", value=param_val))
            except ValueError:
                self.after(0, lambda: messagebox.showerror("Lỗi", f"Định dạng vị trí mục tiêu không hợp lệ cho {motor}"))
                return None

        speed = self.entries[motor]['speed'].get().strip()
        try:
            speed_val = int(speed)
            param_val = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=speed_val)
            params.append(Parameter(name="speed", value=param_val))
        except ValueError:
            self.after(0, lambda: messagebox.showerror("Lỗi", f"Tốc độ không hợp lệ cho {motor}"))
            return None

        accel = self.entries[motor]['accel'].get().strip()
        try:
            accel_val = int(accel)
            param_val = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=accel_val)
            params.append(Parameter(name="accel", value=param_val))
        except ValueError:
            self.after(0, lambda: messagebox.showerror("Lỗi", f"Gia tốc không hợp lệ cho {motor}"))
            return None

        return params

    def send_command(self, motor, params):
        if params:
            def safe_send():
                success = self.param_client.set_parameters(motor, params)
                self.after(0, lambda: messagebox.showinfo("Thành công", f"Đã gửi tham số đến {motor}") if success else
                           messagebox.showerror("Lỗi", f"Không thể gửi tham số đến {motor}"))
            self.after(0, safe_send)

    def update_status(self, topic, value):
        def safe_update():
            motor, param = topic.split('_', 1)
            if param in self.status_labels[motor]:
                self.status_labels[motor][param].config(
                    text=f"{value:.2f}" if isinstance(value, float) else str(value)
                )
            if param == 'reached':
                self.reached_status[motor] = value
                if all(self.reached_status.values()):
                    self.reached_event.set()
        self.after(0, safe_update)

    def subscribe_to_status(self):
        for motor in self.motors_config:
            self.status_subscriber.subscribe(motor["name"], self.update_status)