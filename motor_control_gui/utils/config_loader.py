import yaml

def load_config(file_path: str, expected_keys: list = None) -> dict:
    """Tải cấu hình từ tệp YAML."""
    try:
        with open(file_path, 'r') as file:
            config = yaml.safe_load(file)
        if expected_keys:
            for key in expected_keys:
                if key not in config:
                    raise ValueError(f"Thiếu khóa {key} trong cấu hình")
        return config
    except Exception as e:
        raise RuntimeError(f"Không thể tải YAML: {e}")