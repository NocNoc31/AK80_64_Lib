import logging

def setup_logger(level: str = "INFO", log_file: str = None):
    """Cấu hình logging."""
    logging.basicConfig(level=getattr(logging, level.upper()), format='%(asctime)s - %(levelname)s - %(message)s')
    if log_file:
        handler = logging.FileHandler(log_file)
        handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
        logging.getLogger().addHandler(handler)