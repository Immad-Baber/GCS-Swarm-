import logging
import os
from datetime import datetime


class ConsoleFilter(logging.Filter):
    def filter(self, record):
        # Only allow INFO and higher (no DEBUG)
        return record.levelno >= logging.INFO


def setup_logger():
    """
    Sets up the logger to write logs to a timestamped file and to the console.
    Returns: str: The full path to the log file.
    """
    # Remove existing handlers
    for handler in logging.root.handlers[:]:
        logging.root.removeHandler(handler)

    # Determine logs directory
    parent_dir = os.path.abspath(os.path.join(os.getcwd(), os.pardir))
    logs_dir = os.path.join(parent_dir, "logs")
    os.makedirs(logs_dir, exist_ok=True)

    # Generate timestamped log filename
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")
    log_filename = os.path.join(logs_dir, f"{timestamp}.txt")

    # File handler – logs everything
    file_handler = logging.FileHandler(log_filename, mode='w', encoding='utf-8')
    file_handler.setLevel(logging.DEBUG)

    # Console handler – logs only INFO and above
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_handler.addFilter(ConsoleFilter())

    # Common formatter
    formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)

    # Root logger setup
    logging.basicConfig(level=logging.DEBUG, handlers=[file_handler, console_handler])

    logging.info("Logging initialized → %s", log_filename)
    return log_filename
