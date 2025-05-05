"""
Logging utility for SenseBridge.
Sets up logging configuration for the application.
"""

import os
import logging
import logging.handlers
import time
from pathlib import Path


def setup_logging(log_level=logging.INFO):
    """Set up logging configuration.

    Args:
        log_level: Logging level (default: INFO)
    """
    # Create logs directory if it doesn't exist
    log_dir = Path("logs")
    os.makedirs(log_dir, exist_ok=True)

    # Create timestamp for log file
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    log_file = log_dir / f"senseBridge_{timestamp}.log"

    # Configure root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(log_level)

    # Create console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(log_level)
    console_format = logging.Formatter(
        '%(asctime)s - %(levelname)s - %(name)s - %(message)s',
        datefmt='%H:%M:%S'
    )
    console_handler.setFormatter(console_format)

    # Create file handler
    file_handler = logging.handlers.RotatingFileHandler(
        log_file,
        maxBytes=10 * 1024 * 1024,  # 10 MB
        backupCount=5
    )
    file_handler.setLevel(log_level)
    file_format = logging.Formatter(
        '%(asctime)s - %(levelname)s - %(name)s - %(pathname)s:%(lineno)d - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    file_handler.setFormatter(file_format)

    # Add handlers to root logger
    root_logger.addHandler(console_handler)
    root_logger.addHandler(file_handler)

    # Create custom loggers for main components
    loggers = [
        "audio", "speech", "notification", "models",
        "hardware", "gui", "utils"
    ]

    for logger_name in loggers:
        module_logger = logging.getLogger(f"src.{logger_name}")
        module_logger.setLevel(log_level)
        # No need to add handlers as they inherit from root logger

    logging.info("Logging initialized")
    return logging.getLogger("src")