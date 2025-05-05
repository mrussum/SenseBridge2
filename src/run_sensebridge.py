#!/usr/bin/env python3
# run_sensebridge.py - Simplified launcher for SenseBridge

import os
import sys
import subprocess
import argparse
import logging
import time
from pathlib import Path

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("SenseBridge")


def check_environment():
    """Check if the environment is properly set up for SenseBridge."""
    # Check for Python version
    python_version = sys.version_info
    if python_version.major < 3 or (python_version.major == 3 and python_version.minor < 8):
        logger.error("Python 3.8 or newer is required.")
        return False

    # Check for required directories
    required_dirs = ["src", "config", "models"]
    for directory in required_dirs:
        if not os.path.exists(directory):
            logger.error(f"Required directory '{directory}' not found.")
            return False

    # Check for virtual environment
    if not hasattr(sys, 'real_prefix') and not (hasattr(sys, 'base_prefix') and sys.base_prefix != sys.prefix):
        logger.warning("Not running in a virtual environment. It's recommended to use the .venv environment.")
        logger.warning("Try running: source .venv/bin/activate")

    # Check for configuration files
    config_files = ["config/device_config.json", "config/sound_events.json", "config/user_prefs.json"]
    for config_file in config_files:
        if not os.path.exists(config_file):
            logger.error(f"Required configuration file '{config_file}' not found.")
            return False

    # Check for YAMNet model
    model_file = "models/yamnet_model/yamnet.tflite"
    if not os.path.exists(model_file):
        logger.warning(f"YAMNet model file '{model_file}' not found. Will use fallback classification.")

    return True


def setup_environment():
    """Set up the environment for SenseBridge."""
    logger.info("Setting up SenseBridge environment...")

    # Check if setup script exists
    if os.path.exists("setup.sh"):
        # Make it executable if it isn't already
        subprocess.run(["chmod", "+x", "setup.sh"])

        # Run the setup script
        logger.info("Running setup script...")
        result = subprocess.run(["./setup.sh"], capture_output=True, text=True)

        if result.returncode != 0:
            logger.error(f"Setup script failed with error code {result.returncode}")
            logger.error(result.stderr)
            return False

        logger.info("Setup script completed successfully.")
        return True
    else:
        logger.error("Setup script (setup.sh) not found.")
        return False


def create_directory_structure():
    """Create the basic directory structure if it doesn't exist."""
    logger.info("Creating directory structure...")

    # Create required directories
    directories = ["src", "config", "models/yamnet_model", "logs"]
    for directory in directories:
        os.makedirs(directory, exist_ok=True)

    logger.info("Directory structure created.")
    return True


def run_sensebridge(args):
    """Run the SenseBridge application with the given arguments."""
    # Import path magic for direct execution
    sys.path.insert(0, os.getcwd())

    try:
        # Build command
        command = [sys.executable, "-m", "src.main"]

        if args.headless:
            command.append("--headless")

        if args.simulation:
            command.append("--simulation")

        if args.debug:
            command.append("--debug")

        if args.timeout:
            command.append(f"--timeout={args.timeout}")

        # Run SenseBridge
        logger.info(f"Running SenseBridge with command: {' '.join(command)}")
        process = subprocess.run(command)

        return process.returncode == 0

    except Exception as e:
        logger.error(f"Error running SenseBridge: {e}")
        return False


def run_tests(args):
    """Run SenseBridge tests."""
    # Import path magic for direct execution
    sys.path.insert(0, os.getcwd())

    try:
        # Build command
        command = [sys.executable, "test_system.py"]

        if args.all:
            command.append("--all")
        else:
            if args.config:
                command.append("--config")
            if args.audio:
                command.append("--audio")
            if args.recognition:
                command.append("--recognition")
            if args.notification:
                command.append("--notification")
            if args.gui:
                command.append("--gui")

        # Run tests
        logger.info(f"Running tests with command: {' '.join(command)}")
        process = subprocess.run(command)

        return process.returncode == 0

    except Exception as e:
        logger.error(f"Error running tests: {e}")
        return False


def fix_project_structure():
    """Fix the project structure by creating necessary files and directories."""
    logger.info("Fixing project structure...")

    # Check for fix_structure.py
    if os.path.exists("fix_structure.py"):
        # Run fix_structure.py
        logger.info("Running fix_structure.py...")
        subprocess.run([sys.executable, "fix_structure.py"])
    else:
        # Create basic directory structure
        create_directory_structure()

    # Check for fix_imports.py
    if os.path.exists("fix_imports.py"):
        # Run fix_imports.py
        logger.info("Running fix_imports.py...")
        subprocess.run([sys.executable, "fix_imports.py"])

    # Check for create_init_files.py
    if os.path.exists("create_init_files.py"):
        # Run create_init_files.py
        logger.info("Running create_init_files.py...")
        subprocess.run([sys.executable, "create_init_files.py"])

    # Check for create_config_files.py
    if os.path.exists("create_config_files.py"):
        # Run create_config_files.py
        logger.info("Running create_config_files.py...")
        subprocess.run([sys.executable, "create_config_files.py"])

    logger.info("Project structure fixed.")
    return True


def main():
    """Main entry point for the SenseBridge launcher."""
    parser = argparse.ArgumentParser(description="SenseBridge Launcher")

    # Create subparsers for different commands
    subparsers = parser.add_subparsers(dest="command", help="Command to run")

    # Setup command
    setup_parser = subparsers.add_parser("setup", help="Set up the SenseBridge environment")

    # Fix command
    fix_parser = subparsers.add_parser("fix", help="Fix the project structure")

    # Run command
    run_parser = subparsers.add_parser("run", help="Run the SenseBridge application")
    run_parser.add_argument("--headless", action="store_true", help="Run in headless mode (no GUI)")
    run_parser.add_argument("--simulation", action="store_true", help="Run in simulation mode (simulated hardware)")
    run_parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    run_parser.add_argument("--timeout", type=float, help="Exit after specified timeout (in seconds)")

    # Test command
    test_parser = subparsers.add_parser("test", help="Run SenseBridge tests")
    test_parser.add_argument("--all", action="store_true", help="Run all tests")
    test_parser.add_argument("--config", action="store_true", help="Test configuration")
    test_parser.add_argument("--audio", action="store_true", help="Test audio processing")
    test_parser.add_argument("--recognition", action="store_true", help="Test sound recognition")
    test_parser.add_argument("--notification", action="store_true", help="Test notifications")
    test_parser.add_argument("--gui", action="store_true", help="Test GUI")

    # Parse arguments
    args = parser.parse_args()

    # If no command specified, show help
    if args.command is None:
        parser.print_help()
        return 0

    # Handle commands
    if args.command == "setup":
        if setup_environment():
            logger.info("SenseBridge setup completed successfully.")
            return 0
        else:
            logger.error("SenseBridge setup failed.")
            return 1

    elif args.command == "fix":
        if fix_project_structure():
            logger.info("Project structure fixed successfully.")
            return 0
        else:
            logger.error("Failed to fix project structure.")
            return 1

    elif args.command == "run":
        # Check environment before running
        if not check_environment():
            logger.error("Environment check failed. Run 'python run_sensebridge.py setup' to set up the environment.")
            return 1

        # Run SenseBridge
        if run_sensebridge(args):
            return 0
        else:
            return 1

    elif args.command == "test":
        # Check environment before running tests
        if not check_environment():
            logger.error("Environment check failed. Run 'python run_sensebridge.py setup' to set up the environment.")
            return 1

        # Run tests
        if run_tests(args):
            return 0
        else:
            return 1

    # Should never reach here
    return 0


if __name__ == "__main__":
    sys.exit(main())