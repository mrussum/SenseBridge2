# Updated run_simulation.py
import sys
import os
import time


def run_tests():
    """Run the test suite with mocks."""
    print("\n=== Running SenseBridge Tests with Mock Hardware ===\n")

    # Use correct paths with forward slashes
    tests = [
        "test/test_config.py",
        "test/test_audio.py",
        "test/test_notification.py",
        "test/test_gui.py"
    ]

    for test in tests:
        print(f"\n--- Running {test} ---\n")
        test_path = os.path.join(os.getcwd(), test.replace('/', os.sep))
        if os.path.exists(test_path):
            os.system(f"python {test_path}")
        else:
            print(f"Test file not found: {test_path}")

    print("\nAll tests completed.")


def run_main():
    """Run the main application in simulation mode."""
    print("\n=== Starting SenseBridge in Simulation Mode ===\n")

    try:
        # Run with a short timeout just to test it
        os.system("python -m src.main --headless --simulation --timeout=5")
        return True
    except Exception as e:
        print(f"SenseBridge simulation failed: {e}")
        return False


if __name__ == "__main__":
    # First run tests
    run_tests()

    # Then try running the main app
    run_main()