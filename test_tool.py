# test_tool.py
"""
SenseBridge automated testing tool.
Provides a framework for testing different components of SenseBridge.
"""

import os
import sys
import time
import argparse
import logging
import threading
import queue
import json
import random
import numpy as np
from collections import defaultdict

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("test_results.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("SenseBridgeTest")

# Test result counter
test_results = defaultdict(int)


def run_test(test_name, test_func, *args, **kwargs):
    """Run a test and record the result.

    Args:
        test_name: Name of the test
        test_func: Test function to run
        *args: Arguments to pass to test function
        **kwargs: Keyword arguments to pass to test function

    Returns:
        True if test passed, False otherwise
    """
    logger.info(f"Running test: {test_name}")

    try:
        start_time = time.time()
        result = test_func(*args, **kwargs)
        elapsed_time = time.time() - start_time

        if result:
            logger.info(f"✓ Test passed: {test_name} ({elapsed_time:.2f}s)")
            test_results["passed"] += 1
        else:
            logger.error(f"✗ Test failed: {test_name} ({elapsed_time:.2f}s)")
            test_results["failed"] += 1

        return result

    except Exception as e:
        logger.error(f"✗ Test error: {test_name} - {e}")
        import traceback
        traceback.print_exc()
        test_results["errors"] += 1
        return False


def test_config():
    """Test configuration module."""
    # Add current directory to Python path
    if not os.getcwd() in sys.path:
        sys.path.insert(0, os.getcwd())

    # Import configuration
    from src.utils.config import Config

    config = Config()
    logger.info("Testing configuration loading...")

    # Check device config
    device_config = config.get_device_config()
    if not device_config or not isinstance(device_config, dict):
        logger.error("Device config is empty or invalid")
        return False

    logger.info(f"Device config has {len(device_config)} sections: {list(device_config.keys())}")

    # Check sound events
    sound_events = config.get_sound_events()
    if not sound_events or not isinstance(sound_events, dict):
        logger.error("Sound events config is empty or invalid")
        return False

    logger.info(f"Sound events config has {len(sound_events)} events: {list(sound_events.keys())}")

    # Check user preferences
    user_prefs = config.get_user_preferences()
    if not user_prefs or not isinstance(user_prefs, dict):
        logger.error("User preferences are empty or invalid")
        return False

    logger.info(f"User preferences has {len(user_prefs)} sections: {list(user_prefs.keys())}")

    # Test saving config changes
    device_config["test_value"] = "test"
    result = config.update_device_config(device_config)
    if not result:
        logger.error("Failed to save device config changes")
        return False

    # Verify changes were saved
    updated_config = config.get_device_config()
    if "test_value" not in updated_config or updated_config["test_value"] != "test":
        logger.error("Failed to verify device config changes")
        return False

    # Remove test value
    del updated_config["test_value"]
    config.update_device_config(updated_config)

    return True


def test_audio_processor(duration=3):
    """Test audio processing module."""
    # Add current directory to Python path
    if not os.getcwd() in sys.path:
        sys.path.insert(0, os.getcwd())

    # Import audio processor
    from src.audio.audio_processor import AudioProcessor

    logger.info("Testing audio processor...")

    # Create audio processor
    processor = AudioProcessor()

    # Start processor
    processor.start()
    logger.info("Audio processor started")

    # Get some audio data
    logger.info(f"Capturing audio for {duration} seconds...")
    start_time = time.time()
    samples = []

    while time.time() - start_time < duration:
        audio_data = processor.get_audio_data(timeout=0.1)
        if audio_data is not None:
            samples.append(audio_data)
            logger.info(f"Got audio sample, length: {len(audio_data)}")
        time.sleep(0.1)

    # Stop processor
    processor.stop()
    logger.info("Audio processor stopped")

    if len(samples) > 0:
        logger.info(f"Captured {len(samples)} audio samples")
        return True
    else:
        logger.error("No audio samples captured")
        return False


def test_sound_recognition(duration=10):
    """Test sound recognition."""
    # Add current directory to Python path
    if not os.getcwd() in sys.path:
        sys.path.insert(0, os.getcwd())

    # Import sound recognition
    from src.audio.sound_recognition import SoundRecognition

    detected_sounds = []

    def sound_callback(sound_type, confidence, audio_data):
        logger.info(f"Detected sound: {sound_type} (confidence: {confidence:.2f})")
        detected_sounds.append((sound_type, confidence))

    logger.info("Testing sound recognition...")
    recognition = SoundRecognition(callback=sound_callback)

    # Start recognition
    recognition.start()
    logger.info(f"Sound recognition started, running for {duration} seconds...")
    logger.info("Make some noise (knock, clap, etc.) to test detection...")

    # Wait for detection
    time.sleep(duration)

    # Stop recognition
    recognition.stop()
    logger.info("Sound recognition stopped")

    if detected_sounds:
        logger.info(f"Detected {len(detected_sounds)} sounds during test")
        for sound_type, confidence in detected_sounds:
            logger.info(f"  - {sound_type}: {confidence:.2f}")
        return True
    else:
        logger.warning("No sounds detected during test")
        return False


def test_notification():
    """Test notification systems."""
    # Add current directory to Python path
    if not os.getcwd() in sys.path:
        sys.path.insert(0, os.getcwd())

    # Import notification manager
    from src.notification.notification_manager import NotificationManager

    logger.info("Testing notification manager...")
    manager = NotificationManager()

    # Start notification manager
    manager.start()
    logger.info("Notification manager started")

    # Test results
    results = []

    # Send test notifications with delay between each
    for event_type in ["doorbell", "knock", "alarm"]:
        logger.info(f"Testing notification for {event_type}...")
        success = manager.notify(event_type, confidence=0.8)
        results.append(success)
        time.sleep(2.0)

    # Stop notification manager
    manager.stop()
    logger.info("Notification manager stopped")

    # Check if all notifications were successful
    if all(results):
        logger.info("All notifications were successful")
        return True
    else:
        logger.error(f"Some notifications failed: {results}")
        return False


def test_gui():
    """Test GUI functionality."""
    # Add current directory to Python path
    if not os.getcwd() in sys.path:
        sys.path.insert(0, os.getcwd())

    # Import GUI app
    from src.gui.app import create_app

    logger.info("Testing GUI functionality...")

    try:
        # Create app in headless mode for testing
        app = create_app(use_gui=False)

        # Test notification methods
        app.show_notification("Test Notification")
        app.update_speech_text("This is a test of speech-to-text display")
        app.update_status_message("System is working properly")

        logger.info("GUI test completed successfully")
        return True
    except Exception as e:
        logger.error(f"GUI test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_hardware():
    """Test hardware control."""
    # Add current directory to Python path
    if not os.getcwd() in sys.path:
        sys.path.insert(0, os.getcwd())

    # Import device controller
    from src.hardware.device_control import DeviceController

    logger.info("Testing hardware control...")

    try:
        # Create device controller
        controller = DeviceController()

        # Test LED control
        logger.info("Testing LED control...")
        controller.activate_device("led", 0.5)
        time.sleep(1.0)
        controller.deactivate_device("led")

        # Test haptic control
        logger.info("Testing haptic control...")
        controller.activate_device("haptic", 0.8)
        time.sleep(1.0)
        controller.deactivate_device("haptic")

        # Test button callback
        button_pressed = threading.Event()

        def button_callback():
            logger.info("Button callback triggered")
            button_pressed.set()

        controller.set_button_callback(button_callback)
        logger.info("Press the button within 10 seconds...")

        # Wait for button press or timeout
        button_pressed.wait(10.0)

        if button_pressed.is_set():
            logger.info("Button press detected")
        else:
            logger.warning("No button press detected (timeout)")

        # Clean up
        controller.cleanup()

        logger.info("Hardware test completed")
        return True
    except Exception as e:
        logger.error(f"Hardware test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_integration(duration=30):
    """Test full system integration."""
    # Add current directory to Python path
    if not os.getcwd() in sys.path:
        sys.path.insert(0, os.getcwd())

    # Import SenseBridge main class
    from src.main import SenseBridge

    logger.info(f"Testing full system integration for {duration} seconds...")

    # Track detected events
    detected_events = []

    # Create SenseBridge with headless and simulation mode
    app = SenseBridge(headless=True, simulation=True)

    # Override callbacks to track events
    original_sound_callback = app.on_sound_detected
    original_speech_callback = app.on_speech_recognized

    def sound_callback_wrapper(sound_type, confidence, audio_data):
        detected_events.append(("sound", sound_type, confidence))
        return original_sound_callback(sound_type, confidence, audio_data)

    def speech_callback_wrapper(text):
        detected_events.append(("speech", text))
        return original_speech_callback(text)

    app.on_sound_detected = sound_callback_wrapper
    app.on_speech_recognized = speech_callback_wrapper

    # Start the application with timeout
    logger.info("Starting SenseBridge...")

    # Use a thread to run SenseBridge
    stop_event = threading.Event()

    def run_app():
        try:
            app.start_timeout(duration)
        except Exception as e:
            logger.error(f"Error in SenseBridge: {e}")
        finally:
            stop_event.set()

    app_thread = threading.Thread(target=run_app)
    app_thread.daemon = True
    app_thread.start()

    # Wait for application to start
    time.sleep(2.0)

    # Simulate some events during the test
    logger.info("Simulating events...")

    # Simulate button press
    time.sleep(2.0)
    logger.info("Simulating button press...")
    app.on_button_press()

    # Simulate sound detection
    time.sleep(2.0)
    for sound_type in ["doorbell", "knock", "alarm", "microwave_beep"]:
        audio_data = np.random.normal(0, 0.3, 16000).astype(np.float32)
        confidence = random.uniform(0.7, 0.95)
        logger.info(f"Simulating sound detection: {sound_type} ({confidence:.2f})")
        app.on_sound_detected(sound_type, confidence, audio_data)
        time.sleep(2.0)

    # Simulate speech recognition
    time.sleep(2.0)
    for text in ["Hello, this is a test", "SenseBridge is working", "Testing speech recognition"]:
        logger.info(f"Simulating speech recognition: {text}")
        app.on_speech_recognized(text)
        time.sleep(2.0)

    # Wait for application to complete
    logger.info(f"Waiting for test to complete (max {duration} seconds)...")
    stop_event.wait(timeout=duration)

    # Stop the application if still running
    if not stop_event.is_set():
        logger.warning("Forcing application to stop (timeout reached)")
        app.stop()

    # Check results
    sound_events = [event for event in detected_events if event[0] == "sound"]
    speech_events = [event for event in detected_events if event[0] == "speech"]

    logger.info(f"Detected {len(sound_events)} sound events and {len(speech_events)} speech events")

    if len(sound_events) > 0 and len(speech_events) > 0:
        logger.info("Integration test completed successfully")
        return True
    else:
        logger.error("Integration test failed to detect events")
        return False


def main():
    """Main entry point for test tool."""
    parser = argparse.ArgumentParser(description="SenseBridge test tool")
    parser.add_argument("--all", action="store_true", help="Run all tests")
    parser.add_argument("--config", action="store_true", help="Test configuration")
    parser.add_argument("--audio", action="store_true", help="Test audio processing")
    parser.add_argument("--recognition", action="store_true", help="Test sound recognition")
    parser.add_argument("--notification", action="store_true", help="Test notifications")
    parser.add_argument("--gui", action="store_true", help="Test GUI")
    parser.add_argument("--hardware", action="store_true", help="Test hardware control")
    parser.add_argument("--integration", action="store_true", help="Test full system integration")
    parser.add_argument("--duration", type=int, default=30, help="Duration for long-running tests (seconds)")

    args = parser.parse_args()

    # If no specific tests are requested, run all tests
    run_all = args.all or not (args.config or args.audio or args.recognition or
                               args.notification or args.gui or args.hardware or
                               args.integration)

    if run_all or args.config:
        run_test("Configuration", test_config)

    if run_all or args.gui:
        run_test("GUI", test_gui)

    if run_all or args.notification:
        run_test("Notification", test_notification)

    if run_all or args.hardware:
        run_test("Hardware", test_hardware)

    if run_all or args.audio:
        run_test("Audio Processing", test_audio_processor, 3)

    if run_all or args.recognition:
        run_test("Sound Recognition", test_sound_recognition, args.duration)

    if run_all or args.integration:
        run_test("Integration", test_integration, args.duration)

    # Print test summary
    print("\n=== Test Summary ===")
    print(f"Passed: {test_results['passed']}")
    print(f"Failed: {test_results['failed']}")
    print(f"Errors: {test_results['errors']}")
    print(f"Total:  {sum(test_results.values())}")

    # Return success if all tests passed
    return test_results['failed'] == 0 and test_results['errors'] == 0


if __name__ == "__main__":
    sys.exit(0 if main() else 1)