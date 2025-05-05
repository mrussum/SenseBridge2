# test_system.py
import os
import sys
import time
import argparse
import logging

# Setup logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("SenseBridgeTest")


def test_config():
    """Test configuration module."""
    try:
        from src.utils.config import Config

        config = Config()
        logger.info("Testing configuration loading...")

        device_config = config.get_device_config()
        logger.info(f"Device config: {list(device_config.keys())}")

        sound_events = config.get_sound_events()
        logger.info(f"Sound events: {list(sound_events.keys())}")

        user_prefs = config.get_user_preferences()
        logger.info(f"User preferences: {list(user_prefs.keys())}")

        return True
    except Exception as e:
        logger.error(f"Config test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_audio_processing(duration=3):
    """Test audio processing module."""
    try:
        from src.audio.audio_processor import AudioProcessor

        logger.info("Testing audio processor...")
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

        return len(samples) > 0
    except Exception as e:
        logger.error(f"Audio processing test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_sound_recognition(duration=10):
    """Test sound recognition."""
    try:
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
            return True
        else:
            logger.warning("No sounds detected during test")
            return False
    except Exception as e:
        logger.error(f"Sound recognition test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_notification():
    """Test notification systems."""
    try:
        from src.notification.notification_manager import NotificationManager

        logger.info("Testing notification manager...")
        manager = NotificationManager()

        # Start notification manager
        manager.start()
        logger.info("Notification manager started")

        # Send test notifications
        for event_type in ["doorbell", "knock", "alarm"]:
            logger.info(f"Testing notification for {event_type}...")
            manager.notify(event_type, confidence=0.8)
            time.sleep(2.0)

        # Stop notification manager
        manager.stop()
        logger.info("Notification manager stopped")

        return True
    except Exception as e:
        logger.error(f"Notification test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_gui():
    """Test GUI functionality."""
    try:
        from src.gui.app import create_app

        logger.info("Testing GUI functionality...")
        app = create_app(use_gui=False)  # Headless mode for testing

        logger.info("Sending test notifications to GUI...")
        app.show_notification("Test Notification")
        app.update_speech_text("This is a test of speech-to-text display")
        app.update_status_message("System is working properly")

        logger.info("GUI test completed")
        return True
    except Exception as e:
        logger.error(f"GUI test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    parser = argparse.ArgumentParser(description="Test SenseBridge system")
    parser.add_argument("--all", action="store_true", help="Run all tests")
    parser.add_argument("--config", action="store_true", help="Test configuration")
    parser.add_argument("--audio", action="store_true", help="Test audio processing")
    parser.add_argument("--recognition", action="store_true", help="Test sound recognition")
    parser.add_argument("--notification", action="store_true", help="Test notifications")
    parser.add_argument("--gui", action="store_true", help="Test GUI")

    args = parser.parse_args()

    # If no specific tests are specified, run all tests
    run_all = args.all or not (args.config or args.audio or args.recognition or
                               args.notification or args.gui)

    results = {}

    if run_all or args.config:
        results["Configuration"] = test_config()

    if run_all or args.audio:
        results["Audio Processing"] = test_audio_processing()

    if run_all or args.recognition:
        results["Sound Recognition"] = test_sound_recognition()

    if run_all or args.notification:
        results["Notification"] = test_notification()

    if run_all or args.gui:
        results["GUI"] = test_gui()

    # Print summary
    logger.info("\n=== Test Results Summary ===")
    for test, result in results.items():
        logger.info(f"{test}: {'PASS' if result else 'FAIL'}")


if __name__ == "__main__":
    main()