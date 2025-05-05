# src/main.py
"""
Main entry point for SenseBridge application.
Initializes and coordinates all system components with better error handling.
"""

import logging
import threading
import time
import os
import signal
import sys
import argparse
from .utils.logger import setup_logging
from .utils.config import Config
from .utils.hardware_detection import get_hardware_detector
from .audio.sound_recognition import SoundRecognition
from .speech.speech_to_text import SpeechToText
from .notification.notification_manager import NotificationManager
from .hardware.device_control import DeviceController
from .hardware.wearable import WearableDevice
from .gui.app import create_app

# Conditionally import simulator
try:
    from .simulator.simulator_ui import SimulatorUI
    SIMULATOR_AVAILABLE = True
except ImportError:
    SIMULATOR_AVAILABLE = False

# Global flag to signal program exit
running = True


class SenseBridge:
    """Main application class for SenseBridge."""

    def __init__(self, headless=False, simulation=False):
        """Initialize the SenseBridge application.

        Args:
            headless: Run in headless mode (no GUI)
            simulation: Run in simulation mode (simulated hardware)
        """
        # Set up logging
        self.logger = setup_logging()
        self.logger.info("Initializing SenseBridge...")
        
        # Check hardware capabilities
        self.hardware = get_hardware_detector()
        self.logger.info(f"Hardware capabilities: {self.hardware.get_capabilities()}")
        
        # If no display and not headless, switch to headless mode
        if not self.hardware.has_display and not headless:
            self.logger.warning("No display detected, switching to headless mode")
            headless = True
        
        # Store parameters
        self.headless = headless
        self.simulation = simulation or not self.hardware.is_raspberry_pi
        
        if self.simulation:
            self.logger.info("Running in simulation mode")

        # Load configuration
        self.config = Config()

        # Initialize components
        self.notification_manager = None
        self.sound_recognition = None
        self.speech_to_text = None
        self.device_controller = None
        self.wearable = None
        self.app = None
        self.simulator = None
        
        try:
            # Initialize components with error handling
            self._initialize_components()
            
            # Register signal handlers for graceful shutdown
            signal.signal(signal.SIGINT, self.signal_handler)
            signal.signal(signal.SIGTERM, self.signal_handler)

            self.logger.info("SenseBridge initialized")
            
        except Exception as e:
            self.logger.error(f"Error initializing SenseBridge: {str(e)}")
            self.stop()
            raise
            
    def _initialize_components(self):
        """Initialize all system components."""
        # Initialize device controller first
        self.device_controller = DeviceController()
        
        # Initialize notification manager
        self.notification_manager = NotificationManager()
        
        # Initialize wearable device
        self.wearable = WearableDevice()
        
        # Initialize speech-to-text if audio is available
        if self.hardware.has_audio:
            self.speech_to_text = SpeechToText(text_callback=self.on_speech_recognized)
        else:
            self.logger.warning("Audio not available, speech-to-text disabled")
            self.speech_to_text = None
            
        # Initialize sound recognition if audio is available
        if self.hardware.has_audio:
            self.sound_recognition = SoundRecognition(callback=self.on_sound_detected)
        else:
            self.logger.warning("Audio not available, sound recognition disabled")
            self.sound_recognition = None
        
        # Create GUI app
        self.app = create_app(use_gui=not self.headless)
        
        # Create simulator if in simulation mode
        if self.simulation and SIMULATOR_AVAILABLE:
            self.simulator = SimulatorUI(
                sound_callback=self.on_sound_detected,
                button_callback=self.on_button_press
            )
            self.logger.info("Simulator UI created")
        else:
            self.simulator = None
    
    def start(self):
        """Start all SenseBridge components."""
        self.logger.info("Starting SenseBridge...")

        try:
            # Start notification system first
            self.notification_manager.start()

            # Start wearable device
            self.wearable.

    # src/main.py (continued)
    def start(self):
        """Start all SenseBridge components."""
        self.logger.info("Starting SenseBridge...")

        try:
            # Start notification system first
            self.notification_manager.start()

            # Start wearable device
            self.wearable.start()

            # Register button callback
            self.device_controller.set_button_callback(self.on_button_press)

            # Start speech-to-text if available
            if self.speech_to_text:
                self.speech_to_text.start()

            # Start sound recognition if available
            if self.sound_recognition:
                self.sound_recognition.start()

            # Start simulator if available
            if self.simulator:
                self.simulator.start()

            # Show startup message
            self.app.show_notification("SenseBridge is ready!")
            self.app.update_status_message("System active")

            self.logger.info("SenseBridge started")

            # Start GUI main loop
            if hasattr(self.app.root, 'mainloop'):
                self.logger.info("Starting GUI main loop")
                self.app.root.mainloop()
            else:
                # Headless mode - just keep running
                self.logger.info("Running in headless mode")
                global running
                while running:
                    time.sleep(1)

        except Exception as e:
            self.logger.error(f"Error starting SenseBridge: {str(e)}")
            self.stop()

    def start_timeout(self, timeout):
        """Start SenseBridge with a timeout.

        Args:
            timeout: Time in seconds to run before stopping
        """

        def timeout_handler():
            self.logger.info(f"Timeout of {timeout} seconds reached")
            self.stop()

        # Start timeout thread
        timer = threading.Timer(timeout, timeout_handler)
        timer.daemon = True
        timer.start()

        # Start normally
        self.start()

    def stop(self):
        """Stop all SenseBridge components."""
        self.logger.info("Stopping SenseBridge...")

        # Stop components in reverse order
        if self.simulator:
            self.simulator.stop()

        if self.sound_recognition:
            self.sound_recognition.stop()

        if self.speech_to_text:
            self.speech_to_text.stop()

        if self.wearable:
            self.wearable.stop()

        if self.notification_manager:
            self.notification_manager.stop()

        # Clean up GPIO
        if self.device_controller:
            self.device_controller.cleanup()

        self.logger.info("SenseBridge stopped")

    def on_sound_detected(self, sound_type, confidence, audio_data):
        """Callback for when a sound is detected.

        Args:
            sound_type: Type of detected sound
            confidence: Detection confidence
            audio_data: Raw audio data
        """
        self.logger.info(f"Sound detected: {sound_type} (confidence: {confidence:.2f})")

        # Notify through the notification manager
        self.notification_manager.notify(sound_type, confidence, audio_data)

        # Update GUI
        self.app.show_notification(f"Detected: {sound_type.capitalize()}")
        self.app.update_status_message(f"Last event: {sound_type} ({confidence:.2f})")

        # Update simulator if available
        if self.simulator:
            self.simulator.log_event(f"Detected sound: {sound_type} ({confidence:.2f})")

    def on_speech_recognized(self, text):
        """Callback for when speech is recognized.

        Args:
            text: Recognized speech text
        """
        if text:
            self.logger.info(f"Speech recognized: {text}")

            # Notify through the notification manager
            self.notification_manager.notify("speech", 1.0, text)

            # Update GUI with speech text
            self.app.update_speech_text(text)

            # Update simulator if available
            if self.simulator:
                self.simulator.log_event(f"Speech recognized: {text}")

    def on_button_press(self):
        """Callback for when the button is pressed."""
        self.logger.info("Button pressed")

        # Trigger a test notification
        self.notification_manager.notify("doorbell", 1.0, None)

        # Update GUI
        self.app.show_notification("Button pressed")

        # Update simulator if available
        if self.simulator:
            self.simulator.log_event("Button pressed")

    def signal_handler(self, sig, frame):
        """Handle termination signals for graceful shutdown.

        Args:
            sig: Signal number
            frame: Current stack frame
        """
        global running
        self.logger.info(f"Received signal {sig}, shutting down...")
        running = False

        # Stop all components
        self.stop()

        # Exit program
        sys.exit(0)


def main():
    """Main entry point for SenseBridge application."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="SenseBridge assistive technology system")
    parser.add_argument("--headless", action="store_true", help="Run in headless mode (no GUI)")
    parser.add_argument("--simulation", action="store_true", help="Run in simulation mode (simulated hardware)")
    parser.add_argument("--timeout", type=float, help="Exit after specified timeout (in seconds)")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")

    args = parser.parse_args()

    # Set up logging level
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    # Create and start SenseBridge
    app = SenseBridge(headless=args.headless, simulation=args.simulation)

    try:
        # If timeout specified, run with timeout
        if args.timeout:
            app.start_timeout(args.timeout)
        else:
            app.start()
    except KeyboardInterrupt:
        print("\nReceived keyboard interrupt")
        app.stop()
    except Exception as e:
        print(f"Error: {e}")
        app.stop()


if __name__ == "__main__":
    main()