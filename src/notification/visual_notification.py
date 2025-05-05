"""
Visual notification module for SenseBridge.
Controls screen display and LED indicators for visual alerts.
"""

import threading
import time
import logging
import queue
from ..utils.config import Config
from ..hardware.device_control import activate_device, deactivate_device
from ..gui.interface import show_alert

logger = logging.getLogger(__name__)


class VisualNotification:
    """Controls visual notifications and screen display."""

    def __init__(self):
        """Initialize the visual notification system."""
        self.config = Config()
        self.device_config = self.config.get_device_config()

        # Get hardware configuration
        self.hardware_config = self.device_config["hardware"]
        self.led_pin = self.hardware_config["led_pin"]

        # Visual patterns (duration and brightness)
        self.patterns = {
            "flash_low": [(0.5, 0.3), (0.5, 0.0)],  # [(duration, brightness), ...]
            "flash_medium": [(0.5, 0.6), (0.5, 0.0)],
            "flash_bright": [(0.5, 1.0), (0.5, 0.0)],
            "flash_urgent": [(0.2, 1.0), (0.2, 0.0), (0.2, 1.0), (0.2, 0.0), (0.2, 1.0)],
            "constant": [(2.0, 1.0)],
            "gentle_pulse": [(0.5, 0.3), (0.5, 0.1), (0.5, 0.3), (0.5, 0.0)]
        }

        # Queue for visual commands
        self.command_queue = queue.Queue()

        # Thread for processing visual commands
        self.visual_thread = None
        self.running = False

        # Text display queue (for speech-to-text)
        self.text_queue = queue.Queue()
        self.text_thread = None

        # Currently displayed message
        self.current_text = ""
        self.text_timestamp = 0
        self.text_display_time = 10  # seconds to display text

        logger.info("VisualNotification initialized")

    def start(self):
        """Start the visual notification system."""
        if self.running:
            logger.warning("Visual notification system already running")
            return

        self.running = True

        # Start visual processing thread
        self.visual_thread = threading.Thread(target=self._visual_loop)
        self.visual_thread.daemon = True
        self.visual_thread.start()

        # Start text display thread
        self.text_thread = threading.Thread(target=self._text_loop)
        self.text_thread.daemon = True
        self.text_thread.start()

        logger.info("Visual notification system started")

    def stop(self):
        """Stop the visual notification system."""
        if not self.running:
            return

        self.running = False

        # Stop visual thread
        if self.visual_thread:
            self.visual_thread.join(timeout=2.0)

        # Stop text thread
        if self.text_thread:
            self.text_thread.join(timeout=2.0)

        # Ensure LED is off
        deactivate_device(f"led_pin_{self.led_pin}")

        logger.info("Visual notification system stopped")

    def show_notification(self, message, pattern_name="flash_medium", priority="medium"):
        """Show a visual notification.

        Args:
            message: Text message to display
            pattern_name: Name of the visual pattern
            priority: Priority level ("low", "medium", "high")
        """
        # Get pattern, default to flash_medium if not found
        pattern = self.patterns.get(pattern_name, self.patterns["flash_medium"])

        # Add to command queue with priority
        priority_level = {"low": 0, "medium": 1, "high": 2}.get(priority, 1)

        self.command_queue.put({
            "message": message,
            "pattern": pattern,
            "priority": priority_level,
            "timestamp": time.time()
        })

        logger.debug(f"Visual notification queued: {message} (pattern: {pattern_name}, priority: {priority})")

    def show_text(self, text):
        """Display text on the screen (for speech-to-text).

        Args:
            text: Text to display
        """
        if not text:
            return

        self.text_queue.put({
            "text": text,
            "timestamp": time.time()
        })

        logger.debug(f"Text display queued: {text}")

    def _visual_loop(self):
        """Main visual notification processing loop."""
        last_visual_time = 0

        while self.running:
            try:
                # Get next command from queue with timeout
                try:
                    command = self.command_queue.get(timeout=0.5)
                except queue.Empty:
                    continue

                message = command["message"]
                pattern = command["pattern"]
                priority = command["priority"]
                timestamp = command["timestamp"]

                # Skip low priority commands if too recent (within 3 seconds)
                if priority < 2 and time.time() - last_visual_time < 3.0:
                    logger.debug("Skipping low priority visual notification")
                    continue

                # Show alert message in GUI
                show_alert(message)

                # Execute visual pattern
                self._execute_pattern(pattern)

                # Update last visual time
                last_visual_time = time.time()

            except Exception as e:
                logger.error(f"Error in visual notification loop: {str(e)}")

    def _text_loop(self):
        """Main text display processing loop."""
        while self.running:
            try:
                # Check if current text has expired
                if self.current_text and time.time() - self.text_timestamp > self.text_display_time:
                    self.current_text = ""

                # Get next text from queue with timeout
                try:
                    text_data = self.text_queue.get(timeout=0.5)
                except queue.Empty:
                    continue

                text = text_data["text"]
                timestamp = text_data["timestamp"]

                # Update current text and timestamp
                self.current_text = text
                self.text_timestamp = timestamp

                # Display text in GUI
                # Here we're using show_alert, but in a real implementation
                # this would update a dedicated text area in the GUI
                show_alert(f"Speech: {text}")

            except Exception as e:
                logger.error(f"Error in text display loop: {str(e)}")

    def _execute_pattern(self, pattern):
        """Execute a visual pattern.

        Args:
            pattern: List of (duration, brightness) tuples
        """
        try:
            logger.debug(f"Executing visual pattern: {pattern}")

            for duration, brightness in pattern:
                if not self.running:
                    break

                if brightness > 0:
                    # Activate LED with appropriate brightness
                    activate_device(f"led_pin_{self.led_pin}")
                    time.sleep(duration)
                    deactivate_device(f"led_pin_{self.led_pin}")
                else:
                    # LED off period
                    time.sleep(duration)

        except Exception as e:
            logger.error(f"Error executing visual pattern: {str(e)}")


def show_visual_notification():
    """Legacy function for compatibility with existing code."""
    # This is just a placeholder
    # The actual implementation uses the VisualNotification class
    activate_device("led")
    time.sleep(0.5)
    deactivate_device("led")