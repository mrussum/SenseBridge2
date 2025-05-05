"""
Hardware device control for SenseBridge.
Controls GPIO pins and hardware devices on Raspberry Pi.
"""

import logging
import time
import threading
import os
from ..utils.config import Config

logger = logging.getLogger(__name__)

# Flag to determine if running on Raspberry Pi
try:
    import RPi.GPIO as GPIO

    ON_RASPBERRY_PI = True
except ImportError:
    ON_RASPBERRY_PI = False
    logger.warning("RPi.GPIO not available - running in simulation mode")


class DeviceController:
    """Controls hardware devices connected to Raspberry Pi."""

    # Singleton instance
    _instance = None

    def __new__(cls):
        """Create a new DeviceController instance or return existing one (singleton)."""
        if cls._instance is None:
            cls._instance = super(DeviceController, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        """Initialize the device controller."""
        if self._initialized:
            return

        self._initialized = True

        self.config = Config()
        self.device_config = self.config.get_device_config()
        self.hardware_config = self.device_config["hardware"]

        # Get pin configurations
        self.haptic_pin = self.hardware_config["haptic_pin"]
        self.led_pin = self.hardware_config["led_pin"]
        self.button_pin = self.hardware_config["button_pin"]

        # Active devices
        self.active_devices = {}

        # Button callback
        self.button_callback = None

        # Initialize GPIO if on Raspberry Pi
        if ON_RASPBERRY_PI:
            self._setup_gpio()

        logger.info("DeviceController initialized")

    def _setup_gpio(self):
        """Set up GPIO pins."""
        try:
            # Use BCM pin numbering
            GPIO.setmode(GPIO.BCM)

            # Set up output pins
            GPIO.setup(self.haptic_pin, GPIO.OUT)
            GPIO.setup(self.led_pin, GPIO.OUT)

            # Set up input pin with pull-up resistor
            GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

            # Set up PWM for LED brightness control
            self.led_pwm = GPIO.PWM(self.led_pin, 100)  # 100 Hz
            self.led_pwm.start(0)  # Start with 0% duty cycle (off)

            # Set up PWM for haptic vibration control
            self.haptic_pwm = GPIO.PWM(self.haptic_pin, 100)  # 100 Hz
            self.haptic_pwm.start(0)  # Start with 0% duty cycle (off)

            logger.info("GPIO initialized")

        except Exception as e:
            logger.error(f"Error setting up GPIO: {str(e)}")

    def cleanup(self):
        """Clean up GPIO resources."""
        if ON_RASPBERRY_PI:
            try:
                # Stop PWM
                self.led_pwm.stop()
                self.haptic_pwm.stop()

                # Clean up GPIO
                GPIO.cleanup()
                logger.info("GPIO cleaned up")

            except Exception as e:
                logger.error(f"Error cleaning up GPIO: {str(e)}")

    def activate_device(self, device_name, intensity=1.0, duration=None):
        """Activate a device.

        Args:
            device_name: Name of the device to activate
            intensity: Intensity level (0.0-1.0)
            duration: Duration in seconds (None for indefinite)

        Returns:
            True if successful, False otherwise
        """
        try:
            # Validate intensity
            intensity = max(0.0, min(1.0, intensity))
            duty_cycle = int(intensity * 100)

            logger.debug(f"Activating {device_name} with intensity {intensity}")

            if ON_RASPBERRY_PI:
                # Handle different device types
                if device_name == f"haptic_pin_{self.haptic_pin}" or device_name == "haptic":
                    self.haptic_pwm.ChangeDutyCycle(duty_cycle)

                elif device_name == f"led_pin_{self.led_pin}" or device_name == "led":
                    self.led_pwm.ChangeDutyCycle(duty_cycle)

                else:
                    logger.warning(f"Unknown device: {device_name}")
                    return False
            else:
                # Simulation mode
                logger.debug(f"Simulating activation of {device_name}")

            # Track active device
            self.active_devices[device_name] = {
                "intensity": intensity,
                "timestamp": time.time()
            }

            # Handle automatic deactivation
            if duration is not None:
                threading.Timer(
                    duration,
                    self.deactivate_device,
                    args=[device_name]
                ).start()

            return True

        except Exception as e:
            logger.error(f"Error activating device {device_name}: {str(e)}")
            return False

    def deactivate_device(self, device_name):
        """Deactivate a device.

        Args:
            device_name: Name of the device to deactivate

        Returns:
            True if successful, False otherwise
        """
        try:
            logger.debug(f"Deactivating {device_name}")

            if ON_RASPBERRY_PI:
                # Handle different device types
                if device_name == f"haptic_pin_{self.haptic_pin}" or device_name == "haptic":
                    self.haptic_pwm.ChangeDutyCycle(0)

                elif device_name == f"led_pin_{self.led_pin}" or device_name == "led":
                    self.led_pwm.ChangeDutyCycle(0)

                else:
                    logger.warning(f"Unknown device: {device_name}")
                    return False
            else:
                # Simulation mode
                logger.debug(f"Simulating deactivation of {device_name}")

            # Remove from active devices
            if device_name in self.active_devices:
                del self.active_devices[device_name]

            return True

        except Exception as e:
            logger.error(f"Error deactivating device {device_name}: {str(e)}")
            return False

    def set_button_callback(self, callback):
        """Set callback function for button press.

        Args:
            callback: Function to call when button is pressed

        Returns:
            True if successful, False otherwise
        """
        try:
            self.button_callback = callback

            if ON_RASPBERRY_PI:
                # Set up event detection for button press
                GPIO.remove_event_detect(self.button_pin)
                GPIO.add_event_detect(
                    self.button_pin,
                    GPIO.FALLING,
                    callback=self._button_event_handler,
                    bouncetime=300
                )
                logger.info("Button callback registered")

            return True

        except Exception as e:
            logger.error(f"Error setting button callback: {str(e)}")
            return False

    def _button_event_handler(self, channel):
        """Handle button press event.

        Args:
            channel: GPIO channel that triggered the event
        """
        if self.button_callback:
            # Call the callback in a separate thread to avoid blocking
            threading.Thread(target=self.button_callback).start()


# Legacy functions for compatibility with existing code
def activate_device(device_name):
    """Activate a device (legacy function)."""
    controller = DeviceController()
    return controller.activate_device(device_name)


def deactivate_device(device_name):
    """Deactivate a device (legacy function)."""
    controller = DeviceController()
    return controller.deactivate_device(device_name)