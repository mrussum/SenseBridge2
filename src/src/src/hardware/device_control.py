# src/hardware/device_control.py
"""
Enhanced hardware device control for SenseBridge.
Controls GPIO pins and hardware devices with better fallback handling.
"""

import logging
import time
import threading
import os
from ..utils.config import Config
from ..utils.hardware_detection import get_hardware_detector

logger = logging.getLogger(__name__)

# Get hardware capabilities
hardware = get_hardware_detector()

# Import GPIO if available
if hardware.has_gpio:
    try:
        import RPi.GPIO as GPIO
        GPIO_AVAILABLE = True
    except ImportError:
        GPIO_AVAILABLE = False
else:
    GPIO_AVAILABLE = False
    logger.warning("GPIO not available - running in simulation mode")


class DeviceController:
    """Controls hardware devices with graceful fallbacks."""

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

        # LED PWM instance
        self.led_pwm = None

        # Haptic PWM instance
        self.haptic_pwm = None

        # Initialize GPIO if available
        if GPIO_AVAILABLE:
            self._setup_gpio()
        else:
            logger.info("Running in simulation mode (no GPIO)")

        logger.info("DeviceController initialized")

    def _setup_gpio(self):
        """Set up GPIO pins."""
        try:
            # Use BCM pin numbering
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)

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
            # If GPIO setup fails, set to None to indicate simulation mode
            self.led_pwm = None
            self.haptic_pwm = None

    def cleanup(self):
        """Clean up GPIO resources."""
        if GPIO_AVAILABLE:
            try:
                # Stop PWM
                if self.led_pwm:
                    self.led_pwm.stop()
                if self.haptic_pwm:
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

            if GPIO_AVAILABLE:
                # Handle different device types
                if device_name in [f"haptic_pin_{self.haptic_pin}", "haptic"]:
                    if self.haptic_pwm:
                        self.haptic_pwm.ChangeDutyCycle(duty_cycle)
                    else:
                        logger.debug(f"Simulating haptic activation: {duty_cycle}%")

                elif device_name in [f"led_pin_{self.led_pin}", "led"]:
                    if self.led_pwm:
                        self.led_pwm.ChangeDutyCycle(duty_cycle)
                    else:
                        logger.debug(f"Simulating LED activation: {duty_cycle}%")

                else:
                    logger.warning(f"Unknown device: {device_name}")
                    return False
            else:
                # Simulation mode
                logger.debug(f"Simulating activation of {device_name}: {duty_cycle}%")

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

            if GPIO_AVAILABLE:
                # Handle different device types
                if device_name in [f"haptic_pin_{self.haptic_pin}", "haptic"]:
                    if self.haptic_pwm:
                        self.haptic_pwm.ChangeDutyCycle(0)

                elif device_name in [f"led_pin_{self.led_pin}", "led"]:
                    if self.led_pwm:
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

            if GPIO_AVAILABLE:
                # Set up event detection for button press
                try:
                    GPIO.remove_event_detect(self.button_pin)
                except:
                    pass

                GPIO.add_event_detect(
                    self.button_pin,
                    GPIO.FALLING,
                    callback=self._button_event_handler,
                    bouncetime=300
                )
                logger.info("Button callback registered")
            else:
                # In simulation mode, we'll create a thread to simulate button presses
                threading.Thread(target=self._simulate_button_presses, daemon=True).start()
                logger.info("Button simulation started")

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

    def _simulate_button_presses(self):
        """Simulate button presses in simulation mode."""
        while True:
            # Sleep for a random time between 20-40 seconds
            time.sleep(30)

            # If a callback is registered, call it
            if self.button_callback:
                logger.info("Simulating button press")
                self.button_callback()


# Legacy functions for compatibility with existing code
def activate_device(device_name, intensity=1.0, duration=None):
    """Activate a device (legacy function)."""
    controller = DeviceController()
    return controller.activate_device(device_name, intensity, duration)


def deactivate_device(device_name):
    """Deactivate a device (legacy function)."""
    controller = DeviceController()
    return controller.deactivate_device(device_name)