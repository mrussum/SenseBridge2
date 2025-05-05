# src/mock/gpio.py
"""Mock GPIO module for development on systems without Raspberry Pi GPIO."""

import logging
import threading
import time

logger = logging.getLogger(__name__)

# Constants for pin modes
OUT = "OUT"
IN = "IN"
BCM = "BCM"
BOARD = "BOARD"
PUD_UP = "PUD_UP"
PUD_DOWN = "PUD_DOWN"
FALLING = "FALLING"
RISING = "RISING"
BOTH = "BOTH"

# Store pin states
_pin_modes = {}
_pin_values = {}
_pin_event_callbacks = {}
_pin_event_bouncetime = {}
_pwm_instances = {}


# Mock functions
def setmode(mode):
    """Set GPIO pin numbering mode."""
    logger.debug(f"[MOCK] Setting GPIO mode to {mode}")


def setwarnings(flag):
    """Set warnings flag."""
    logger.debug(f"[MOCK] Setting GPIO warnings to {flag}")


def setup(pin, mode, pull_up_down=None, initial=None):
    """Set up a GPIO pin."""
    _pin_modes[pin] = mode
    if mode == OUT:
        _pin_values[pin] = initial if initial is not None else False
    logger.debug(f"[MOCK] Setting up GPIO pin {pin} as {mode}")


def output(pin, value):
    """Set output value for a pin."""
    _pin_values[pin] = value
    logger.debug(f"[MOCK] Setting GPIO pin {pin} to {value}")


def input(pin):
    """Get input value from a pin."""
    value = _pin_values.get(pin, False)
    logger.debug(f"[MOCK] Reading GPIO pin {pin}: {value}")
    return value


def cleanup():
    """Clean up GPIO resources."""
    _pin_modes.clear()
    _pin_values.clear()
    _pin_event_callbacks.clear()
    _pin_event_bouncetime.clear()
    for pwm in _pwm_instances.values():
        pwm.stop()
    _pwm_instances.clear()
    logger.debug("[MOCK] GPIO cleanup")


def add_event_detect(pin, edge, callback=None, bouncetime=None):
    """Add event detection to a pin."""
    _pin_event_callbacks[pin] = callback
    _pin_event_bouncetime[pin] = bouncetime
    logger.debug(f"[MOCK] Adding event detection to pin {pin} for edge {edge}")

    # Start a thread to simulate events
    if callback:
        def event_simulator():
            while pin in _pin_event_callbacks:
                # Sleep for a random time between 5-15 seconds
                time.sleep(10)
                # Call the callback if it's still registered
                if pin in _pin_event_callbacks and _pin_event_callbacks[pin]:
                    logger.debug(f"[MOCK] Simulating event on pin {pin}")
                    _pin_event_callbacks[pin](pin)

        threading.Thread(target=event_simulator, daemon=True).start()


def remove_event_detect(pin):
    """Remove event detection from a pin."""
    if pin in _pin_event_callbacks:
        del _pin_event_callbacks[pin]
    if pin in _pin_event_bouncetime:
        del _pin_event_bouncetime[pin]
    logger.debug(f"[MOCK] Removing event detection from pin {pin}")


class PWM:
    """Mock PWM class."""

    def __init__(self, pin, frequency):
        """Initialize PWM on a pin."""
        self.pin = pin
        self.frequency = frequency
        self.duty_cycle = 0
        self.running = False
        _pwm_instances[pin] = self
        logger.debug(f"[MOCK] Creating PWM instance for pin {pin} with frequency {frequency} Hz")

    def start(self, duty_cycle):
        """Start PWM with a specified duty cycle."""
        self.duty_cycle = duty_cycle
        self.running = True
        logger.debug(f"[MOCK] Starting PWM on pin {self.pin} with duty cycle {duty_cycle}%")

    def ChangeDutyCycle(self, duty_cycle):
        """Change PWM duty cycle."""
        self.duty_cycle = duty_cycle
        logger.debug(f"[MOCK] Changing duty cycle on pin {self.pin} to {duty_cycle}%")

    def ChangeFrequency(self, frequency):
        """Change PWM frequency."""
        self.frequency = frequency
        logger.debug(f"[MOCK] Changing frequency on pin {self.pin} to {frequency} Hz")

    def stop(self):
        """Stop PWM."""
        self.running = False
        logger.debug(f"[MOCK] Stopping PWM on pin {self.pin}")