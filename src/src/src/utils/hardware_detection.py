# src/utils/hardware_detection.py
import os
import platform
import logging
import importlib.util

logger = logging.getLogger(__name__)


class HardwareDetector:
    """Detects available hardware and capabilities."""

    def __init__(self):
        """Initialize hardware detection."""
        self.platform = platform.system()
        self.is_raspberry_pi = self._detect_raspberry_pi()
        self.has_gpio = self._check_gpio()
        self.has_audio = self._check_audio()
        self.has_bluetooth = self._check_bluetooth()
        self.has_display = self._check_display()

        logger.info(f"Hardware detection: Platform={self.platform}, "
                    f"RaspberryPi={self.is_raspberry_pi}, "
                    f"GPIO={self.has_gpio}, "
                    f"Audio={self.has_audio}, "
                    f"Bluetooth={self.has_bluetooth}, "
                    f"Display={self.has_display}")

    def _detect_raspberry_pi(self):
        """Detect if running on Raspberry Pi."""
        # Check for Raspberry Pi-specific files
        if os.path.exists('/proc/device-tree/model'):
            with open('/proc/device-tree/model') as f:
                model = f.read()
                if 'Raspberry Pi' in model:
                    return True

        # Check for /etc/rpi-issue which is present on Raspberry Pi OS
        if os.path.exists('/etc/rpi-issue'):
            return True

        return False

    def _check_gpio(self):
        """Check if GPIO access is available."""
        try:
            # Try to import RPi.GPIO
            import RPi.GPIO
            return True
        except ImportError:
            # Try to import the mock module
            try:
                spec = importlib.util.find_spec('src.mock.gpio')
                if spec is not None:
                    return False  # Mock available, but not real GPIO
            except:
                pass

            logger.warning("GPIO access not available")
            return False

    def _check_audio(self):
        """Check if audio capture is available."""
        try:
            import pyaudio
            p = pyaudio.PyAudio()
            input_device_count = p.get_device_count()
            p.terminate()
            return input_device_count > 0
        except:
            logger.warning("Audio capture not available")
            return False

    def _check_bluetooth(self):
        """Check if Bluetooth is available."""
        try:
            import bluetooth
            return True
        except ImportError:
            logger.warning("Bluetooth not available")
            return False

    def _check_display(self):
        """Check if display is available."""
        # Check for DISPLAY environment variable
        if "DISPLAY" in os.environ and os.environ["DISPLAY"]:
            return True

        # Check if on Raspberry Pi with a display
        if self.is_raspberry_pi:
            try:
                # Check for Raspberry Pi display drivers
                return os.path.exists("/dev/fb0")
            except:
                pass

        return False

    def get_capabilities(self):
        """Get a dictionary of available hardware capabilities."""
        return {
            "platform": self.platform,
            "is_raspberry_pi": self.is_raspberry_pi,
            "has_gpio": self.has_gpio,
            "has_audio": self.has_audio,
            "has_bluetooth": self.has_bluetooth,
            "has_display": self.has_display
        }


# Create a singleton instance
_hardware_detector = None


def get_hardware_detector():
    """Get the hardware detector instance."""
    global _hardware_detector
    if _hardware_detector is None:
        _hardware_detector = HardwareDetector()
    return _hardware_detector