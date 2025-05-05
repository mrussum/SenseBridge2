"""
Haptic feedback module for SenseBridge.
Controls vibration patterns for wearable device.
"""

import threading
import time
import logging
import queue
import json
import socket
import bluetooth
from src.utils.config import Config
from ..hardware.device_control import activate_device, deactivate_device

logger = logging.getLogger(__name__)


class HapticFeedback:
    """Controls haptic feedback for wearable devices."""

    def __init__(self):
        """Initialize the haptic feedback system."""
        self.config = Config()
        self.device_config = self.config.get_device_config()

        # Get hardware configuration
        self.hardware_config = self.device_config["hardware"]
        self.haptic_pin = self.hardware_config["haptic_pin"]

        # Get Bluetooth configuration
        self.bluetooth_config = self.device_config["bluetooth"]
        self.wearable_mac = self.bluetooth_config.get("wearable_mac", "")

        # Use Bluetooth if MAC address is configured, otherwise use GPIO
        self.use_bluetooth = bool(self.wearable_mac)

        # Vibration patterns (duration in seconds)
        self.patterns = {
            "short_single": [(0.2, 1.0)],  # (duration, intensity)
            "short_double": [(0.2, 1.0), (0.1, 0.0), (0.2, 1.0)],
            "short_triple": [(0.2, 1.0), (0.1, 0.0), (0.2, 1.0), (0.1, 0.0), (0.2, 1.0)],
            "long_single": [(0.8, 1.0)],
            "long_double": [(0.8, 1.0), (0.2, 0.0), (0.8, 1.0)],
            "continuous": [(3.0, 1.0)],
            "escalating": [(0.2, 0.3), (0.1, 0.0), (0.2, 0.6), (0.1, 0.0), (0.3, 1.0)]
        }

        # Queue for haptic commands
        self.command_queue = queue.Queue()

        # Thread for processing haptic commands
        self.haptic_thread = None
        self.running = False

        # Bluetooth socket for wearable communication
        self.bt_socket = None

        logger.info("HapticFeedback initialized")

    def start(self):
        """Start the haptic feedback system."""
        if self.running:
            logger.warning("Haptic feedback system already running")
            return

        self.running = True

        # Connect to Bluetooth wearable if configured
        if self.use_bluetooth:
            self._connect_bluetooth()

        # Start haptic processing thread
        self.haptic_thread = threading.Thread(target=self._haptic_loop)
        self.haptic_thread.daemon = True
        self.haptic_thread.start()

        logger.info("Haptic feedback system started")

    def stop(self):
        """Stop the haptic feedback system."""
        if not self.running:
            return

        self.running = False

        # Stop haptic thread
        if self.haptic_thread:
            self.haptic_thread.join(timeout=2.0)

        # Disconnect Bluetooth
        if self.bt_socket:
            try:
                self.bt_socket.close()
            except:
                pass
            self.bt_socket = None

        logger.info("Haptic feedback system stopped")

    def send_feedback(self, pattern_name="short_double", priority="medium"):
        """Send a haptic feedback pattern.

        Args:
            pattern_name: Name of the vibration pattern
            priority: Priority level ("low", "medium", "high")
        """
        # Get pattern, default to short_double if not found
        pattern = self.patterns.get(pattern_name, self.patterns["short_double"])

        # Add to command queue with priority
        priority_level = {"low": 0, "medium": 1, "high": 2}.get(priority, 1)

        self.command_queue.put({
            "pattern": pattern,
            "priority": priority_level,
            "timestamp": time.time()
        })

        logger.debug(f"Haptic feedback queued: {pattern_name} (priority: {priority})")

    def _haptic_loop(self):
        """Main haptic processing loop."""
        last_haptic_time = 0

        while self.running:
            try:
                # Get next command from queue with timeout
                try:
                    command = self.command_queue.get(timeout=0.5)
                except queue.Empty:
                    continue

                pattern = command["pattern"]
                priority = command["priority"]
                timestamp = command["timestamp"]

                # Skip low priority commands if too recent (within 5 seconds)
                if priority < 2 and time.time() - last_haptic_time < 5.0:
                    logger.debug("Skipping low priority haptic feedback")
                    continue

                # Execute vibration pattern
                self._execute_pattern(pattern)

                # Update last haptic time
                last_haptic_time = time.time()

            except Exception as e:
                logger.error(f"Error in haptic loop: {str(e)}")

    def _execute_pattern(self, pattern):
        """Execute a vibration pattern.

        Args:
            pattern: List of (duration, intensity) tuples
        """
        try:
            logger.debug(f"Executing haptic pattern: {pattern}")

            for duration, intensity in pattern:
                if not self.running:
                    break

                if intensity > 0:
                    # Activate vibration
                    if self.use_bluetooth:
                        self._send_bluetooth_command("vibrate", {
                            "intensity": intensity,
                            "duration": int(duration * 1000)
                        })
                    else:
                        # Use GPIO for local vibration motor
                        activate_device(f"haptic_pin_{self.haptic_pin}")

                    # Wait for duration
                    time.sleep(duration)

                    # Deactivate vibration if not using Bluetooth
                    if not self.use_bluetooth:
                        deactivate_device(f"haptic_pin_{self.haptic_pin}")
                else:
                    # Just wait for the off period
                    time.sleep(duration)

        except Exception as e:
            logger.error(f"Error executing haptic pattern: {str(e)}")

    def _connect_bluetooth(self):
        """Connect to the Bluetooth wearable device."""
        if not self.wearable_mac:
            logger.warning("No wearable MAC address configured")
            return False

        try:
            logger.info(f"Connecting to wearable at {self.wearable_mac}...")

            # Scan for nearby devices (to check if wearable is available)
            nearby_devices = bluetooth.discover_devices(duration=2, lookup_names=True)
            device_found = any(addr == self.wearable_mac for addr, name in nearby_devices)

            if not device_found:
                logger.warning(f"Wearable device {self.wearable_mac} not found nearby")
                return False

            # Create socket and connect
            self.bt_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            self.bt_socket.connect((self.wearable_mac, 1))  # RFCOMM channel 1

            logger.info("Connected to wearable device")
            return True

        except Exception as e:
            logger.error(f"Error connecting to Bluetooth wearable: {str(e)}")
            self.bt_socket = None
            return False

    def _send_bluetooth_command(self, command, params=None):
        """Send a command to the Bluetooth wearable.

        Args:
            command: Command name
            params: Command parameters
        """
        if not self.bt_socket:
            if not self._connect_bluetooth():
                logger.error("Cannot send command - Bluetooth not connected")
                return False

        try:
            # Create command JSON
            cmd_data = {
                "cmd": command,
                "params": params or {}
            }

            # Send command
            self.bt_socket.send(json.dumps(cmd_data) + "\n")
            return True

        except Exception as e:
            logger.error(f"Error sending Bluetooth command: {str(e)}")
            # Try to reconnect next time
            self.bt_socket = None
            return False


def send_haptic_feedback():
    """Legacy function for compatibility with existing code."""
    # This is just a placeholder
    # The actual implementation uses the HapticFeedback class
    activate_device("haptic")
    time.sleep(0.3)
    deactivate_device("haptic")