# src/hardware/wearable.py
"""
Wearable device module for SenseBridge.
Manages communication with Bluetooth wearable devices.
"""

import logging
import threading
import time
import json

logger = logging.getLogger(__name__)

# Try to import Bluetooth, fall back to mock if unavailable
try:
    import bluetooth
except ImportError:
    try:
        from ..mock.bluetooth import *

        logger.warning("Using mock Bluetooth module")
    except ImportError:
        logger.error("Bluetooth module not available")
        bluetooth = None

from ..utils.config import Config


class WearableDevice:
    """Manages communication with a Bluetooth wearable device."""

    def __init__(self):
        """Initialize the wearable device manager."""
        self.config = Config()
        self.device_config = self.config.get_device_config()

        # Get Bluetooth configuration
        self.bluetooth_config = self.device_config["bluetooth"]
        self.device_name = self.bluetooth_config["device_name"]
        self.wearable_mac = self.bluetooth_config.get("wearable_mac", "")

        # Connection status
        self.connected = False
        self.socket = None

        # Thread for connection maintenance
        self.connection_thread = None
        self.running = False

        logger.info("WearableDevice initialized")

    def start(self):
        """Start the wearable device manager."""
        if self.running:
            logger.warning("Wearable device manager already running")
            return

        if not bluetooth:
            logger.warning("Bluetooth not available, running in simulation mode")
            self.running = True
            return

        self.running = True

        # Start connection thread
        self.connection_thread = threading.Thread(target=self._connection_loop)
        self.connection_thread.daemon = True
        self.connection_thread.start()

        logger.info("Wearable device manager started")

    def stop(self):
        """Stop the wearable device manager."""
        if not self.running:
            return

        self.running = False

        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None

        if self.connection_thread:
            self.connection_thread.join(timeout=2.0)

        logger.info("Wearable device manager stopped")

    def send_command(self, command, params=None):
        """Send a command to the wearable device.

        Args:
            command: Command name
            params: Command parameters

        Returns:
            True if successful, False otherwise
        """
        if not self.connected or not self.socket:
            logger.warning("Cannot send command - not connected to wearable")
            return False

        try:
            # Create command JSON
            cmd_data = {
                "cmd": command,
                "params": params or {}
            }

            # Send command
            self.socket.send(json.dumps(cmd_data).encode() + b"\n")
            return True

        except Exception as e:
            logger.error(f"Error sending command to wearable: {str(e)}")
            self.connected = False
            return False

    def _connection_loop(self):
        """Main connection maintenance loop."""
        while self.running:
            try:
                if not self.connected:
                    self._connect()

                # Sleep to avoid busy waiting
                time.sleep(5.0)

            except Exception as e:
                logger.error(f"Error in wearable connection loop: {str(e)}")
                self.connected = False
                time.sleep(10.0)  # Wait longer on error

    def _connect(self):
        """Connect to the wearable device."""
        if not self.wearable_mac:
            logger.warning("No wearable MAC address configured")
            return False

        try:
            logger.info(f"Connecting to wearable at {self.wearable_mac}...")

            # Create socket and connect
            self.socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            self.socket.connect((self.wearable_mac, 1))  # RFCOMM channel 1
            self.socket.settimeout(5.0)

            # Send hello message
            hello_msg = {"cmd": "hello", "params": {"name": "SenseBridge"}}
            self.socket.send(json.dumps(hello_msg).encode() + b"\n")

            # Wait for response
            try:
                response = self.socket.recv(1024)
                logger.info(f"Wearable response: {response}")
            except:
                pass

            self.connected = True
            logger.info("Connected to wearable device")
            return True

        except Exception as e:
            logger.error(f"Error connecting to wearable: {str(e)}")
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass
                self.socket = None
            self.connected = False
            return False