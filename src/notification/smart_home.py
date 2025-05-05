"""
Smart home integration module for SenseBridge.
Connects to smart home devices for additional notification options.
"""

import threading
import time
import logging
import json
import paho.mqtt.client as mqtt
from ..utils.config import Config

logger = logging.getLogger(__name__)


class SmartHomeIntegration:
    """Connects to smart home devices for notifications."""

    def __init__(self):
        """Initialize the smart home integration."""
        self.config = Config()
        self.user_prefs = self.config.get_user_preferences()

        # Get smart home configuration
        self.smart_home_config = self.user_prefs["smart_home"]
        self.mqtt_broker = self.smart_home_config["mqtt_broker"]
        self.mqtt_port = self.smart_home_config["mqtt_port"]
        self.mqtt_username = self.smart_home_config["mqtt_username"]
        self.mqtt_password = self.smart_home_config["mqtt_password"]
        self.light_topic = self.smart_home_config["light_topic"]

        # MQTT client
        self.mqtt_client = None
        self.connected = False

        # Thread for MQTT client
        self.mqtt_thread = None
        self.running = False

        logger.info("SmartHomeIntegration initialized")

    def start(self):
        """Start the smart home integration."""
        if self.running:
            logger.warning("Smart home integration already running")
            return

        self.running = True

        # Start MQTT client if broker is configured
        if self.mqtt_broker:
            self.mqtt_thread = threading.Thread(target=self._mqtt_loop)
            self.mqtt_thread.daemon = True
            self.mqtt_thread.start()
            logger.info("Smart home integration started")
        else:
            logger.info("Smart home integration disabled (no MQTT broker configured)")

    def stop(self):
        """Stop the smart home integration."""
        if not self.running:
            return

        self.running = False

        # Disconnect MQTT client
        if self.mqtt_client and self.connected:
            try:
                self.mqtt_client.disconnect()
            except:
                pass

        # Stop MQTT thread
        if self.mqtt_thread:
            self.mqtt_thread.join(timeout=2.0)

        logger.info("Smart home integration stopped")

    def trigger_notification(self, event_type, priority="medium"):
        """Trigger a smart home notification for the given event.

        Args:
            event_type: Type of event (e.g., "doorbell", "knock")
            priority: Priority level ("low", "medium", "high")
        """
        if not self.connected or not self.mqtt_client:
            logger.warning("Cannot send notification - MQTT not connected")
            return False

        try:
            # Create notification message
            priority_level = {"low": 0, "medium": 1, "high": 2}.get(priority, 1)

            message = {
                "event": event_type,
                "priority": priority,
                "timestamp": time.time()
            }

            # Send message to light topic
            self._send_light_command(event_type, priority_level)

            # Send general event notification
            topic = f"senseBridge/events/{event_type}"
            self.mqtt_client.publish(topic, json.dumps(message))

            logger.debug(f"Smart home notification sent: {event_type}")
            return True

        except Exception as e:
            logger.error(f"Error sending smart home notification: {str(e)}")
            return False

    def _mqtt_loop(self):
        """Main MQTT client loop."""
        while self.running:
            try:
                if not self.connected:
                    self._connect_mqtt()

                # Sleep to avoid busy waiting
                time.sleep(1.0)

            except Exception as e:
                logger.error(f"Error in MQTT loop: {str(e)}")
                self.connected = False
                time.sleep(5.0)  # Wait before reconnecting

    def _connect_mqtt(self):
        """Connect to the MQTT broker."""
        if not self.mqtt_broker:
            return False

        try:
            logger.info(f"Connecting to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}...")

            # Create new client instance
            client_id = f"senseBridge-{time.time()}"
            self.mqtt_client = mqtt.Client(client_id)

            # Set username and password if configured
            if self.mqtt_username and self.mqtt_password:
                self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)

            # Set callbacks
            self.mqtt_client.on_connect = self._on_connect
            self.mqtt_client.on_disconnect = self._on_disconnect

            # Connect to broker
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)

            # Start the loop
            self.mqtt_client.loop_start()

            # Wait for connection
            timeout = time.time() + 10.0
            while not self.connected and time.time() < timeout:
                time.sleep(0.1)

            if not self.connected:
                logger.warning("MQTT connection timed out")
                self.mqtt_client.loop_stop()
                return False

            return True

        except Exception as e:
            logger.error(f"Error connecting to MQTT broker: {str(e)}")
            self.connected = False
            return False

    def _on_connect(self, client, userdata, flags, rc):
        """Callback for when the client connects to the broker."""
        if rc == 0:
            logger.info("Connected to MQTT broker")
            self.connected = True

            # Subscribe to control topics
            self.mqtt_client.subscribe("senseBridge/control/#")
        else:
            logger.warning(f"Failed to connect to MQTT broker with code {rc}")
            self.connected = False

    def _on_disconnect(self, client, userdata, rc):
        """Callback for when the client disconnects from the broker."""
        logger.info("Disconnected from MQTT broker")
        self.connected = False

    def _send_light_command(self, event_type, priority_level):
        """Send a command to smart lights based on event.

        Args:
            event_type: Type of event
            priority_level: Priority level (0-2)
        """
        # Define light effects based on event and priority
        effects = {
            "doorbell": {
                0: {"color": "blue", "brightness": 150, "flash": 1},
                1: {"color": "blue", "brightness": 200, "flash": 2},
                2: {"color": "blue", "brightness": 255, "flash": 3}
            },
            "knock": {
                0: {"color": "green", "brightness": 150, "flash": 1},
                1: {"color": "green", "brightness": 200, "flash": 2},
                2: {"color": "green", "brightness": 255, "flash": 3}
            },
            "alarm": {
                0: {"color": "red", "brightness": 200, "flash": 2},
                1: {"color": "red", "brightness": 225, "flash": 3},
                2: {"color": "red", "brightness": 255, "flash": 5}
            },
            "microwave_beep": {
                0: {"color": "yellow", "brightness": 150, "flash": 1},
                1: {"color": "yellow", "brightness": 200, "flash": 2},
                2: {"color": "yellow", "brightness": 225, "flash": 3}
            }
        }

        # Get effect for this event type and priority
        if event_type in effects and priority_level in effects[event_type]:
            effect = effects[event_type][priority_level]
        else:
            # Default effect
            effect = {"color": "white", "brightness": 200, "flash": 2}

        # Create command
        command = {
            "state": "ON",
            "color": effect["color"],
            "brightness": effect["brightness"],
            "flash": effect["flash"],
            "transition": 0.5
        }

        # Publish command to light topic
        self.mqtt_client.publish(self.light_topic, json.dumps(command))
        logger.debug(f"Light command sent: {command}")