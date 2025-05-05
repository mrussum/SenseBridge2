# file: src/mock/paho/mqtt.py
"""Mock MQTT client module for simulation testing."""


class Client:
    def __init__(self, client_id="", clean_session=True, userdata=None, protocol=4, transport="tcp"):
        self.client_id = client_id
        self.clean_session = clean_session
        self.userdata = userdata
        self.protocol = protocol
        self.transport = transport

        # Event callbacks
        self.on_connect = None
        self.on_disconnect = None
        self.connected = False

    def username_pw_set(self, username, password=None):
        """Set username and password for broker authentication."""
        print(f"[MOCK] Setting username {username} and password")

    def connect(self, host, port=1883, keepalive=60):
        """Connect to an MQTT broker."""
        print(f"[MOCK] Connecting to MQTT broker at {host}:{port}")
        self.connected = True
        # Trigger on_connect callback if set
        if self.on_connect:
            self.on_connect(self, None, None, 0)
        return 0

    def disconnect(self):
        """Disconnect from the broker."""
        print("[MOCK] Disconnecting from MQTT broker")
        self.connected = False
        # Trigger on_disconnect callback if set
        if self.on_disconnect:
            self.on_disconnect(self, None, 0)

    def loop_start(self):
        """Start the MQTT client loop."""
        print("[MOCK] Starting MQTT client loop")

    def loop_stop(self):
        """Stop the MQTT client loop."""
        print("[MOCK] Stopping MQTT client loop")

    def publish(self, topic, payload=None, qos=0, retain=False):
        """Publish a message to a topic."""
        print(f"[MOCK] Publishing to topic {topic}: {payload}")
        return 0

    def subscribe(self, topic, qos=0):
        """Subscribe to a topic."""
        print(f"[MOCK] Subscribing to topic {topic} with QoS {qos}")
        return (0, 0)