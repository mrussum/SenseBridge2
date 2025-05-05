# file: test/test_helper.py
import os
import sys

# Add the parent directory to the Python path
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
    print(f"Added {parent_dir} to Python path")

# Add mock directory to path
mock_dir = os.path.join(parent_dir, "src", "mock")
if os.path.exists(mock_dir) and mock_dir not in sys.path:
    sys.path.insert(0, mock_dir)

# Import and set up mocks
try:
    import tensorflow
    import bluetooth
    import PIL


    # Create a simple mock MQTT client class
    class MockClient:
        def __init__(self, client_id="", clean_session=True, userdata=None, protocol=4, transport="tcp"):
            self.client_id = client_id
            self.on_connect = None
            self.on_disconnect = None

        def username_pw_set(self, username, password=None):
            pass

        def connect(self, host, port=1883, keepalive=60):
            if self.on_connect:
                self.on_connect(self, None, None, 0)
            return 0

        def disconnect(self):
            if self.on_disconnect:
                self.on_disconnect(self, None, 0)

        def loop_start(self):
            pass

        def loop_stop(self):
            pass

        def publish(self, topic, payload=None, qos=0, retain=False):
            return 0

        def subscribe(self, topic, qos=0):
            return (0, 0)


    # The key fix: directly mock the specific import used in the code
    sys.modules['paho'] = type('paho', (), {})
    sys.modules['paho.mqtt'] = type('mqtt', (), {})
    sys.modules['paho.mqtt.client'] = MockClient

    print("Mock modules imported successfully")
except ImportError:
    print("Warning: Mock modules not found")