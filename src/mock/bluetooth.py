# file: src/mock/bluetooth.py
"""Mock Bluetooth module for development on systems without PyBluez."""


def discover_devices(duration=8, lookup_names=False):
    """Simulates device discovery."""
    print(f"[MOCK] Discovering Bluetooth devices for {duration} seconds...")
    # Return a list of mock device tuples (addr, name)
    return [("00:11:22:33:44:55", "MockWearable")]


class BluetoothSocket:
    def __init__(self, protocol):
        print(f"[MOCK] Creating Bluetooth socket with protocol {protocol}")
        self.connected = False

    def connect(self, address_tuple):
        addr, port = address_tuple
        print(f"[MOCK] Connecting to {addr} on port {port}")
        self.connected = True

    def send(self, data):
        print(f"[MOCK] Sending data: {data[:20]}...")
        return len(data)

    def recv(self, size):
        import json
        # Return mock response
        resp = json.dumps({"status": "ok"}).encode()
        return resp

    def close(self):
        print("[MOCK] Closing Bluetooth connection")
        self.connected = False

    def settimeout(self, timeout):
        print(f"[MOCK] Setting socket timeout to {timeout}")


# Mock Bluetooth error
class btcommon:
    class BluetoothError(Exception):
        pass