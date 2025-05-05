# file: test_sensebridge_core.py
import os
import json


def test_config():
    try:
        from src.utils.config import Config
        config = Config()

        # Test loading configurations
        device_config = config.get_device_config()
        sound_events = config.get_sound_events()
        user_prefs = config.get_user_preferences()

        print(f"✓ Config loaded successfully")
        print(f"  - Device config has {len(device_config)} sections")
        print(f"  - Sound events: {list(sound_events.keys())}")
        print(f"  - User prefs has {len(user_prefs)} sections")

        return True
    except Exception as e:
        print(f"✗ Config test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_hardware_mock():
    try:
        from src.hardware.device_control import DeviceController

        controller = DeviceController()
        print(f"✓ DeviceController initialized (simulation mode)")

        # Test activating a device (should work in simulation mode)
        success = controller.activate_device("test_device")
        print(f"{'✓' if success else '✗'} Device activation test")

        return True
    except Exception as e:
        print(f"✗ Hardware test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    print("\n=== Testing SenseBridge Core Components ===\n")

    print("1. Testing configuration...")
    test_config()

    print("\n2. Testing hardware (simulation mode)...")
    test_hardware_mock()

    print("\nTests completed.")