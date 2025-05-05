# file: test/test_config.py
import os
import sys
# Import helper to fix path
from test_helper import *


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


if __name__ == "__main__":
    print("\n=== Testing SenseBridge Configuration ===\n")
    test_config()
    print("\nConfiguration test completed.")