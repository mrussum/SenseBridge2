# file: test_config.py
def test_config():
    try:
        from src.utils.config import Config
        print("✓ Config module imported successfully")

        config = Config()
        print("✓ Config class instantiated successfully")

        device_config = config.get_device_config()
        print(f"✓ Device config loaded")
        print(f"  Sample audio settings: {device_config.get('audio', {})}")

        return True
    except Exception as e:
        print(f"✗ Config test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    print("Testing SenseBridge configuration...")
    test_config()