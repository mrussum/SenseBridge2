# file: test_notification.py
def test_notification_manager():
    try:
        from src.notification.notification_manager import NotificationManager

        # Initialize without starting
        manager = NotificationManager()
        print("✓ NotificationManager initialized")

        # Check event types
        events = manager.sound_events
        print(f"✓ Sound events loaded: {list(events.keys())}")

        print("✓ NotificationManager test complete (simulation mode)")
        return True
    except Exception as e:
        print(f"✗ NotificationManager test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    print("\n=== Testing SenseBridge Notification Components ===\n")

    print("Testing notification manager...")
    test_notification_manager()

    print("\nNotification tests completed.")