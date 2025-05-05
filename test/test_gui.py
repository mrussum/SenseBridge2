# Add at the top
import os
import sys
# Import helper to fix path
from test_helper import *

def test_gui_interface():
    # Rest of the code remains the same...
    try:
        # Just import the module to see if it loads correctly
        from src.gui.interface import show_alert, show_speech_text, update_status

        print("✓ GUI interface modules imported successfully")

        # Don't actually initialize GUI to avoid tkinter window popping up
        print("✓ GUI test complete (import only)")
        return True
    except Exception as e:
        print(f"✗ GUI test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    print("\n=== Testing SenseBridge GUI Components ===\n")

    print("Testing GUI interface...")
    test_gui_interface()

    print("\nGUI tests completed.")