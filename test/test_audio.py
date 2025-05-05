# Add at the top

import os
import sys

# Import helper to fix path

from test_helper import *

def test_audio_processor():
    # Rest of the code remains the same...
    try:
        from src.audio.audio_processor import AudioProcessor

        # Initialize without starting capture
        processor = AudioProcessor()
        print("✓ AudioProcessor initialized")

        # Don't actually start the processor to avoid audio device conflicts
        print("✓ AudioProcessor test complete (simulation mode)")
        return True
    except Exception as e:
        print(f"✗ AudioProcessor test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_sound_recognition():
    try:
        from src.audio.sound_recognition import SoundRecognition

        # Test callback
        def test_callback(sound_type, confidence, audio_data):
            print(f"  Callback received: {sound_type} (confidence: {confidence:.2f})")

        # Initialize without starting detection
        recognition = SoundRecognition(callback=test_callback)
        print("✓ SoundRecognition initialized")

        # Don't actually start recognition to avoid audio device conflicts
        print("✓ SoundRecognition test complete (simulation mode)")
        return True
    except Exception as e:
        print(f"✗ SoundRecognition test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    print("\n=== Testing SenseBridge Audio Components ===\n")

    print("1. Testing audio processor...")
    test_audio_processor()

    print("\n2. Testing sound recognition...")
    test_sound_recognition()

    print("\nAudio tests completed.")