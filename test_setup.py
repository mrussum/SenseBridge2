def test_imports():
    print("Testing imports...")

    # Test core libraries
    try:
        import numpy
        print("✓ NumPy imported successfully")
    except ImportError as e:
        print(f"✗ NumPy import failed: {e}")

    try:
        import scipy
        print("✓ SciPy imported successfully")
    except ImportError as e:
        print(f"✗ SciPy import failed: {e}")

    # Test audio processing
    try:
        import pyaudio
        print("✓ PyAudio imported successfully")
    except ImportError as e:
        print(f"✗ PyAudio import failed: {e}")

    # Test more imports
    try:
        import tensorflow
        print("✓ TensorFlow imported successfully")
    except ImportError as e:
        print(f"✗ TensorFlow import failed: {e}")

    print("\nBasic import test complete!")


if __name__ == "__main__":
    test_imports()