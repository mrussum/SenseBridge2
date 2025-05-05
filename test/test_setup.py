def test_imports():
    print("Testing imports...")

    # Test core libraries
    import numpy
    print("✓ NumPy imported successfully")

    import scipy
    print("✓ SciPy imported successfully")

    # Test audio processing
    import pyaudio
    print("✓ PyAudio imported successfully")

    # Optional: test more imports
    try:
        import tensorflow
        print("✓ TensorFlow imported successfully")
    except ImportError:
        print("⚠ TensorFlow import failed - not critical for initial testing")

    print("\nAll critical imports successful!")


if __name__ == "__main__":
    test_imports()