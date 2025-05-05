# file: use_mocks.py
import sys
import os


def setup_mocks():
    """Set up mock modules for testing."""
    # Add the mock directory to Python's path
    mock_dir = os.path.join(os.getcwd(), "src", "mock")
    if os.path.exists(mock_dir) and mock_dir not in sys.path:
        sys.path.insert(0, mock_dir)

    # Define which modules to mock
    modules_to_mock = [
        'tensorflow',
        'bluetooth',
        'PIL'
    ]

    # Insert mock modules into sys.modules
    import importlib.util
    for module_name in modules_to_mock:
        mock_path = os.path.join(mock_dir, f"{module_name}.py")
        if os.path.exists(mock_path):
            spec = importlib.util.spec_from_file_location(module_name, mock_path)
            mock_module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mock_module)
            sys.modules[module_name] = mock_module
            print(f"Mocked module: {module_name}")

    print("Mock modules set up successfully")


if __name__ == "__main__":
    setup_mocks()