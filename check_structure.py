# file: check_structure.py
import os


def check_directory(path, name):
    exists = os.path.exists(path)
    print(f"{'✓' if exists else '✗'} {name}: {path}")
    return exists


def main():
    print("\nChecking SenseBridge project structure...")

    # Check main directories
    root = os.getcwd()
    check_directory(os.path.join(root, "src"), "Source directory")
    check_directory(os.path.join(root, "config"), "Config directory")
    check_directory(os.path.join(root, "models"), "Models directory")
    check_directory(os.path.join(root, "tests"), "Tests directory")

    # Check source subdirectories
    src_path = os.path.join(root, "src")
    if os.path.exists(src_path):
        print("\nChecking source subdirectories:")
        check_directory(os.path.join(src_path, "audio"), "Audio module")
        check_directory(os.path.join(src_path, "speech"), "Speech module")
        check_directory(os.path.join(src_path, "models"), "Models module")
        check_directory(os.path.join(src_path, "notification"), "Notification module")
        check_directory(os.path.join(src_path, "hardware"), "Hardware module")
        check_directory(os.path.join(src_path, "gui"), "GUI module")
        check_directory(os.path.join(src_path, "utils"), "Utils module")

    # Check for __init__.py files
    print("\nChecking for __init__.py files:")
    if os.path.exists(src_path):
        check_directory(os.path.join(src_path, "__init__.py"), "src/__init__.py")
        for subdir in ["audio", "speech", "models", "notification", "hardware", "gui", "utils"]:
            subdir_path = os.path.join(src_path, subdir)
            if os.path.exists(subdir_path):
                check_directory(os.path.join(subdir_path, "__init__.py"), f"src/{subdir}/__init__.py")


if __name__ == "__main__":
    main()