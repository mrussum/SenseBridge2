# fix_structure.py
import os
import shutil


def create_directory(path):
    if not os.path.exists(path):
        os.makedirs(path)
        print(f"Created directory: {path}")


def main():
    # Ensure all required directories exist
    directories = [
        "src",
        "src/audio",
        "src/speech",
        "src/notification",
        "src/models",
        "src/hardware",
        "src/gui",
        "src/utils",
        "config",
        "models/yamnet_model",
        "logs",
        "tests"
    ]

    for directory in directories:
        create_directory(directory)

    # Move files from duplicate directories to the correct location
    # Example: Move files from src/communication/gui to src/gui if needed
    if os.path.exists("src/communication/gui"):
        for file in os.listdir("src/communication/gui"):
            src_path = os.path.join("src/communication/gui", file)
            dst_path = os.path.join("src/gui", file)
            if os.path.isfile(src_path) and not os.path.exists(dst_path):
                shutil.copy2(src_path, dst_path)
                print(f"Copied {src_path} to {dst_path}")

    # Similar for other duplicate/misplaced directories

    # Create missing __init__.py files
    for directory in directories:
        init_file = os.path.join(directory, "__init__.py")
        if not os.path.exists(init_file):
            open(init_file, 'a').close()
            print(f"Created {init_file}")


if __name__ == "__main__":
    main()