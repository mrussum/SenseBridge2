# file: create_init_files.py
import os


def create_init_file(directory):
    init_path = os.path.join(directory, "__init__.py")
    if not os.path.exists(init_path):
        try:
            with open(init_path, 'w') as f:
                pass  # Create empty file
            print(f"Created {init_path}")
        except Exception as e:
            print(f"Error creating {init_path}: {e}")
    else:
        print(f"Already exists: {init_path}")


def main():
    print("Creating __init__.py files...")

    # Create in src directory
    src_path = os.path.join(os.getcwd(), "src")
    if os.path.exists(src_path):
        create_init_file(src_path)

        # Create in subdirectories
        for subdir in ["audio", "speech", "models", "notification", "hardware", "gui", "utils"]:
            subdir_path = os.path.join(src_path, subdir)
            if os.path.exists(subdir_path):
                create_init_file(subdir_path)


if __name__ == "__main__":
    main()