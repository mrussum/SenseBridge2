# fix_imports.py
import os
import re


def fix_imports_in_file(file_path):
    with open(file_path, 'r') as file:
        content = file.read()

    # Fix common import patterns
    # Example: Change from ..integration.models.sound_classifier to ..models.sound_classifier
    content = content.replace("..integration.models", "..models")
    content = content.replace("..communication.gui", "..gui")
    content = content.replace("..management.logger", "..utils.logger")
    content = content.replace("src.with TF", "src")

    # Write back the fixed content
    with open(file_path, 'w') as file:
        file.write(content)
    print(f"Fixed imports in {file_path}")


def main():
    # Walk through all Python files in src directory
    for root, _, files in os.walk("src"):
        for file in files:
            if file.endswith(".py"):
                fix_imports_in_file(os.path.join(root, file))


if __name__ == "__main__":
    main()