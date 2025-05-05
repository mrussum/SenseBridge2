# file: src/mock/pil.py
"""Mock PIL module for development."""


class Image:
    @staticmethod
    def open(path):
        print(f"[MOCK] Opening image from {path}")
        return MockImage()


class MockImage:
    def __init__(self):
        self.width = 800
        self.height = 600

    def resize(self, size):
        print(f"[MOCK] Resizing image to {size}")
        return self


class ImageTk:
    @staticmethod
    def PhotoImage(image):
        print("[MOCK] Creating PhotoImage")
        return "MOCK_PHOTO_IMAGE"