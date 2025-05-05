# file: src/mock/tensorflow.py
"""Mock TensorFlow module for development on systems without TensorFlow."""

class lite:
    class Interpreter:
        def __init__(self, model_path=None):
            print(f"[MOCK] Loading TensorFlow model from {model_path}")
            self.model_path = model_path

        def allocate_tensors(self):
            print("[MOCK] Allocating tensors")

        def get_input_details(self):
            return [{'index': 0, 'shape': [1, 16000]}]

        def get_output_details(self):
            return [{'index': 0, 'shape': [1, 521]}]

        def set_tensor(self, index, tensor):
            pass

        def invoke(self):
            pass

        def get_tensor(self, index):
            import numpy as np
            # Return mock prediction data
            return np.array([[0.1, 0.8, 0.05, 0.05]])