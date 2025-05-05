"""
Sound classification module for SenseBridge.
Uses TensorFlow to classify environmental sounds.
"""

import os
import numpy as np
import tensorflow as tf
import logging
import time
from ..utils.config import Config

logger = logging.getLogger(__name__)


class SoundClassifier:
    """Classifies environmental sounds using a TensorFlow model."""

    def __init__(self):
        """Initialize the sound classifier."""
        self.config = Config()
        self.device_config = self.config.get_device_config()
        self.sound_events = self.config.get_sound_events()

        self.model = None
        self.labels = []
        self.model_loaded = False

        # Audio processing parameters
        self.sample_rate = self.device_config["audio"]["sample_rate"]
        self.waveform_duration = 0.975  # YAMNet expects 0.975 seconds of audio

        # Target classes that we want to detect
        self.target_classes = {
            "doorbell": ["doorbell", "ding-dong"],
            "knock": ["knock", "tap", "tapping"],
            "alarm": ["alarm", "alarm clock", "siren", "smoke detector", "fire alarm"],
            "microwave_beep": ["microwave oven", "beep", "microwave", "oven", "electronic beep"],
            "phone_ring": ["telephone", "ringtone", "phone", "telephone bell ringing"],
            "baby_cry": ["crying baby", "baby cry", "child cry"],
            "dog_bark": ["dog", "bark", "dog bark"],
            "car_horn": ["car horn", "honking", "vehicle horn"]
        }

        logger.info("SoundClassifier initialized")

    def load_model(self):
        """Load the TensorFlow model for sound classification."""
        if self.model_loaded:
            return

        try:
            start_time = time.time()
            logger.info("Loading sound classification model...")

            # Get model path
            model_path = os.path.join("models", "yamnet_model", "yamnet.tflite")
            labels_path = os.path.join("models", "yamnet_model", "yamnet_labels.txt")

            # Load TFLite model
            self.interpreter = tf.lite.Interpreter(model_path=model_path)
            self.interpreter.allocate_tensors()

            # Get model details
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()

            # Load class labels
            with open(labels_path, 'r') as f:
                self.labels = [line.strip() for line in f]

            logger.info(f"Model loaded in {time.time() - start_time:.2f} seconds")
            logger.info(f"Model has {len(self.labels)} sound classes")

            self.model_loaded = True

        except Exception as e:
            logger.error(f"Error loading sound model: {str(e)}")
            # Fallback to a dummy classifier
            self.model_loaded = False

    def classify_sound(self, audio_data):
        """Classify the audio data using the TensorFlow model.

        Args:
            audio_data: Audio data as numpy array

        Returns:
            Tuple of (sound_type, confidence)
        """
        if not self.model_loaded:
            self.load_model()
            if not self.model_loaded:
                # Fallback to simple classification if model couldn't be loaded
                return self._classify_fallback(audio_data)

        try:
            # Prepare audio for the model (reshape to expected duration)
            expected_samples = int(self.sample_rate * self.waveform_duration)

            # If audio is too short, pad with zeros
            if len(audio_data) < expected_samples:
                padding = expected_samples - len(audio_data)
                audio_data = np.pad(audio_data, (0, padding), 'constant')

            # If audio is too long, take the middle section
            elif len(audio_data) > expected_samples:
                start = (len(audio_data) - expected_samples) // 2
                audio_data = audio_data[start:start + expected_samples]

            # Ensure the audio is the right shape for the model
            audio_data = audio_data.reshape(1, -1).astype(np.float32)

            # Set the input tensor
            self.interpreter.set_tensor(self.input_details[0]['index'], audio_data)

            # Run inference
            self.interpreter.invoke()

            # Get predictions
            scores = self.interpreter.get_tensor(self.output_details[0]['index'])
            scores = scores[0]

            # Map scores to labels and find the highest confidence for target classes
            best_match = None
            best_confidence = 0.0

            for target_class, keywords in self.target_classes.items():
                for keyword in keywords:
                    # Find partial matches in labels
                    for i, label in enumerate(self.labels):
                        if keyword.lower() in label.lower():
                            confidence = float(scores[i])
                            if confidence > best_confidence:
                                best_confidence = confidence
                                best_match = target_class

            # If nothing detected with confidence, return unknown
            if best_match is None or best_confidence < 0.3:
                return "unknown", 0.0

            return best_match, best_confidence

        except Exception as e:
            logger.error(f"Error in sound classification: {str(e)}")
            return self._classify_fallback(audio_data)

    def _classify_fallback(self, audio_data):
        """Simple fallback classification when the model isn't available.

        Args:
            audio_data: Audio data as numpy array

        Returns:
            Tuple of (sound_type, confidence)
        """
        # Calculate basic audio features
        audio_level = np.abs(audio_data).mean()
        zero_crossings = np.sum(np.abs(np.diff(np.signbit(audio_data)))) / len(audio_data)

        # Simple heuristic classification
        if audio_level > 0.4 and zero_crossings > 0.1:
            # High-frequency sound with high energy (like a doorbell or alarm)
            return "doorbell", 0.6
        elif audio_level > 0.3 and zero_crossings < 0.05:
            # Low-frequency sound with high energy (like a knock)
            return "knock", 0.6
        elif audio_level > 0.2:
            # Medium energy sound (could be various things)
            return "unknown", 0.4
        else:
            # Low energy, probably background noise
            return "unknown", 0.1


def load_sound_model():
    """Legacy function for compatibility with existing code."""
    classifier = SoundClassifier()
    classifier.load_model()
    return classifier


def classify_sound(audio_data):
    """Legacy function for compatibility with existing code."""
    classifier = SoundClassifier()
    sound_type, _ = classifier.classify_sound(audio_data)
    return sound_type