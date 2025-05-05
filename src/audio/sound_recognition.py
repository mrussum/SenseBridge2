"""
Sound recognition module for SenseBridge.
Detects and analyzes environmental sounds.
"""

import numpy as np
import time
import threading
import logging
from .audio_processor import AudioProcessor
from ..models.sound_classifier import SoundClassifier
from ..utils.config import Config

logger = logging.getLogger(__name__)


class SoundRecognition:
    """Detects and analyzes environmental sounds."""

    def __init__(self, callback=None):
        """Initialize the sound recognition system.

        Args:
            callback: Function to call when a sound is detected
        """
        self.config = Config()
        self.user_prefs = self.config.get_user_preferences()
        self.sound_config = self.user_prefs["sound_detection"]

        self.sensitivity = self.sound_config["sensitivity"]
        self.min_confidence = self.sound_config["min_confidence"]
        self.ambient_adjustment = self.sound_config["ambient_adjustment"]

        self.audio_processor = AudioProcessor()
        self.sound_classifier = SoundClassifier()

        self.callback = callback
        self.running = False
        self.recognition_thread = None

        # For ambient noise adjustment
        self.ambient_level = 0.01  # Initial ambient noise level
        self.ambient_samples = []
        self.ambient_window_size = 50

        logger.info("SoundRecognition initialized with sensitivity: %.2f", self.sensitivity)

    def start(self):
        """Start sound recognition in a separate thread."""
        if self.running:
            logger.warning("Sound recognition already running")
            return

        self.running = True
        self.audio_processor.start()

        # Wait for classifier to load
        self.sound_classifier.load_model()

        self.recognition_thread = threading.Thread(target=self._recognition_loop)
        self.recognition_thread.daemon = True
        self.recognition_thread.start()
        logger.info("Sound recognition started")

    def stop(self):
        """Stop sound recognition."""
        if not self.running:
            return

        self.running = False

        if self.recognition_thread:
            self.recognition_thread.join(timeout=2.0)

        self.audio_processor.stop()
        logger.info("Sound recognition stopped")

    def _recognition_loop(self):
        """Main recognition loop that runs in a separate thread."""
        # Calibrate ambient noise level first
        if self.ambient_adjustment:
            self._calibrate_ambient_noise()

        # Main detection loop
        while self.running:
            try:
                # Get audio data from processor
                audio_data = self.audio_processor.get_audio_data()

                if audio_data is None:
                    continue

                # Check if audio level is above threshold
                audio_level = np.abs(audio_data).mean()

                # Adjust threshold based on ambient noise if enabled
                threshold = self.ambient_level * (1.0 + 5.0 * self.sensitivity)

                if audio_level > threshold:
                    # Audio level is above threshold, classify the sound
                    sound_type, confidence = self.sound_classifier.classify_sound(audio_data)

                    # Update ambient noise level if this isn't a recognized sound or confidence is low
                    if confidence < self.min_confidence and self.ambient_adjustment:
                        self._update_ambient_level(audio_level)

                    # If confidence is high enough, trigger callback
                    if confidence >= self.min_confidence:
                        logger.info(f"Detected sound: {sound_type} (confidence: {confidence:.2f})")
                        if self.callback:
                            self.callback(sound_type, confidence, audio_data)
                else:
                    # Update ambient noise level with current audio level
                    if self.ambient_adjustment:
                        self._update_ambient_level(audio_level)

            except Exception as e:
                logger.error(f"Error in sound recognition loop: {str(e)}")
                time.sleep(0.1)

    def _calibrate_ambient_noise(self):
        """Calibrate ambient noise level."""
        logger.info("Calibrating ambient noise level...")
        samples = []

        # Collect audio samples for 3 seconds
        start_time = time.time()
        while time.time() - start_time < 3.0:
            audio_data = self.audio_processor.get_audio_data(timeout=0.1)
            if audio_data is not None:
                level = np.abs(audio_data).mean()
                samples.append(level)
            time.sleep(0.05)

        if samples:
            # Calculate ambient noise level (median to avoid outliers)
            self.ambient_level = np.median(samples)
            logger.info(f"Ambient noise level calibrated to: {self.ambient_level:.6f}")
        else:
            logger.warning("Failed to calibrate ambient noise level")

    def _update_ambient_level(self, audio_level):
        """Update ambient noise level using a moving window approach.

        Args:
            audio_level: Current audio level
        """
        self.ambient_samples.append(audio_level)
        if len(self.ambient_samples) > self.ambient_window_size:
            self.ambient_samples.pop(0)

        # Use a robust estimate (median) to avoid influence of sudden noises
        self.ambient_level = np.median(self.ambient_samples)


def detect_sound():
    """Legacy function for compatibility with existing code.
    Returns a dummy audio data array.
    """
    # This is just a placeholder that returns random audio data
    # The actual implementation uses the SoundRecognition class
    return np.random.normal(0, 500, 1024).astype(np.int16)