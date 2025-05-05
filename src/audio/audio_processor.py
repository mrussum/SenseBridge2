"""
Audio processing module for SenseBridge.
Handles audio capture and preprocessing for sound recognition.
"""

import numpy as np
import pyaudio
import time
import threading
import queue
from scipy import signal
import logging
from ..utils.config import Config

logger = logging.getLogger(__name__)


class AudioProcessor:
    """Handles audio capture and preprocessing for sound recognition."""

    def __init__(self):
        """Initialize the audio processor with configuration settings."""
        self.config = Config().get_device_config()
        self.audio_config = self.config["audio"]

        self.sample_rate = self.audio_config["sample_rate"]
        self.chunk_size = self.audio_config["chunk_size"]
        self.channels = self.audio_config["channels"]

        self.audio_queue = queue.Queue()
        self.running = False
        self.audio_thread = None
        self.pyaudio_instance = None
        self.stream = None

        logger.info("AudioProcessor initialized with sample rate: %d Hz", self.sample_rate)

    def start(self):
        """Start audio capture in a separate thread."""
        if self.running:
            logger.warning("Audio processor already running")
            return

        self.running = True
        self.audio_thread = threading.Thread(target=self._audio_capture_loop)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        logger.info("Audio processor started")

    def stop(self):
        """Stop audio capture."""
        if not self.running:
            return

        self.running = False
        if self.audio_thread:
            self.audio_thread.join(timeout=2.0)

        if self.stream:
            self.stream.stop_stream()
            self.stream.close()

        if self.pyaudio_instance:
            self.pyaudio_instance.terminate()

        logger.info("Audio processor stopped")

    def _audio_capture_loop(self):
        """Main audio capture loop that runs in a separate thread."""
        try:
            self.pyaudio_instance = pyaudio.PyAudio()
            self.stream = self.pyaudio_instance.open(
                format=pyaudio.paInt16,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size,
                stream_callback=self._audio_callback
            )

            self.stream.start_stream()

            # Keep thread alive while stream is active
            while self.running and self.stream.is_active():
                time.sleep(0.1)

        except Exception as e:
            self.running = False
            logger.error(f"Error in audio capture: {str(e)}")

    def _audio_callback(self, in_data, frame_count, time_info, status):
        """Callback function for PyAudio stream."""
        if status:
            logger.warning(f"Audio callback status: {status}")

        # Convert audio data to numpy array
        audio_data = np.frombuffer(in_data, dtype=np.int16)

        # Preprocess audio data
        processed_data = self._preprocess_audio(audio_data)

        # Put processed data in queue
        self.audio_queue.put(processed_data)

        return (in_data, pyaudio.paContinue)

    def _preprocess_audio(self, audio_data):
        """Preprocess audio data for sound recognition."""
        # Convert to float32 and normalize to [-1.0, 1.0]
        normalized_data = audio_data.astype(np.float32) / 32768.0

        # Apply pre-emphasis filter to enhance higher frequencies
        emphasized_data = signal.lfilter([1.0, -0.97], [1], normalized_data)

        return emphasized_data

    def get_audio_data(self, timeout=1.0):
        """Get processed audio data from the queue.

        Args:
            timeout: Time to wait for audio data in seconds

        Returns:
            Processed audio data or None if timeout
        """
        try:
            return self.audio_queue.get(timeout=timeout)
        except queue.Empty:
            return None