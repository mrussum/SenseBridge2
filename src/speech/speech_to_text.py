"""
Speech to text module for SenseBridge.
Converts spoken language to text for display to the user.
"""

import os
import threading
import queue
import time
import logging
import speech_recognition as sr
from ..utils.config import Config

logger = logging.getLogger(__name__)


class SpeechToText:
    """Converts spoken language to text for display."""

    def __init__(self, text_callback=None):
        """Initialize the speech to text processor.

        Args:
            text_callback: Function to call when text is recognized
        """
        self.config = Config()
        self.user_prefs = self.config.get_user_preferences()
        self.speech_config = self.user_prefs["speech_to_text"]

        self.enabled = self.speech_config["enabled"]
        self.continuous_mode = self.speech_config["continuous_mode"]
        self.display_timeout = self.speech_config["display_timeout"]

        self.recognizer = sr.Recognizer()
        self.microphone = None

        self.text_callback = text_callback
        self.running = False
        self.speech_thread = None

        # Ambient noise adjustment
        self.energy_threshold = 300  # Default value
        self.dynamic_adjustment = True

        # Queue for continuous recognition
        self.text_queue = queue.Queue()

        logger.info("SpeechToText initialized")

    def start(self):
        """Start speech recognition in a separate thread."""
        if not self.enabled:
            logger.info("Speech to text is disabled in preferences")
            return

        if self.running:
            logger.warning("Speech to text already running")
            return

        self.running = True

        # Initialize microphone
        try:
            # List available microphones
            mics = sr.Microphone.list_microphone_names()
            logger.info(f"Available microphones: {mics}")

            # Use default microphone
            self.microphone = sr.Microphone()

            # Adjust for ambient noise
            with self.microphone as source:
                logger.info("Adjusting for ambient noise...")
                self.recognizer.adjust_for_ambient_noise(source, duration=1)
                self.energy_threshold = self.recognizer.energy_threshold
                logger.info(f"Energy threshold set to {self.energy_threshold}")

        except Exception as e:
            logger.error(f"Error initializing microphone: {str(e)}")
            self.running = False
            return

        # Start recognition thread
        self.speech_thread = threading.Thread(target=self._speech_recognition_loop)
        self.speech_thread.daemon = True
        self.speech_thread.start()

        logger.info("Speech to text started")

    def stop(self):
        """Stop speech recognition."""
        if not self.running:
            return

        self.running = False

        if self.speech_thread:
            self.speech_thread.join(timeout=2.0)

        logger.info("Speech to text stopped")

    def _speech_recognition_loop(self):
        """Main speech recognition loop that runs in a separate thread."""
        if self.continuous_mode:
            self._continuous_recognition()
        else:
            self._triggered_recognition()

    def _continuous_recognition(self):
        """Continuously listen for speech and convert to text."""
        logger.info("Starting continuous speech recognition")

        with self.microphone as source:
            while self.running:
                try:
                    # Listen for speech
                    logger.debug("Listening for speech...")
                    audio = self.recognizer.listen(source, timeout=10.0, phrase_time_limit=10.0)

                    # Recognize speech using Google Speech Recognition
                    text = self.recognizer.recognize_google(audio)

                    logger.info(f"Recognized: {text}")

                    # Call callback with recognized text
                    if self.text_callback:
                        self.text_callback(text)

                except sr.WaitTimeoutError:
                    logger.debug("No speech detected within timeout")
                    continue

                except sr.UnknownValueError:
                    logger.debug("Speech Recognition could not understand audio")
                    continue

                except sr.RequestError as e:
                    logger.error(f"Could not request results from Google Speech Recognition service: {e}")
                    time.sleep(1.0)  # Wait before retrying

                except Exception as e:
                    logger.error(f"Error in speech recognition: {str(e)}")
                    time.sleep(0.5)

    def _triggered_recognition(self):
        """Listen for speech only when triggered."""
        logger.info("Starting triggered speech recognition")

        # In triggered mode, we'd wait for some event (like a button press)
        # For now, we'll simulate this with a simple loop that recognizes once every few seconds

        with self.microphone as source:
            while self.running:
                try:
                    # Listen for speech with a short timeout
                    logger.debug("Waiting for speech...")
                    audio = self.recognizer.listen(source, timeout=3.0, phrase_time_limit=5.0)

                    # Recognize speech using Google Speech Recognition
                    text = self.recognizer.recognize_google(audio)

                    logger.info(f"Recognized: {text}")

                    # Call callback with recognized text
                    if self.text_callback:
                        self.text_callback(text)

                    # Wait before listening again
                    time.sleep(2.0)

                except sr.WaitTimeoutError:
                    # No speech detected, just continue
                    continue

                except sr.UnknownValueError:
                    # Speech was unintelligible
                    continue

                except sr.RequestError as e:
                    logger.error(f"Could not request results from Google Speech Recognition service: {e}")
                    time.sleep(1.0)

                except Exception as e:
                    logger.error(f"Error in speech recognition: {str(e)}")
                    time.sleep(0.5)


def recognize_speech():
    """Legacy function for compatibility with existing code."""
    # This is just a placeholder that returns text
    # The actual implementation uses the SpeechToText class
    time.sleep(0.1)  # Simulate processing time
    return "Hello, this is a test message."