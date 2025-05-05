"""
GUI app module for SenseBridge.
Main application window and user interface.
"""

import tkinter as tk
from tkinter import ttk
import threading
import logging
import os
from .interface import initialize_gui, show_alert, show_speech_text, update_status, DummyTk
from ..utils.config import Config

logger = logging.getLogger(__name__)


class SenseBridgeApp:
    """Main application class for SenseBridge GUI."""

    def __init__(self, use_gui=True):
        """Initialize the application.

        Args:
            use_gui: Whether to use GUI (False for headless mode)
        """
        self.config = Config()
        self.use_gui = use_gui
        self.root = None
        self.dashboard = None

        logger.info("SenseBridgeApp initializing...")

    def start(self):
        """Start the application."""
        if self.use_gui:
            try:
                # Initialize the GUI
                self.root = initialize_gui()

                # Update status
                update_status("Initializing systems...")

                logger.info("GUI started")
                return self.root

            except Exception as e:
                logger.error(f"Error starting GUI: {str(e)}")
                self.use_gui = False
                logger.warning("Falling back to headless mode")

        if not self.use_gui:
            # Create a dummy Tk instance for headless mode
            self.root = DummyTk()
            logger.info("Running in headless mode")

        return self.root

    def show_notification(self, message, event_type="info"):
        """Show a notification in the UI.

        Args:
            message: Notification message
            event_type: Type of event (for styling)
        """
        if not message:
            return

        # Log the notification
        logger.info(f"Notification: {message} (type: {event_type})")

        # Display in GUI if available
        if self.use_gui:
            show_alert(message)

    def update_speech_text(self, text):
        """Update the speech-to-text display.

        Args:
            text: Speech text to display
        """
        if not text:
            return

        # Log the speech text
        logger.debug(f"Speech text: {text}")

        # Display in GUI if available
        if self.use_gui:
            show_speech_text(text)

    def update_status_message(self, status):
        """Update the status message.

        Args:
            status: Status message
        """
        if not status:
            return

        # Log the status
        logger.debug(f"Status: {status}")

        # Display in GUI if available
        if self.use_gui:
            update_status(status)


def create_app(use_gui=True):
    """Create and initialize the application.

    Args:
        use_gui: Whether to use GUI

    Returns:
        SenseBridgeApp instance
    """
    # Determine whether to use GUI based on environment
    if os.environ.get("DISPLAY") is None and not os.path.exists("/etc/rpi-issue"):
        # No display available and not on Raspberry Pi
        use_gui = False

    # Create app instance
    app = SenseBridgeApp(use_gui=use_gui)

    # Start the app
    app.start()

    return app