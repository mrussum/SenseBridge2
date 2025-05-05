"""
GUI interface module for SenseBridge.
Provides the user interface for the SenseBridge application.
"""

import tkinter as tk
from tkinter import ttk, font
import threading
import time
import logging
import queue
from PIL import Image, ImageTk
import os

logger = logging.getLogger(__name__)

# Global variables for GUI access
root = None
alert_label = None
speech_text_var = None
status_var = None
alert_queue = queue.Queue()
speech_queue = queue.Queue()
status_queue = queue.Queue()


def initialize_gui():
    """Initialize the GUI for SenseBridge."""
    global root, alert_label, speech_text_var, status_var

    logger.info("Initializing GUI...")

    # Create the main window
    root = tk.Tk()
    root.title("SenseBridge")
    root.geometry("800x480")  # Common Raspberry Pi display size

    # Make window fullscreen on Raspberry Pi
    if os.path.exists("/etc/rpi-issue"):
        root.attributes("-fullscreen", True)

    # Configure styles for high contrast
    style = ttk.Style()
    style.configure("TFrame", background="black")
    style.configure("Alert.TLabel", background="black", foreground="yellow",
                    font=("Arial", 24, "bold"))
    style.configure("Speech.TLabel", background="black", foreground="white",
                    font=("Arial", 16))
    style.configure("Status.TLabel", background="black", foreground="gray",
                    font=("Arial", 12))

    # Create main frame
    main_frame = ttk.Frame(root, style="TFrame")
    main_frame.pack(fill=tk.BOTH, expand=True)

    # Create header with logo and title
    header_frame = ttk.Frame(main_frame, style="TFrame")
    header_frame.pack(fill=tk.X, padx=20, pady=10)

    title_label = ttk.Label(
        header_frame,
        text="SenseBridge",
        style="Alert.TLabel",
        font=("Arial", 32, "bold")
    )
    title_label.pack(side=tk.LEFT, padx=20)

    # Create alert section
    alert_frame = ttk.Frame(main_frame, style="TFrame")
    alert_frame.pack(fill=tk.X, padx=20, pady=10)

    alert_label = ttk.Label(
        alert_frame,
        text="Ready",
        style="Alert.TLabel"
    )
    alert_label.pack(fill=tk.X, padx=20, pady=10)

    # Create speech-to-text section
    speech_frame = ttk.Frame(main_frame, style="TFrame")
    speech_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)

    speech_title = ttk.Label(
        speech_frame,
        text="Speech-to-Text",
        style="Speech.TLabel",
        font=("Arial", 20, "bold")
    )
    speech_title.pack(anchor=tk.W, padx=10, pady=5)

    speech_text_var = tk.StringVar()
    speech_text_var.set("Listening for speech...")

    speech_text = ttk.Label(
        speech_frame,
        textvariable=speech_text_var,
        style="Speech.TLabel",
        wraplength=760
    )
    speech_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

    # Create status bar
    status_frame = ttk.Frame(main_frame, style="TFrame")
    status_frame.pack(fill=tk.X, side=tk.BOTTOM, padx=20, pady=5)

    status_var = tk.StringVar()
    status_var.set("System ready")

    status_label = ttk.Label(
        status_frame,
        textvariable=status_var,
        style="Status.TLabel"
    )
    status_label.pack(side=tk.LEFT, padx=10)

    # Start the update thread
    update_thread = threading.Thread(target=_update_loop)
    update_thread.daemon = True
    update_thread.start()

    logger.info("GUI initialized")
    return root


def show_alert(message):
    """Show an alert message in the GUI.

    Args:
        message: Message to display
    """
    if not message:
        return

    alert_queue.put(message)
    logger.debug(f"Alert queued: {message}")


def show_speech_text(text):
    """Show speech-to-text in the GUI.

    Args:
        text: Text to display
    """
    if not text:
        return

    speech_queue.put(text)
    logger.debug(f"Speech text queued: {text}")


def update_status(status):
    """Update the status bar in the GUI.

    Args:
        status: Status message
    """
    if not status:
        return

    status_queue.put(status)
    logger.debug(f"Status update queued: {status}")


def _update_loop():
    """Background thread to update the GUI from queues."""
    last_alert_time = 0
    last_speech_time = 0

    while True:
        try:
            # Process alerts
            try:
                alert_message = alert_queue.get_nowait()
                _update_alert(alert_message)
                last_alert_time = time.time()
            except queue.Empty:
                # Clear alert after 5 seconds
                if time.time() - last_alert_time > 5.0 and alert_label is not None:
                    root.after(0, lambda: alert_label.config(text="Ready"))

            # Process speech text
            try:
                speech_text = speech_queue.get_nowait()
                _update_speech(speech_text)
                last_speech_time = time.time()
            except queue.Empty:
                pass

            # Process status updates
            try:
                status_text = status_queue.get_nowait()
                _update_status(status_text)
            except queue.Empty:
                pass

            # Sleep to reduce CPU usage
            time.sleep(0.1)

        except Exception as e:
            logger.error(f"Error in GUI update loop: {str(e)}")
            time.sleep(1.0)  # Wait longer on error


def _update_alert(message):
    """Update the alert label in the GUI thread.

    Args:
        message: Alert message
    """
    if root is not None and alert_label is not None:
        root.after(0, lambda: alert_label.config(text=message))


def _update_speech(text):
    """Update the speech text in the GUI thread.

    Args:
        text: Speech text
    """
    if root is not None and speech_text_var is not None:
        root.after(0, lambda: speech_text_var.set(text))


def _update_status(status):
    """Update the status bar in the GUI thread.

    Args:
        status: Status message
    """
    if root is not None and status_var is not None:
        root.after(0, lambda: status_var.set(status))


class DummyTk:
    """Dummy Tk class for when GUI is not available."""

    def __init__(self):
        pass

    def update(self):
        pass

    def after(self, ms, func):
        pass

    def mainloop(self):
        while True:
            time.sleep(1)