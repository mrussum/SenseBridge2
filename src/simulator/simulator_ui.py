# src/simulator/simulator_ui.py
"""
Simulator UI for SenseBridge testing.
Provides a simple UI to simulate various events and hardware.
"""

import tkinter as tk
from tkinter import ttk
import logging
import threading
import time
import numpy as np
import queue

logger = logging.getLogger(__name__)


class SimulatorUI:
    """Provides a simulator UI for testing SenseBridge."""

    def __init__(self, sound_callback=None, button_callback=None):
        """Initialize the simulator UI.

        Args:
            sound_callback: Function to call when a sound is simulated
            button_callback: Function to call when a button is pressed
        """
        self.sound_callback = sound_callback
        self.button_callback = button_callback

        self.root = None
        self.running = False
        self.event_queue = queue.Queue()

        # Status indicators
        self.haptic_active = False
        self.led_active = False

        # UI elements
        self.haptic_indicator = None
        self.led_indicator = None

    def start(self):
        """Start the simulator UI in a separate thread."""
        if self.running:
            logger.warning("Simulator UI already running")
            return

        self.running = True

        # Start UI thread
        threading.Thread(target=self._ui_thread, daemon=True).start()

        logger.info("Simulator UI started")

    def stop(self):
        """Stop the simulator UI."""
        if not self.running:
            return

        self.running = False

        if self.root:
            self.root.quit()

        logger.info("Simulator UI stopped")

    def _ui_thread(self):
        """Main UI thread."""
        try:
            # Create the main window
            self.root = tk.Tk()
            self.root.title("SenseBridge Simulator")
            self.root.geometry("600x400")

            # Create main frame
            main_frame = ttk.Frame(self.root, padding=10)
            main_frame.pack(fill=tk.BOTH, expand=True)

            # Create sound simulation section
            sound_frame = ttk.LabelFrame(main_frame, text="Simulate Sound Events", padding=10)
            sound_frame.pack(fill=tk.X, padx=5, pady=5)

            # Add sound buttons
            sound_types = ["doorbell", "knock", "alarm", "microwave_beep"]
            for i, sound_type in enumerate(sound_types):
                sound_btn = ttk.Button(
                    sound_frame,
                    text=sound_type.capitalize(),
                    command=lambda st=sound_type: self._simulate_sound(st)
                )
                sound_btn.grid(row=0, column=i, padx=5, pady=5)

            # Create hardware simulation section
            hw_frame = ttk.LabelFrame(main_frame, text="Hardware Simulation", padding=10)
            hw_frame.pack(fill=tk.X, padx=5, pady=5)

            # Add button simulation
            button_frame = ttk.Frame(hw_frame)
            button_frame.pack(fill=tk.X, padx=5, pady=5)

            ttk.Label(button_frame, text="Button:").pack(side=tk.LEFT, padx=5)
            ttk.Button(
                button_frame,
                text="Press",
                command=self._simulate_button_press
            ).pack(side=tk.LEFT, padx=5)

            # Add hardware indicators
            indicator_frame = ttk.Frame(hw_frame)
            indicator_frame.pack(fill=tk.X, padx=5, pady=5)

            # LED indicator
            ttk.Label(indicator_frame, text="LED:").grid(row=0, column=0, padx=5, pady=5)
            self.led_indicator = ttk.Label(indicator_frame, text="OFF", background="gray")
            self.led_indicator.grid(row=0, column=1, padx=5, pady=5)

            # Haptic indicator
            ttk.Label(indicator_frame, text="Haptic:").grid(row=1, column=0, padx=5, pady=5)
            self.haptic_indicator = ttk.Label(indicator_frame, text="OFF", background="gray")
            self.haptic_indicator.grid(row=1, column=1, padx=5, pady=5)

            # Create status log
            log_frame = ttk.LabelFrame(main_frame, text="Event Log", padding=10)
            log_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

            self.log_text = tk.Text(log_frame, height=10, width=60)
            self.log_text.pack(fill=tk.BOTH, expand=True)

            # Start event processing
            self.root.after(100, self._process_events)

            # Start the main loop
            self.root.protocol("WM_DELETE_WINDOW", self.stop)
            self.root.mainloop()

        except Exception as e:
            logger.error(f"Error in simulator UI thread: {e}")
            self.running = False

    def _process_events(self):
        """Process events from the queue."""
        if not self.running:
            return

        try:
            # Process all available events
            while not self.event_queue.empty():
                event = self.event_queue.get_nowait()
                event_type = event.get("type")
                event_data = event.get("data")

                if event_type == "log":
                    self._append_log(event_data)
                elif event_type == "led":
                    self._update_led(event_data)
                elif event_type == "haptic":
                    self._update_haptic(event_data)

        except queue.Empty:
            pass
        except Exception as e:
            logger.error(f"Error processing events: {e}")

        # Schedule next processing
        if self.running and self.root:
            self.root.after(100, self._process_events)

    def _append_log(self, message):
        """Append a message to the log."""
        if self.log_text:
            self.log_text.insert(tk.END, f"{message}\n")
            self.log_text.see(tk.END)

    def _update_led(self, state):
        """Update the LED indicator."""
        if self.led_indicator:
            if state:
                self.led_indicator.config(text="ON", background="yellow")
            else:
                self.led_indicator.config(text="OFF", background="gray")

    def _update_haptic(self, state):
        """Update the haptic indicator."""
        if self.haptic_indicator:
            if state:
                self.haptic_indicator.config(text="ON", background="green")
            else:
                self.haptic_indicator.config(text="OFF", background="gray")

    def _simulate_sound(self, sound_type):
        """Simulate a sound event."""
        self.event_queue.put({
            "type": "log",
            "data": f"Simulating sound: {sound_type}"
        })

        if self.sound_callback:
            # Create a simple audio sample
            audio_data = np.random.normal(0, 0.3, 16000).astype(np.float32)

            # Call the callback
            self.sound_callback(sound_type, 0.8, audio_data)

    def _simulate_button_press(self):
        """Simulate a button press."""
        self.event_queue.put({
            "type": "log",
            "data": "Simulating button press"
        })

        if self.button_callback:
            self.button_callback()

    def log_event(self, message):
        """Log an event to the UI.

        Args:
            message: Message to log
        """
        if self.running:
            self.event_queue.put({
                "type": "log",
                "data": message
            })

    def set_led_state(self, state):
        """Set the LED indicator state.

        Args:
            state: True for on, False for off
        """
        if self.running:
            self.event_queue.put({
                "type": "led",
                "data": state
            })

    def set_haptic_state(self, state):
        """Set the haptic indicator state.

        Args:
            state: True for on, False for off
        """
        if self.running:
            self.event_queue.put({
                "type": "haptic",
                "data": state
            })