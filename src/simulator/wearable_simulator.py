# src/simulator/wearable_simulator.py
"""
Wearable device simulator for SenseBridge.
Simulates a Bluetooth wearable device for testing.
"""

import tkinter as tk
from tkinter import ttk
import logging
import threading
import time
import queue
import json

logger = logging.getLogger(__name__)


class WearableSimulator:
    """Simulates a Bluetooth wearable device for testing."""

    def __init__(self):
        """Initialize the wearable simulator."""
        self.root = None
        self.running = False
        self.event_queue = queue.Queue()

        # Vibration state
        self.vibrating = False
        self.vibration_indicator = None

        # Command log
        self.log_text = None

    def start(self):
        """Start the wearable simulator in a separate thread."""
        if self.running:
            logger.warning("Wearable simulator already running")
            return

        self.running = True

        # Start UI thread
        threading.Thread(target=self._ui_thread, daemon=True).start()

        logger.info("Wearable simulator started")

    def stop(self):
        """Stop the wearable simulator."""
        if not self.running:
            return

        self.running = False

        if self.root:
            self.root.quit()

        logger.info("Wearable simulator stopped")

    def _ui_thread(self):
        """Main UI thread."""
        try:
            # Create the main window
            self.root = tk.Tk()
            self.root.title("SenseBridge Wearable Simulator")
            self.root.geometry("400x300")

            # Create main frame
            main_frame = ttk.Frame(self.root, padding=10)
            main_frame.pack(fill=tk.BOTH, expand=True)

            # Create status section
            status_frame = ttk.LabelFrame(main_frame, text="Wearable Status", padding=10)
            status_frame.pack(fill=tk.X, padx=5, pady=5)

            # Vibration indicator
            indicator_frame = ttk.Frame(status_frame)
            indicator_frame.pack(fill=tk.X, padx=5, pady=5)

            ttk.Label(indicator_frame, text="Vibration:").pack(side=tk.LEFT, padx=5)
            self.vibration_indicator = ttk.Label(indicator_frame, text="OFF", background="gray", width=10)
            self.vibration_indicator.pack(side=tk.LEFT, padx=5)

            # Battery level
            battery_frame = ttk.Frame(status_frame)
            battery_frame.pack(fill=tk.X, padx=5, pady=5)

            ttk.Label(battery_frame, text="Battery:").pack(side=tk.LEFT, padx=5)
            battery_level = ttk.Label(battery_frame, text="95%", background="green", width=10)
            battery_level.pack(side=tk.LEFT, padx=5)

            # Create command log
            log_frame = ttk.LabelFrame(main_frame, text="Command Log", padding=10)
            log_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

            self.log_text = tk.Text(log_frame, height=10, width=40)
            self.log_text.pack(fill=tk.BOTH, expand=True)

            # Add a scrollbar to the log
            scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
            scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
            self.log_text.configure(yscrollcommand=scrollbar.set)

            # Start event processing
            self.root.after(100, self._process_events)

            # Log startup
            self._append_log("Wearable simulator started")

            # Start the main loop
            self.root.protocol("WM_DELETE_WINDOW", self.stop)
            self.root.mainloop()

        except Exception as e:
            logger.error(f"Error in wearable simulator UI thread: {e}")
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
                elif event_type == "vibrate":
                    self._set_vibration(event_data.get("active", False))

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
            timestamp = time.strftime("%H:%M:%S")
            self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
            self.log_text.see(tk.END)

    def _set_vibration(self, active):
        """Set the vibration indicator state."""
        if self.vibration_indicator:
            if active:
                self.vibration_indicator.config(text="ON", background="orange")

                # Auto-disable after a short time
                if self.running and self.root:
                    self.root.after(500, lambda: self._set_vibration(False))
            else:
                self.vibration_indicator.config(text="OFF", background="gray")

    def simulate_command(self, command_data):
        """Simulate receiving a command from SenseBridge.

        Args:
            command_data: Command data as JSON string or dictionary
        """
        if isinstance(command_data, str):
            try:
                command_data = json.loads(command_data)
            except:
                command_data = {"cmd": command_data}

        cmd = command_data.get("cmd", "unknown")
        params = command_data.get("params", {})

        # Log the command
        cmd_str = f"Command: {cmd}"
        if params:
            cmd_str += f" - Params: {params}"

        self.event_queue.put({
            "type": "log",
            "data": cmd_str
        })

        # Handle specific commands
        if cmd == "vibrate":
            intensity = params.get("intensity", 1.0)
            duration = params.get("duration", 500)

            self.event_queue.put({
                "type": "vibrate",
                "data": {"active": True, "intensity": intensity}
            })

            # Log additional info
            self.event_queue.put({
                "type": "log",
                "data": f"Vibrating with intensity {intensity} for {duration}ms"
            })