"""
Notification manager for SenseBridge.
Coordinates different notification methods based on detected events.
"""

import logging
import threading
import time
import queue
from ..utils.config import Config

logger = logging.getLogger(__name__)


class NotificationManager:
    """Manages and coordinates notification delivery for detected events."""

    def __init__(self):
        """Initialize the notification manager."""
        self.config = Config()
        self.user_prefs = self.config.get_user_preferences()
        self.sound_events = self.config.get_sound_events()

        self.notification_config = self.user_prefs["notifications"]

        # Enable/disable different notification types
        self.haptic_enabled = self.notification_config["haptic"]
        self.visual_enabled = self.notification_config["visual"]
        self.smart_home_enabled = self.notification_config["smart_home"]

        # Import notification modules
        from .haptic_feedback import HapticFeedback
        from .visual_notification import VisualNotification
        from .smart_home import SmartHomeIntegration

        # Initialize notification subsystems
        self.haptic = HapticFeedback() if self.haptic_enabled else None
        self.visual = VisualNotification() if self.visual_enabled else None
        self.smart_home = SmartHomeIntegration() if self.smart_home_enabled else None

        # Queue for notification events
        self.notification_queue = queue.Queue()

        # Thread for processing notifications
        self.notification_thread = None
        self.running = False

        # Track active notifications to avoid duplicates
        self.active_notifications = {}

        logger.info("NotificationManager initialized")

    def start(self):
        """Start the notification manager."""
        if self.running:
            logger.warning("Notification manager already running")
            return

        self.running = True

        # Start notification subsystems
        if self.haptic:
            self.haptic.start()

        if self.visual:
            self.visual.start()

        if self.smart_home:
            self.smart_home.start()

        # Start notification processing thread
        self.notification_thread = threading.Thread(target=self._notification_loop)
        self.notification_thread.daemon = True
        self.notification_thread.start()

        logger.info("Notification manager started")

    def stop(self):
        """Stop the notification manager."""
        if not self.running:
            return

        self.running = False

        # Stop notification subsystems
        if self.haptic:
            self.haptic.stop()

        if self.visual:
            self.visual.stop()

        if self.smart_home:
            self.smart_home.stop()

        # Stop notification thread
        if self.notification_thread:
            self.notification_thread.join(timeout=2.0)

        logger.info("Notification manager stopped")

    def notify(self, event_type, confidence=1.0, data=None):
        """Schedule a notification for the given event.

        Args:
            event_type: Type of event (e.g., "doorbell", "knock")
            confidence: Confidence level of the detection (0.0-1.0)
            data: Additional data about the event
        """
        # Check if event type is supported
        if event_type not in self.sound_events and event_type != "speech":
            logger.warning(f"Unknown event type: {event_type}")
            return

        # Add to notification queue
        self.notification_queue.put({
            "event_type": event_type,
            "confidence": confidence,
            "data": data,
            "timestamp": time.time()
        })

        logger.info(f"Notification queued: {event_type} (confidence: {confidence:.2f})")

    def _notification_loop(self):
        """Main notification processing loop."""
        while self.running:
            try:
                # Get next notification from queue
                try:
                    notification = self.notification_queue.get(timeout=0.5)
                except queue.Empty:
                    continue

                event_type = notification["event_type"]
                confidence = notification["confidence"]
                data = notification["data"]
                timestamp = notification["timestamp"]

                # Check for duplicate notifications (same type within 3 seconds)
                if event_type in self.active_notifications:
                    last_time = self.active_notifications[event_type]
                    if timestamp - last_time < 3.0:
                        logger.debug(f"Skipping duplicate notification: {event_type}")
                        continue

                # Update active notifications
                self.active_notifications[event_type] = timestamp

                # Special handling for speech notifications
                if event_type == "speech":
                    self._notify_speech(data)
                    continue

                # Get event configuration
                event_config = self.sound_events.get(event_type, {})
                priority = event_config.get("priority", "medium")

                # Send appropriate notifications based on event config
                if self.haptic and self.haptic_enabled:
                    haptic_pattern = event_config.get("haptic_pattern", "short_double")
                    self.haptic.send_feedback(haptic_pattern, priority=priority)

                if self.visual and self.visual_enabled:
                    visual_pattern = event_config.get("visual_pattern", "flash_medium")
                    label = event_config.get("label", event_type.capitalize())
                    self.visual.show_notification(label, visual_pattern, priority=priority)

                if self.smart_home and self.smart_home_enabled:
                    self.smart_home.trigger_notification(event_type, priority)

                # Log notification
                logger.info(f"Notification sent: {event_type} (priority: {priority})")

            except Exception as e:
                logger.error(f"Error in notification loop: {str(e)}")

    def _notify_speech(self, text):
        """Handle speech notifications.

        Args:
            text: Recognized speech text
        """
        if not text:
            return

        # Display text via visual notification if enabled
        if self.visual and self.visual_enabled:
            self.visual.show_text(text)

        # Log speech
        logger.info(f"Speech notification: {text}")