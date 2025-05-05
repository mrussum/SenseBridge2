"""
Configuration management for SenseBridge.
Handles loading and saving configuration files.
"""

import os
import json
import logging
from pathlib import Path

logger = logging.getLogger(__name__)


class Config:
    """Handles application configuration."""

    # Singleton instance
    _instance = None

    def __new__(cls):
        """Create a new Config instance or return existing one (singleton)."""
        if cls._instance is None:
            cls._instance = super(Config, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        """Initialize the configuration manager."""
        if self._initialized:
            return

        self._initialized = True

        # Default config paths
        self.config_dir = Path("config")
        self.device_config_path = self.config_dir / "device_config.json"
        self.sound_events_path = self.config_dir / "sound_events.json"
        self.user_prefs_path = self.config_dir / "user_prefs.json"

        # Cached configs
        self.device_config = None
        self.sound_events = None
        self.user_preferences = None

        # Initialize
        self._load_configs()

        logger.info("Config manager initialized")

    def _load_configs(self):
        """Load all configuration files."""
        self._load_device_config()
        self._load_sound_events()
        self._load_user_preferences()

    def _load_device_config(self):
        """Load device configuration."""
        try:
            if self.device_config_path.exists():
                with open(self.device_config_path, 'r') as f:
                    self.device_config = json.load(f)
                logger.info("Device configuration loaded")
            else:
                # Create default device config
                self.device_config = {
                    "audio": {
                        "sample_rate": 16000,
                        "chunk_size": 1024,
                        "channels": 1
                    },
                    "hardware": {
                        "haptic_pin": 18,
                        "led_pin": 23,
                        "button_pin": 24
                    },
                    "bluetooth": {
                        "device_name": "SenseBridge",
                        "wearable_mac": ""
                    }
                }
                self._save_device_config()
                logger.info("Default device configuration created")
        except Exception as e:
            logger.error(f"Error loading device config: {str(e)}")
            # Fallback to defaults
            self.device_config = {
                "audio": {
                    "sample_rate": 16000,
                    "chunk_size": 1024,
                    "channels": 1
                },
                "hardware": {
                    "haptic_pin": 18,
                    "led_pin": 23,
                    "button_pin": 24
                },
                "bluetooth": {
                    "device_name": "SenseBridge",
                    "wearable_mac": ""
                }
            }

    def _load_sound_events(self):
        """Load sound events configuration."""
        try:
            if self.sound_events_path.exists():
                with open(self.sound_events_path, 'r') as f:
                    self.sound_events = json.load(f)
                logger.info("Sound events configuration loaded")
            else:
                # Create default sound events config
                self.sound_events = {
                    "doorbell": {
                        "label": "Doorbell",
                        "priority": "high",
                        "haptic_pattern": "long_double",
                        "visual_pattern": "flash_bright"
                    },
                    "knock": {
                        "label": "Knock, knock",
                        "priority": "high",
                        "haptic_pattern": "short_triple",
                        "visual_pattern": "flash_medium"
                    },
                    "microwave_beep": {
                        "label": "Microwave",
                        "priority": "medium",
                        "haptic_pattern": "short_double",
                        "visual_pattern": "flash_low"
                    },
                    "alarm": {
                        "label": "Alarm",
                        "priority": "high",
                        "haptic_pattern": "continuous",
                        "visual_pattern": "flash_urgent"
                    }
                }
                self._save_sound_events()
                logger.info("Default sound events configuration created")
        except Exception as e:
            logger.error(f"Error loading sound events: {str(e)}")
            # Fallback to defaults
            self.sound_events = {
                "doorbell": {
                    "label": "Doorbell",
                    "priority": "high",
                    "haptic_pattern": "long_double",
                    "visual_pattern": "flash_bright"
                },
                "knock": {
                    "label": "Knock",
                    "priority": "high",
                    "haptic_pattern": "short_triple",
                    "visual_pattern": "flash_medium"
                }
            }

    def _load_user_preferences(self):
        """Load user preferences."""
        try:
            if self.user_prefs_path.exists():
                with open(self.user_prefs_path, 'r') as f:
                    self.user_preferences = json.load(f)
                logger.info("User preferences loaded")
            else:
                # Create default user preferences
                self.user_preferences = {
                    "notifications": {
                        "haptic": True,
                        "visual": True,
                        "smart_home": False
                    },
                    "speech_to_text": {
                        "enabled": True,
                        "continuous_mode": True,
                        "display_timeout": 30
                    },
                    "sound_detection": {
                        "sensitivity": 0.7,
                        "min_confidence": 0.6,
                        "ambient_adjustment": True
                    },
                    "smart_home": {
                        "mqtt_broker": "",
                        "mqtt_port": 1883,
                        "mqtt_username": "",
                        "mqtt_password": "",
                        "light_topic": "senseBridge/lights"
                    }
                }
                self._save_user_preferences()
                logger.info("Default user preferences created")
        except Exception as e:
            logger.error(f"Error loading user preferences: {str(e)}")
            # Fallback to defaults
            self.user_preferences = {
                "notifications": {
                    "haptic": True,
                    "visual": True,
                    "smart_home": False
                },
                "speech_to_text": {
                    "enabled": True,
                    "continuous_mode": True,
                    "display_timeout": 30
                },
                "sound_detection": {
                    "sensitivity": 0.7,
                    "min_confidence": 0.6,
                    "ambient_adjustment": True
                }
            }

    def _save_device_config(self):
        """Save device configuration to file."""
        try:
            # Ensure config directory exists
            os.makedirs(self.config_dir, exist_ok=True)

            with open(self.device_config_path, 'w') as f:
                json.dump(self.device_config, f, indent=4)

            logger.info("Device configuration saved")
            return True
        except Exception as e:
            logger.error(f"Error saving device config: {str(e)}")
            return False

    def _save_sound_events(self):
        """Save sound events configuration to file."""
        try:
            # Ensure config directory exists
            os.makedirs(self.config_dir, exist_ok=True)

            with open(self.sound_events_path, 'w') as f:
                json.dump(self.sound_events, f, indent=4)

            logger.info("Sound events configuration saved")
            return True
        except Exception as e:
            logger.error(f"Error saving sound events: {str(e)}")
            return False

    def _save_user_preferences(self):
        """Save user preferences to file."""
        try:
            # Ensure config directory exists
            os.makedirs(self.config_dir, exist_ok=True)

            with open(self.user_prefs_path, 'w') as f:
                json.dump(self.user_preferences, f, indent=4)

            logger.info("User preferences saved")
            return True
        except Exception as e:
            logger.error(f"Error saving user preferences: {str(e)}")
            return False

    def get_device_config(self):
        """Get device configuration.

        Returns:
            Device configuration dictionary
        """
        if self.device_config is None:
            self._load_device_config()
        return self.device_config

    def get_sound_events(self):
        """Get sound events configuration.

        Returns:
            Sound events dictionary
        """
        if self.sound_events is None:
            self._load_sound_events()
        return self.sound_events

    def get_user_preferences(self):
        """Get user preferences.

        Returns:
            User preferences dictionary
        """
        if self.user_preferences is None:
            self._load_user_preferences()
        return self.user_preferences

    def update_device_config(self, new_config):
        """Update device configuration.

        Args:
            new_config: Updated configuration dictionary

        Returns:
            True if successful, False otherwise
        """
        self.device_config = new_config
        return self._save_device_config()

    def update_sound_events(self, new_events):
        """Update sound events configuration.

        Args:
            new_events: Updated sound events dictionary

        Returns:
            True if successful, False otherwise
        """
        self.sound_events = new_events
        return self._save_sound_events()

    def update_user_preferences(self, new_preferences):
        """Update user preferences.

        Args:
            new_preferences: Updated preferences dictionary

        Returns:
            True if successful, False otherwise
        """
        self.user_preferences = new_preferences
        return self._save_user_preferences()