# file: create_config_files.py
import os
import json


def create_json_file(path, content):
    with open(path, 'w') as f:
        json.dump(content, f, indent=4)
    print(f"Created {path}")


def main():
    config_dir = os.path.join(os.getcwd(), "config")
    os.makedirs(config_dir, exist_ok=True)

    # Create device_config.json
    device_config = {
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
    create_json_file(os.path.join(config_dir, "device_config.json"), device_config)

    # Create sound_events.json
    sound_events = {
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
    create_json_file(os.path.join(config_dir, "sound_events.json"), sound_events)

    # Create user_prefs.json
    user_prefs = {
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
    create_json_file(os.path.join(config_dir, "user_prefs.json"), user_prefs)


if __name__ == "__main__":
    main()