# SenseBridge: Assistive Technology for Sensory Impairments

SenseBridge is a comprehensive assistive technology system designed to help people with sensory impairments, particularly those who are deaf, hard of hearing, or visually impaired. The system uses AI to detect environmental sounds and provides notifications through haptic feedback, visual alerts, and smart home integration.

## Features

- **Sound Event Detection**: Recognizes important sounds like doorbells, knocks, alarms, and appliance beeps
- **Real-time Speech-to-Text**: Converts spoken language to text for improved communication
- **Multi-modal Notifications**: 
  - Haptic feedback via wearable device
  - Visual alerts with high-contrast display
  - Smart home integration (lights, etc.)
- **Portable Design**: Can be used at home, in the office, or on the go
- **Customizable Alerts**: Different patterns for different sounds
- **Adaptive Technology**: Adjusts to ambient noise levels for better detection
- **Raspberry Pi Compatible**: Designed to run on affordable, accessible hardware

## Requirements

See `requirements.txt` for required Python packages. Main dependencies include:

- Python 3.8+
- TensorFlow for AI sound classification
- PyAudio for audio processing
- SpeechRecognition for speech-to-text
- Bluetooth libraries for wearable communication
- MQTT for smart home integration

## Hardware Requirements

- Raspberry Pi (version 4 or 5 recommended)
- USB Microphone or Raspberry Pi compatible microphone
- Display (touchscreen recommended)
- LED indicators
- Vibration motor for haptic feedback (for development)
- Bluetooth adapter (if not built-in)
- Wearable device (optional - smartwatch or custom wearable)

## Installation

1. Clone the repository
```bash
git clone https://github.com/yourusername/senseBridge.git
cd senseBridge
```

2. Run the setup script to create the virtual environment and install dependencies
```bash
chmod +x setup.sh
./setup.sh
```

3. Activate the virtual environment
```bash
source .venv/bin/activate
```

## Running SenseBridge

Start the main application:
```bash
python -m src.main
```

For headless mode (no GUI):
```bash
python -m src.main --headless
```

## Project Structure

```
senseBridge/
├── README.md              # Project documentation
├── requirements.txt       # Python dependencies
├── setup.sh               # Installation script
├── config/
│   ├── device_config.json # Hardware configuration
│   ├── sound_events.json  # Sound classifications
│   └── user_prefs.json    # User preferences
├── models/
│   └── yamnet_model/      # Pre-trained sound classification model
├── tests/
│   ├── test_audio.py
│   ├── test_notification.py
│   └── test_speech.py
└── src/
    ├── main.py            # Main application entry point
    ├── audio/             # Audio processing modules
    ├── speech/            # Speech recognition modules
    ├── notification/      # Notification system modules
    ├── models/            # AI models for classification
    ├── hardware/          # Hardware control modules
    ├── gui/               # User interface modules
    └── utils/             # Utility modules
```

## Running Tests

Run all tests:
```bash
pytest tests/
```

Run specific test:
```bash
pytest tests/test_sound_recognition.py
```

## Wearable Device

SenseBridge can communicate with a Bluetooth wearable device to provide haptic feedback. The wearable device should implement a simple JSON-based protocol:

- Commands are sent as JSON: `{"cmd": "vibrate", "params": {"intensity": 1.0, "duration": 500}}`
- Supported commands include `vibrate`, `alert`, and `status`

## Smart Home Integration

SenseBridge can integrate with smart home systems via MQTT. Configure your MQTT broker details in `config/user_prefs.json`.

## Development and Customization

### Adding New Sound Types

1. Edit `config/sound_events.json` to add new sound types
2. Add corresponding target classes in `src/models/sound_classifier.py`

### Customizing Notification Patterns

1. Edit haptic patterns in `src/notification/haptic_feedback.py`
2. Edit visual patterns in `src/notification/visual_notification.py`

## Contribution

Contributions to SenseBridge are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- YAMNet audio event detection model from Google
- SpeechRecognition library
- The deaf and hard of hearing community for valuable feedback