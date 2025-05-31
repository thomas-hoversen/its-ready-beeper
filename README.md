# ITS-Ready Beeper - ESP32 Audio Detection System

A real-time audio frequency detection system built for ESP32 that automatically plays notification sounds via Bluetooth when specific beep frequencies are detected. This project demonstrates embedded systems programming, digital signal processing, and wireless audio streaming capabilities.

## 🔧 Technical Overview

- **Microcontroller**: ESP32 (ESP32-DevKitC)
- **Framework**: Arduino/ESP-IDF with PlatformIO
- **Key Features**:
  - Real-time FFT-based frequency analysis (1200-9000 Hz range)
  - Bluetooth A2DP audio streaming
  - SPIFFS filesystem for audio storage
  - Multi-window beep detection with confidence thresholds
  - LED status indicators and button controls
  - Watchdog timer for system reliability

## 📋 Hardware Requirements

- ESP32 development board
- I2S microphone (pins: SCK=14, WS=15, SD=34)
- Bluetooth speaker/headphones
- LEDs (Blue: pin 17, Orange: pin 16)
- Push button (pin 25)

## 🚀 Development Setup

### Prerequisites
- [PlatformIO](https://platformio.org/) installed
- ESP32 drivers for your operating system

### 1. Port Discovery
Find your ESP32 port:
```bash
# macOS/Linux
ls /dev/{tty,cu}.*

# Windows
mode
```
Update `upload_port` and `monitor_port` in `platformio.ini` accordingly.

### 2. Build & Upload Firmware
```bash
# Build the project
pio run

# Upload firmware to ESP32
pio run --target upload
```

### 3. Upload Audio Files (SPIFFS)
The system uses SPIFFS to store audio files. Upload the filesystem:
```bash
# Upload SPIFFS data (audio1.wav)
pio run --target uploadfs
```

### 4. Monitor Serial Output
```bash
# Start serial monitor
pio device monitor

# Or with specific baud rate
pio device monitor --baud 115200
```

## 📁 Project Structure

```
its-ready-beeper/
├── src/
│   ├── main.cpp              # Main application logic
│   ├── MyA2DPSimple.*        # Bluetooth A2DP implementation
│   ├── BeepDetector.*        # FFT-based frequency detection
│   └── BeepHistory.*         # Detection confidence tracking
├── data/
│   └── audio1.wav            # Notification audio file
├── platformio.ini            # Build configuration
└── partitions.csv            # ESP32 memory layout
```

## ⚙️ Configuration

Key parameters in `main.cpp`:
```cpp
#define FREQUENCY_MIN           1200    // Detection range start
#define FREQUENCY_MAX           9000    // Detection range end
#define MULTI_WINDOW_CONFIDENCE 4       // Confidence threshold
#define THRESHOLD_COUNT         3       // Required detections
```

## 🔍 System Behavior

1. **Startup**: Initializes I2S, SPIFFS, and Bluetooth
2. **Detection**: Continuously analyzes audio via FFT
3. **Filtering**: Multi-window confidence system prevents false positives
4. **Notification**: Plays audio file when threshold is reached
5. **LED Feedback**: Visual status indicators for connection and playback states

## 📊 Performance Features

- **Real-time Processing**: 16kHz sampling with 512-point FFT
- **Power Optimization**: `-Os` compilation flags and efficient memory usage
- **Reliability**: Watchdog timer and error handling
- **Scalability**: Modular design for easy feature expansion

## 🛠️ Development Commands

```bash
# Clean build
pio run --target clean

# Erase flash completely
pio run --target erase

# Upload with verbose output
pio run --target upload --verbose

# Build for different environments
pio run -e esp32dev
```

---

*This project showcases embedded systems development, real-time signal processing, and IoT connectivity using modern C++ and ESP32 capabilities.*
