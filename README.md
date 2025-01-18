# its-ready-beeper

This ESP32-based project detects a specific beep frequency and then plays an audio file (e.g., a notification sound) over a Bluetooth-connected speaker. The code utilizes FFT-based audio processing and can be easily managed with the PlatformIO ecosystem.

## Requirements

- **PlatformIO**: This project assumes you are using PlatformIO (either via the VSCode extension or the PlatformIO CLI).
- **ESP32 Development Board**: A board such as the ESP32-DevKitC.
- **Bluetooth Speaker**: A speaker or headset to test audio playback.
- **Audio File (WAV)**: A `.wav` file to be played when a beep is detected.
- **ESP32FS Tool**: Used to upload the SPIFFS filesystem image. You can download `esp32fs.jar` from the [ESP32 PlatformIO File System Uploader repository](https://github.com/me-no-dev/arduino-esp32fs-plugin). Follow the instructions to integrate it into your PlatformIO environment.

## Setup Instructions

1. **Clone or Download the Project**  
   Retrieve this repository and open it in VSCode with the PlatformIO extension installed, or navigate to the project directory using the PlatformIO CLI.

2. **Place Your Audio File**  
   Place your `.wav` file inside the `data` directory of the project. For example:
   ```bash
   its-ready-beeper/
     ├─ src/
     ├─ data/
     │   └─ audio1.wav
     ├─ platformio.ini
     └─ ...
   ```

   Ensure the file is named `audio1.wav` unless you update the code references.

3. **Discover the ESP32 Port**
   Search for the port with the following command:
```bash
ls /dev/{tty,cu}.*
```
Then update the variables upload_port and monitor_port in the platformio.ini file.

4. **Build and Upload the Firmware**  
   Using PlatformIO, run the following command to compile and upload the code to your ESP32:
   ```bash
   platformio run --target upload
   ```

   This will build and flash the main application onto your ESP32.

5. **Upload the SPIFFS Filesystem Image**  
   The audio file resides in SPIFFS (SPI Flash File System) on the ESP32. To upload it, run:
   ```bash
   esptool.py --port /dev/tty.wchusbserial3110 erase_flash
   platformio run --target uploadfs
   ```

   Ensure the `esp32fs.jar` tool is correctly installed and configured to enable this step.

6. **Monitor the Serial Output**  
   After uploading, connect to the serial monitor to view the output logs:
   ```bash
   platformio device monitor
   ```
   The default baud rate is typically `115200` (adjust as needed).

   The logs will show whether:
   - SPIFFS initialized correctly and `audio1.wav` is available.
   - Bluetooth connects to the speaker.
   - The beep detection and audio playback events occur as expected.

7. **Running Unit Tests**  
   You can run unit tests to verify the functionality of individual components. Use the following command:
   ```bash
   platformio test
   ```
   This will execute the tests defined in the `test/` directory on the development board.

## Notes

- **Adjusting Frequency Thresholds**: If your beep frequency differs, you may need to adjust `FREQUENCY_MIN`, `FREQUENCY_MAX`, and amplitude thresholds in the code.
- **Customizing the Audio File**: Rename or change `audio1.wav` references in the code if you prefer a different filename or multiple audio files.
- **Partitions Table**: The `partitions.csv` file defines the memory layout of the ESP32, specifying regions for code, SPIFFS, and other data. Modify this file if you need to adjust the partitioning for your project.
- **Troubleshooting**: If the board does not appear on a serial port, check your USB cable and connection. If SPIFFS upload or code upload fails, ensure that your PlatformIO and ESP32 environment are set up correctly in `platformio.ini`.
