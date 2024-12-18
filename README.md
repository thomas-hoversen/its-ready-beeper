# its-ready-beeper

## Upload Code and Files to the ESP32 Board

Follow these steps to **clear the flash**, upload the SPIFFS image (your `.wav` file), and upload the Arduino code.

---

### 1. Install Required Tools
Make sure the following tools are installed:
- **esptool**: To flash the ESP32.  
   Install it using:
   ```bash
   pip3 install esptool
   ```
- **mkspiffs**: To create the SPIFFS image.  
   Follow the steps below to install it.

---

### 2. Install and Set Up `mkspiffs`

#### Step 1: Download the Correct Release
1. Go to the **mkspiffs GitHub Releases** page:  
   [https://github.com/igrr/mkspiffs/releases](https://github.com/igrr/mkspiffs/releases)

2. Download the appropriate release for **macOS**:  
   - `mkspiffs-0.2.3-arduino-esp32-osx.tar.gz`

#### Step 2: Extract the File and Move It
Extract the file and place it in your project directory:
```bash
cd ~/Downloads
tar -xzf mkspiffs-0.2.3-arduino-esp32-osx.tar.gz
mv mkspiffs-0.2.3-arduino-esp32-osx ~/Desktop/git_projects/its-ready/its-ready-beeper/mkspiffs
chmod +x ~/Desktop/git_projects/its-ready/its-ready-beeper/mkspiffs
```

---

### 3. Generate and Upload the SPIFFS Image

#### Step 1: Place Your `.wav` File
Place your `.wav` file in a folder named `data` inside your project directory:
```
~/Desktop/git_projects/its-ready/its-ready-beeper/data/audio1.wav
```

#### Step 2: Generate the SPIFFS Image
Run this command to create the SPIFFS image:
```bash
cd ~/Desktop/git_projects/its-ready/its-ready-beeper
./mkspiffs -c ./data -b 4096 -p 256 -s 983040 spiffs.bin
```
- `-s 983040`: Sets the SPIFFS size to **960KB**.

#### Step 3: Clear ESP32 Flash Memory
Erase all previous data on the ESP32:
```bash
esptool.py --chip esp32 --port /dev/cu.usbserial-0001 erase_flash
```

#### Step 4: Upload the SPIFFS Image to the ESP32
Flash the generated `spiffs.bin` file to the ESP32 at the specified offset:
```bash
esptool.py --chip esp32 --port /dev/cu.usbserial-0001 write_flash 0x290000 spiffs.bin
```

---

### 4. Upload Arduino Code to ESP32

#### Step 1: Compile the Code
Use `arduino-cli` to compile your sketch with the custom partition scheme:
```bash
arduino-cli compile --fqbn esp32:esp32:esp32 --build-property "build.partitions=partitions.csv" .
```

#### Step 2: Upload the Code
Upload the compiled code to the ESP32:
```bash
arduino-cli upload --fqbn esp32:esp32:esp32 --port /dev/cu.usbserial-0001
```

---

### 5. Verify the Process
1. Open the **Serial Monitor** at **115200 baud** to check logs:
   ```bash
   screen /dev/cu.usbserial-0001 115200
   ```
2. Verify that:
   - SPIFFS is initialized and the file list shows `audio1.wav`.
   - The ESP32 connects to the Bluetooth speaker.
   - The audio file plays correctly when a beep is detected.

---

## Install Tools for the First Time

### Install `esptool` and `arduino-cli`
Run the following commands to install the required tools:
```bash
pip3 install esptool
brew install arduino-cli
```

### Set Up the `arduino-cli`
1. Add the ESP32 board platform:
   ```bash
   arduino-cli core update-index
   arduino-cli core install esp32:esp32
   ```
2. Verify the setup:
   ```bash
   arduino-cli board list
   ```

---

### Notes:
- If the file upload fails, verify the SPIFFS address (`0x290000`) matches the `partitions.csv` file:
   ```csv
   # Name,   Type, SubType, Offset,   Size,      Flags
   nvs,      data, nvs,     0x9000,   0x5000,
   otadata,  data, ota,     0xe000,   0x2000,
   app0,     app,  ota_0,   0x10000,  0x140000,
   app1,     app,  ota_1,   0x150000, 0x140000,
   spiffs,   data, spiffs,  0x290000, 0xF0000   # SPIFFS set to 960KB
   ```

---

By following these steps, your **ESP32** will:
1. Detect beeps using FFT.
2. Flash the LED when a beep is confirmed.
3. Play the `.wav` file over Bluetooth when the detection is triggered. 🎉