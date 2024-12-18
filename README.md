# its-ready-beeper

## Upload the `.wav` File to the ESP32 Board

Follow these steps to upload the `.wav` file to your ESP32 using `mkspiffs` and `esptool`.

---

### 1. Install `esptool` (Python Tool)

Ensure Python and `pip` are installed, then run:
```bash
pip3 install esptool
```

---

### 2. Install and Set Up `mkspiffs`

#### Step 1: Download the Correct Release
1. Go to the **mkspiffs GitHub Releases** page:  
   [https://github.com/igrr/mkspiffs/releases](https://github.com/igrr/mkspiffs/releases)

2. Download the appropriate release for **macOS**:  
   - `mkspiffs-0.2.3-arduino-esp32-osx.tar.gz`

---

#### Step 2: Extract the File
From your `Downloads` directory, extract the file:
```bash
cd ~/Downloads
tar -xzf mkspiffs-0.2.3-arduino-esp32-osx.tar.gz
```

---

#### Step 3: Move `mkspiffs` to the Project Directory
Move the extracted `mkspiffs` executable into your project folder:
```bash
mv mkspiffs-0.2.3-arduino-esp32-osx ~/Desktop/git_projects/its-ready/its-ready-beeper/
```

---

#### Step 4: Rename and Make It Executable
Rename the file for convenience and set executable permissions:
```bash
cd ~/Desktop/git_projects/its-ready/its-ready-beeper/
mv mkspiffs-0.2.3-arduino-esp32-osx mkspiffs
chmod +x mkspiffs
```

---

#### Step 5: Verify the Installation
Run the following command to ensure `mkspiffs` works:
```bash
./mkspiffs --version
```

---

### 3. Prepare the SPIFFS Image

1. Place your `.wav` file inside a folder named `data` in your project directory:
   ```
   ~/Desktop/git_projects/its-ready/its-ready-beeper/data/your-audio.wav
   ```

2. Generate the SPIFFS image:
   ```bash
   ./mkspiffs -c ./data -b 4096 -p 256 -s 1507328 spiffs.bin
   ```

---

### 4. Upload the SPIFFS Image to the ESP32

Use `esptool` to upload the SPIFFS image:
```bash
esptool.py --chip esp32 --port /dev/cu.usbserial-0001 --baud 921600 write_flash 0x291000 spiffs.bin
```
- Replace `/dev/cu.usbserial-0001` with the correct port for your ESP32.
- Adjust the address `0x291000` if you're using a custom partition scheme.

---

### 5. Verify File Upload
After uploading, the `.wav` file will be accessible through SPIFFS on your ESP32 board.
