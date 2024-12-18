# its-ready-beeper

## Upload Code and Files to the ESP32 Board

Follow this **one-command workflow** to **clear the flash memory**, **upload the SPIFFS image**, and **upload the Arduino sketch** in one go.

---

### 1. Install Required Tools

Ensure these tools are installed:
- **esptool**: Install it with:
   ```bash
   pip3 install esptool
   ```
- **mkspiffs**: To generate the SPIFFS image.

   Install `mkspiffs`:
   ```bash
   cd ~/Downloads
   tar -xzf mkspiffs-0.2.3-arduino-esp32-osx.tar.gz
   mv mkspiffs-0.2.3-arduino-esp32-osx ~/Desktop/git_projects/its-ready/its-ready-beeper/mkspiffs
   chmod +x ~/Desktop/git_projects/its-ready/its-ready-beeper/mkspiffs
   ```

- **arduino-cli**: Install it with:
   ```bash
   brew install arduino-cli
   arduino-cli core update-index
   arduino-cli core install esp32:esp32
   ```

---

### 2. Place Your `.wav` File

Place the `.wav` file in the `data` folder:
```
~/Desktop/git_projects/its-ready/its-ready-beeper/data/audio1.wav
```

---

### 3. Clear Flash, Generate SPIFFS, and Upload Code (Single Command)

Copy and paste the command below to **erase the flash**, upload the SPIFFS image, and upload the Arduino code:

```bash
cd ~/Desktop/git_projects/its-ready/its-ready-beeper && \
esptool.py --chip esp32 --port /dev/cu.usbserial-0001 erase_flash && \
./mkspiffs -c ./data -b 4096 -p 256 -s 983040 spiffs.bin && \
esptool.py --chip esp32 --port /dev/cu.usbserial-0001 write_flash 0x290000 spiffs.bin && \
arduino-cli compile --fqbn esp32:esp32:esp32 --build-property "build.partitions=partitions.csv" . && \
arduino-cli upload --fqbn esp32:esp32:esp32 --port /dev/cu.usbserial-0001
```

---

### 4. Resolving Port Busy Errors

If you encounter an error like:
```
A fatal error occurred: Could not open /dev/cu.usbserial-0001, the port is busy or doesn't exist.
```

Follow these steps to resolve it:

1. Close any open **Serial Monitor** or terminal programs (e.g., `screen`) that might be using the port.
2. Check for processes using the port:
   ```bash
   lsof | grep /dev/cu.usbserial-0001
   ```
   - Kill the process using the port (replace `<PID>` with the process ID):
     ```bash
     kill -9 <PID>
     ```
3. Verify that the port is listed:
   ```bash
   ls /dev/cu.*
   ```
   Ensure `/dev/cu.usbserial-0001` appears in the output. If not:
   - Reconnect the ESP32.
   - Check your USB cable and port.

---

### 5. Verify Upload Success

1. Connect to the Serial Monitor:
   ```bash
   screen /dev/cu.usbserial-0001 115200
   ```
2. Verify logs:
   - SPIFFS should show `audio1.wav` as available.
   - The Bluetooth speaker should connect successfully.
   - The audio file plays correctly when a beep is detected.

---

### Notes:

- If you face **upload errors**, verify your **SPIFFS partition offset** matches the `partitions.csv` file:
   ```csv
   # Name,   Type, SubType, Offset,   Size,      Flags
   nvs,      data, nvs,     0x9000,   0x5000,
   otadata,  data, ota,     0xe000,   0x2000,
   app0,     app,  ota_0,   0x10000,  0x140000,
   app1,     app,  ota_1,   0x150000, 0x140000,
   spiffs,   data, spiffs,  0x290000, 0xF0000   # SPIFFS size: 960KB
   ```
