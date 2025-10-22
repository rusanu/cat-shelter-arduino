# Cat Shelter Controller

ESP32-CAM based system to control a winter cat shelter with automated heating and monitoring.

## Features

- **Smart Heating Control**: Automatically activates heated blanket when temperature drops below 10°C and cat is present
- **Cat Detection**: PIR motion sensor detects cat presence
- **Temperature/Humidity Monitoring**: DHT22 sensor tracks environmental conditions
- **Photo Monitoring**:
  - Takes photos every 60 minutes
  - Takes photos when motion is detected (max once per 5 minutes)
  - Uploads photos to AWS S3
- **Power Management**: WiFi disconnects when idle to save power
- **Resilient Boot System**:
  - Tracks boot attempts in NVM (flash memory)
  - Automatically reboots on camera initialization failure
  - Enters safe mode after 3 failed boot attempts
  - Safe mode maintains core functions (heating/monitoring) without camera
  - Resets boot counter after 5 minutes of successful operation

## Hardware Requirements

- ESP32-CAM (AI Thinker module)
- DHT22/AM2302 temperature/humidity sensor
- PIR motion sensor
- Relay module (for heated blanket control)
- Heated blanket

## Hardware Connections

| Component | GPIO Pin | Type |
|-----------|----------|------|
| Relay (Heated Blanket) | GPIO12 | OUTPUT |
| PIR Motion Sensor | GPIO13 | INPUT |
| DHT22 Sensor | GPIO14 | DATA |

## Setup Instructions

### 1. Install PlatformIO

This project uses PlatformIO. Install it via VS Code extension or CLI.

### 2. Configure Secrets

Copy the template and fill in your credentials:

```bash
cp include/secrets.h.template include/secrets.h
```

Edit `include/secrets.h` with your:
- WiFi SSID and password
- AWS S3 bucket name and region
- AWS credentials (or configure bucket for pre-signed URL uploads)

**IMPORTANT**: Never commit `secrets.h` to git! It's already in `.gitignore`.

### 3. Build the Project

```bash
pio run
```

### 4. Upload Firmware

**Note**: The ESP32-CAM requires special wiring to enter boot mode. Connect GPIO0 to GND before powering on, then upload:

```bash
pio run --target upload
```

After upload, disconnect GPIO0 from GND and reset the board.

### 5. Monitor Serial Output

```bash
pio device monitor
```

You should see:
- Camera initialization
- GPIO pin setup
- DHT22 sensor initialization
- WiFi connection and NTP time sync
- Regular status reports every 60 seconds

## Configuration

### Temperature Threshold

Edit in [main.cpp](src/main.cpp):
```cpp
#define TEMP_COLD_THRESHOLD 10.0  // Celsius
```

### Photo Intervals

Edit in [main.cpp](src/main.cpp):
```cpp
#define PHOTO_HOURLY_INTERVAL 3600000  // 60 minutes
#define PHOTO_MOTION_COOLDOWN 300000   // 5 minutes
```

### Status Report Interval

Edit in [main.cpp](src/main.cpp):
```cpp
#define STATUS_REPORT_INTERVAL 60000   // 60 seconds
```

## S3 Upload Configuration

The current implementation uses HTTP PUT to S3. For production use, you have two options:

1. **Pre-signed URLs**: Recommended. Have a backend service generate pre-signed URLs and pass them to the device
2. **Public bucket with ACL**: Not recommended for production, but can be used for testing

## Monitoring

The system outputs a status report every 60 seconds via serial:

```
===== STATUS REPORT =====
Uptime: 3600 seconds
Temperature: 8.5°C
Humidity: 65.0%
Cat Present: YES
Blanket: ON
WiFi: DISCONNECTED
Next hourly photo in: 45 minutes
Time since last photo: 15 minutes
========================
```

## Troubleshooting

### Camera fails to initialize
- The system will automatically reboot and retry up to 3 times
- After 3 failed attempts, it enters **Safe Mode**
- Check ESP32-CAM connections
- Verify you're using the AI Thinker variant
- Ensure sufficient power supply (5V, 2A recommended)

### System in Safe Mode
If you see "SAFE MODE ACTIVE" in the serial output:
- Core functions (heating, temperature monitoring) continue to work
- Camera and photo uploads are disabled
- To exit safe mode:
  1. Fix the underlying issue (usually camera/power related)
  2. Power cycle the device
  3. Boot counter will reset after 5 minutes of successful operation

### WiFi won't connect
- Verify credentials in `secrets.h`
- Ensure router is on 2.4GHz (ESP32 doesn't support 5GHz)
- Check WiFi signal strength
- System forces WPA2-PSK authentication (not WPA3)

### DHT22 readings show NaN
- Check sensor connections
- Verify DHT22 is powered (3.3V or 5V)
- Ensure 10kΩ pull-up resistor on data line (some modules have it built-in)

### Photos not uploading
- Check S3 bucket configuration
- Verify AWS credentials
- Check WiFi connectivity
- Monitor serial output for error messages

### Checking Boot State
Boot attempts and safe mode status are stored in NVM. View them in the status report:
```
Boot attempts: 1/3
Mode: NORMAL
```

## License

This is a hobby project. Use at your own risk.
