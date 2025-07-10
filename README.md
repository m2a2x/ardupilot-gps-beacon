# MAVLink Bridge for ESP32

A WiFi-enabled MAVLink bridge that connects your flight controller to ground control stations like QGroundControl. This ESP32-based device acts as a wireless bridge between your drone's flight controller and ground control software.

## What This Project Does

This ESP32 device:
- **Creates a WiFi Access Point** that your computer/tablet can connect to
- **Bridges MAVLink messages** between UDP (for ground stations) and UART (for flight controller)
- **Provides GPS functionality** for location-based missions
- **Displays status information** on an OLED screen
- **Supports multiple mission types** including Follow Me, Guided, Loiter, and Auto missions
- **Allows manual control** through a button interface

## Hardware Requirements

- **ESP32 Development Board** (ESP32-DOIT-DevKit-V1 or compatible)
- **OLED Display** (SSD1306, 128x64 pixels)
- **GPS Module** (NEO-6M or compatible)
- **Push Button** (for menu navigation)
- **Wires and breadboard** for connections

## Pin Connections

| Component | ESP32 Pin | Description |
|-----------|-----------|-------------|
| GPS RX    | GPIO 18   | GPS module TX |
| GPS TX    | GPIO 19   | GPS module RX |
| Flight Controller RX | GPIO 17 | Flight controller TX |
| Flight Controller TX | GPIO 16 | Flight controller RX |
| OLED SDA  | GPIO 22   | I2C Data |
| OLED SCL  | GPIO 23   | I2C Clock |
| Button    | GPIO 0    | Menu navigation |

## Setup Instructions

### 1. Install PlatformIO
Make sure you have PlatformIO installed in your development environment.

### 2. Configure WiFi Credentials
Edit `src/creds.h` and set your WiFi access point credentials:
```cpp
const char *ap_ssid = "YourWiFiName";
const char *ap_pass = "YourWiFiPassword";
```

### 3. Build and Upload
```bash
# Build the project
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor
```

### 4. Connect to Ground Station
1. Connect your computer/tablet to the WiFi network created by the ESP32
2. Open QGroundControl or your preferred ground control station
3. Add a UDP connection with:
   - **IP Address**: 192.168.4.1
   - **Port**: 14550

## Usage

### Basic Operation
1. Power on the ESP32
2. Wait for the WiFi access point to start
3. Connect your ground control station to the WiFi network
4. The ESP32 will automatically bridge MAVLink messages between your flight controller and ground station

### Menu Navigation
- **Short press** the button to cycle through menu options
- **Long press** to select/confirm an option

### Available Missions
- **Follow Me**: Drone follows your GPS position
- **Guided**: Manual waypoint navigation
- **Loiter**: Circle around a specific point
- **Auto**: Execute pre-programmed mission
- **Arm/Disarm**: Control drone arming state

### Status Display
The OLED screen shows:
- WiFi connection status
- GPS fix status
- Current mission mode
- Battery voltage (if connected)
- Data transfer statistics

## Features

- **Real-time MAVLink bridging** between UDP and UART
- **GPS integration** for location-based missions
- **Multiple mission types** support
- **Status monitoring** via OLED display
- **Button interface** for local control
- **FreeRTOS-based** multitasking
- **Configurable parameters** for different setups

## Troubleshooting

### Common Issues
1. **No WiFi connection**: Check if the ESP32 is powered and the access point is created
2. **No MAVLink data**: Verify UART connections to flight controller
3. **GPS not working**: Check GPS module connections and ensure clear sky view
4. **Display not working**: Verify I2C connections and power supply

### Serial Monitor
Connect to the ESP32 via USB and monitor at 57600 baud to see debug information and error messages.

## Configuration

Key configuration files:
- `src/conf.h`: Hardware pin definitions and network settings
- `src/creds.h`: WiFi credentials
- `platformio.ini`: Build configuration and dependencies

## Dependencies

- Arduino Framework for ESP32
- Adafruit SSD1306 (OLED display)
- Adafruit GFX Library
- TinyGPSPlus (GPS parsing)
- MAVLink v2.0 protocol

## License

This project is open source. Feel free to modify and distribute according to your needs. 