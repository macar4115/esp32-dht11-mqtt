# ESP32 DHT11 with MQTT, PCD8544 Display, and FreeRTOS

This project implements a DHT11 sensor (for temperature and humidity), an MQTT client for remote data publishing, a PCD8544 LCD display for showing real-time data, and FreeRTOS for multitasking. The project runs on an ESP32 (or ESP8266) and leverages a runtime configuration portal for WiFi and MQTT credentials.

## Features

- **WiFi Connectivity & NTP Sync**  
  Automatically connects to WiFi and synchronizes time via an NTP server. In case of repeated connection failures, the device enters a configuration mode to reset credentials.

- **MQTT Communication**  
  Publishes sensor data (temperature, humidity, heat index) to a remote broker and subscribes to control topics for on-board devices (e.g., LED). Configuration changes are saved persistently in SPIFFS.

- **DHT11 Sensor Monitoring**  
  Reads and processes sensor data with dedicated FreeRTOS tasks for timely updates.

- **Display Options**  
  - **PCD8544 LCD Display:** Provides an 84x48 pixel monochrome display for detailed status information.  
  - **16x2 LCD (LiquidCrystal):** An alternative for alphanumeric output with appropriate wiring adjustments.

- **FreeRTOS-Driven Tasks**  
  Manages simultaneous tasks: WiFi handling, MQTT communication, sensor reading, data publishing, and display updates.

- **Runtime Configuration**  
  A user-friendly configuration mode (CONFIG_MODE) launches a WiFiManager portal for updating WiFi and MQTT settings. New settings are stored in SPIFFS for use on subsequent boots.

## Hardware Requirements

- **ESP32 Microcontroller** (e.g., ESP32 Dev Board)
- **DHT11 Sensor** for temperature and humidity measurements
- **Display Module Options:**
  - **PCD8544 LCD Display:**  
    - SCLK (GPIO18)  
    - DIN (GPIO23)  
    - DC (GPIO5)  
    - CS (GPIO17)  
    - RST (GPIO16)
  - **16x2 LCD (LiquidCrystal):** Requires appropriate wiring and library inclusion.
- **Onboard LED:** Provides visual status feedback via MQTT commands.
- **Optional RTC:** DS1302 for enhanced real-time clock support alongside NTP.

## Dependencies

- ESP32/ESP8266 WiFi Library
- PubSubClient (MQTT)
- DHT Sensor Library
- Adafruit PCD8544 and Adafruit GFX
- FreeRTOS
- ESP32Time (for RTC functionality)
- WiFiManager (for runtime configuration)
- ArduinoJson (for JSON-based configuration file management)

## Software Configuration

### WiFi and MQTT Settings

Edit `esp32-dht11-mqtt.ino` to configure the default WiFi and MQTT parameters. If the device fails to connect to WiFi, it will switch to CONFIG_MODE and launch a WiFiManager portal. The updated settings are stored in SPIFFS at `/config.json`.

Example snippet:

```cpp
#define WIFI_SSID "your_ssid"      // Default WiFi network name
#define WIFI_PASS "your_password"  // Default WiFi password

#define MQTT_SERVER "your_mqtt_server"  // MQTT broker address
#define MQTT_PORT 1883                  // MQTT broker port
#define MQTT_CLIENT_ID "your_client_id"  // Unique client identifier
#define MQTT_USER "your_user"           // MQTT username
#define MQTT_PASS "your_pass"           // MQTT password
#define PUBLISH_TOPIC "your_publish_topic"     // Topic for sensor data
#define SUBSCRIBE_TOPIC "your_subscribe_topic" // Topic for control commands
```

### NTP and Time Settings

Configure the NTP server and time offsets for RTC synchronization:

```cpp
#define NTP_SERVER "pool.ntp.org"  // NTP server address
#define UTC_OFFSET 0               // Time zone offset in seconds (adjust as necessary)
#define UTC_OFFSET_DST 0           // Daylight savings offset, if applicable
```

## Setup Instructions

1. **Install Dependencies**  
   Using the Arduino IDE or PlatformIO, install the following libraries:
   - Adafruit PCD8544 and Adafruit GFX
   - PubSubClient
   - DHT Sensor Library
   - ESP32Time
   - WiFiManager
   - ArduinoJson

2. **Configure the Code**  
   Modify `esp32-dht11-mqtt.ino` to update credentials, MQTT settings, and hardware pin assignments as needed. Alternatively, if the device cannot connect to WiFi, it will launch CONFIG_MODE where you can update settings via a web interface.

3. **Upload to the ESP32**  
   In your Arduino IDE (or PlatformIO), select the ESP32 board, compile, and upload the sketch.

4. **Monitor Execution**  
   Open the Serial Monitor to view connection statuses, sensor data, and debug messages.

5. **Runtime Configuration**  
   When the device boots into CONFIG_MODE (after multiple connection failures), follow the on-screen instructions on the WiFiManager portal to update WiFi and MQTT settings. The new configuration will be saved to SPIFFS for future boots.

## Task Breakdown

- **WiFi Task:**  
  Manages the WiFi connection, monitors status, and initiates NTP synchronization.

- **MQTT Task:**  
  Maintains the MQTT connection, publishes sensor data, and processes incoming messages.

- **Sensor Task:**  
  Reads and validates data from the DHT11 sensor.

- **Publish Task:**  
  Formats and publishes sensor readings over MQTT.

- **Display Task:**  
  Updates the selected display with sensor values, connection statuses, and the current time.

## Future Improvements

- **Sensor Upgrade:**  
  Consider switching from the DHT11 to the DHT22 for improved accuracy.

- **Enhanced UI:**  
  Expand the display interface to show more detailed or graphical information.

- **Robust Error Handling:**  
  Introduce advanced recovery procedures for persistent WiFi or MQTT failures.

## License

This project is licensed under the MIT License. See the LICENSE file for details.