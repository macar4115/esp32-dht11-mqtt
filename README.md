# ESP32 DHT11 with MQTT, PCD8544 Display, and FreeRTOS

This project implements a DHT11 sensor (for temperature and humidity), an MQTT client for remote data publishing, a PCD8544 LCD display for showing real-time data, and FreeRTOS for multitasking. The project uses an ESP8266/ESP32 microcontroller for handling WiFi, MQTT, and sensor readings, while leveraging FreeRTOS to run tasks in parallel.

## Features

- **WiFi Connectivity**: Connects to a WiFi network and initializes NTP for time synchronization.
- **MQTT Communication**: Publishes sensor data (temperature, humidity, heat index) to a remote broker and subscribes to control topics for the onboard LED.
- **DHT11 Sensor**: Reads and processes sensor data.
- **Dual Display Options**:
  - **PCD8544 LCD Display**: An 84x48 pixel monochrome display for detailed status.
  - **16x2 LCD Display (LiquidCrystal)**: Use this alternative for alphanumeric output by including the LiquidCrystal library and adjusting wiring.
- **FreeRTOS Tasks**: Uses FreeRTOS to manage multiple concurrent tasks for WiFi, MQTT, sensor updates, data publishing, and display management.
- **Time Synchronization**: Integrates an RTC using NTP and displays the current date and time.
- **Startup Screen**: Shows a welcome message on device startup.

## Hardware

- **ESP32 Microcontroller** (e.g., ESP32 Dev Board)
- **DHT11 Sensor**: For measuring temperature and humidity.
- **Display Options**:
  - **PCD8544 LCD Display**:  
    - **SCLK (Serial Clock)**: GPIO18 (D5)
    - **DIN (Data In)**: GPIO23 (D7)
    - **DC (Data/Command)**: GPIO5 (D1)
    - **CS (Chip Select)**: GPIO17 (D0)
    - **RST (Reset)**: GPIO16 (D4)
  - **16x2 LCD Display (LiquidCrystal)**: Alternatively, adjust wiring and include the LiquidCrystal library.
- **LED**: The onboard LED is controlled via MQTT subscriptions.

## Dependencies

- **ESP32 WiFi Library**: Handles WiFi connectivity.
- **PubSubClient**: Lightweight MQTT client.
- **DHT Sensor Library**: For interfacing with the DHT11 sensor.
- **Adafruit PCD8544**: For controlling the PCD8544 LCD display.
- **FreeRTOS**: Manages multitasking.
- **ESP32Time**: Provides RTC functionality with NTP synchronization.

## Software Configuration

### WiFi Configuration

Configure your network credentials within esp32-dht11-mqtt.ino:

```cpp
#define WIFI_SSID "your_ssid"      // Replace with your WiFi network name
#define WIFI_PASS "your_password"  // Replace with your WiFi password
```

### MQTT Broker Configuration

Set your MQTT broker details to enable data publishing and subscription:

```cpp
#define MQTT_SERVER "your_mqtt_server"  // MQTT broker address
#define MQTT_PORT 1883                  // MQTT broker port
#define MQTT_CLIENT_ID "your_client_id"  // Unique client ID for the connection
#define MQTT_USER "your_user"           // MQTT username
#define MQTT_PASS "your_pass"           // MQTT password
#define PUBLISH_TOPIC "your_publish_topic"   // Replace with your publish topic
#define SUBSCRIBE_TOPIC "your_subscribe_topic" // Replace with your subscribe topic
```

### NTP and Time Configuration

Set up the NTP server and time zone settings:

```cpp
#define NTP_SERVER "pool.ntp.org"  // NTP server address
#define UTC_OFFSET 0               // Time zone offset in seconds (adjust for your region)
#define UTC_OFFSET_DST 0           // Daylight savings offset in seconds
```

## Setup Instructions

1. **Install Dependencies**  
   Using the Arduino IDE or PlatformIO, install the required libraries:
   - **Adafruit PCD8544**
   - **PubSubClient**
   - **DHT sensor library**
   - **ESP32Time**

2. **Configure the Code**  
   Open the file `esp32-dht11-mqtt.ino`, update the WiFi credentials, MQTT details, and any necessary hardware pin assignments before compiling.

3. **Upload to the ESP32**  
   Select the ESP32 board in the Arduino IDE (or choose the proper environment in PlatformIO) and compile/upload the sketch.

4. **Monitor Execution**  
   Open the Serial Monitor to view connection statuses, sensor data, and debug messages.

5. **Using ThingSpeak**  
   If using ThingSpeak as your MQTT broker, ensure that your channel is properly set up and the field configuration matches the sensor data.

## Task Breakdown

- **WiFi Task**: Manages connection to WiFi and NTP time synchronization.
- **MQTT Task**: Establishes and maintains MQTT connections, handles publishing of sensor data, and processes incoming control messages.
- **Sensor Task**: Reads data from the DHT11 sensor periodically.
- **Publish Task**: Sends sensor readings to the MQTT broker.
- **Display Task**: Updates the PCD8544 (or optionally, the 16x2 LCD) with current sensor values, time, and connection statuses.

## Future Improvements

- **Upgrade Sensor**: Consider switching from DHT11 to DHT22 for higher accuracy.
- **Enhanced UI**: Expand the display interface for more detailed information.
- **Robust Error Handling**: Improve recovery procedures for WiFi and MQTT connection issues.

## License

This project is licensed under the MIT License. See the LICENSE file for details.