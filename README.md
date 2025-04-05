# DHT Sensor with MQTT, PCD8544 Display, and FreeRTOS

This project implements a DHT11 sensor (for temperature and humidity), an MQTT client for remote data publishing, a PCD8544 LCD display for showing real-time data, and FreeRTOS for multitasking. The project uses an ESP8266/ESP32 microcontroller for handling WiFi, MQTT, and sensor readings, while leveraging FreeRTOS to run tasks in parallel.

## Features

- **WiFi Connectivity**: Connects to a WiFi network and initializes an NTP time server.
- **MQTT Communication**: Publishes sensor data to ThingSpeak via MQTT and subscribes to control messages for the onboard LED.
- **DHT11 Sensor**: Reads temperature and humidity, calculates heat index, and publishes the data.
- **PCD8544 LCD Display**: Displays time, temperature, humidity, and connection statuses (WiFi and MQTT).
- **FreeRTOS Tasks**: Utilizes FreeRTOS for handling multiple tasks (WiFi connection, MQTT, sensor reading, data publishing, and display updates).
- **RTC (Real-Time Clock)**: Synchronizes time using NTP and displays the current date and time.

## Hardware

- **ESP8266/ESP32 Microcontroller** (e.g., NodeMCU, Wemos D1 Mini, ESP32 Dev Board)
- **DHT11 Sensor**: For measuring temperature and humidity.
- **PCD8544 LCD Display**: An 84x48 pixel monochrome display for showing data.
- **LED**: The onboard LED will be controlled via MQTT messages.

### Pin Configuration

- **DHT11 Sensor Pin**: GPIO12 (D6)
- **PCD8544 Display**:
  - **SCLK (Serial Clock)**: GPIO18 (D5)
  - **DIN (Data In)**: GPIO23 (D7)
  - **DC (Data/Command)**: GPIO5 (D1)
  - **CS (Chip Select)**: GPIO17 (D0)
  - **RST (Reset)**: GPIO16 (D4)
- **LED Pin**: GPIO2 (Built-in LED on most boards)

## Dependencies

- **ESP8266/ESP32 WiFi Library**: For WiFi connectivity and MQTT communication.
- **PubSubClient**: A lightweight MQTT client for Arduino.
- **DHT**: Library to interface with DHT sensors.
- **Adafruit PCD8544**: Library for controlling the PCD8544 LCD display.
- **FreeRTOS**: For task management and parallel execution of various operations.
- **ESP32Time**: A library for handling real-time clock and NTP time synchronization.

## Software Configuration

### WiFi Configuration

Define your WiFi credentials in the code to enable the device to connect to your network:

```cpp
#define WIFI_SSID "your_ssid"  // Replace with your WiFi network name
#define WIFI_PASS "your_password"  // Replace with your WiFi password
```

### MQTT Broker Configuration

Set up the MQTT broker details to enable data publishing and subscription:

```cpp
#define MQTT_SERVER "your_mqtt_server"  // MQTT broker address
#define MQTT_PORT 1883  // MQTT broker port
#define MQTT_CLIENT_ID "your_client_id"  // Unique client ID for the MQTT connection
#define MQTT_USER "your_user"  // MQTT username
#define MQTT_PASS "your_pass"  // MQTT password
#define PUBLISH_TOPIC "your_publish_topic"  // Masked for security
#define SUBSCRIBE_TOPIC "your_subscribe_topic"  // Masked for security
```

### NTP Configuration

Configure the NTP server and time zone settings for accurate time synchronization:

```cpp
#define NTP_SERVER "pool.ntp.org"  // NTP server address
#define UTC_OFFSET 0  // Time zone offset in seconds (adjust for your region)
#define UTC_OFFSET_DST 0  // Daylight saving time offset in seconds
```

### Additional Notes

- Ensure that the WiFi credentials, MQTT broker details, and NTP settings are correctly configured before uploading the code.
- Use secure credentials and avoid sharing sensitive information publicly.
- Test the configuration in a controlled environment to verify connectivity and functionality.

## Setup Instructions

### Step 1: Install Dependencies

1. Install the required libraries through the Arduino IDE:
   - **Adafruit PCD8544**
   - **PubSubClient**
   - **DHT sensor library**
   - **ESP32Time** (for NTP synchronization)

2. Ensure that you have the appropriate board definitions installed in the Arduino IDE for ESP8266 or ESP32.

### Step 2: Configure the Code

- Replace the WiFi credentials and MQTT broker details in the provided code.
- Ensure the correct pin assignments for your hardware.

### Step 3: Upload Code to Microcontroller

- Select your board (e.g., ESP8266/ESP32) in the Arduino IDE.
- Upload the code to your ESP8266/ESP32 device.

### Step 4: Monitor the Serial Output

- Open the Serial Monitor in the Arduino IDE to view debug messages and sensor data.

### Step 5: Use ThingSpeak

- If using ThingSpeak as the MQTT broker, ensure your ThingSpeak channel is set up to receive the data. You will need to create a channel and configure the fields to match the published data (e.g., Humidity, Temperature, Heat Index).

## Task Breakdown

- **WiFi Task**: Connects the ESP8266/ESP32 to the WiFi network and initializes the NTP time server.
- **MQTT Task**: Manages MQTT connections and subscriptions. It also handles publishing sensor data to the broker and subscribing to control topics for the LED.
- **Sensor Task**: Reads data from the DHT11 sensor every 20 seconds and sends it to the sensor and display queues.
- **Publish Task**: Collects data from the sensor queue and publishes it to the MQTT broker.
- **Display Task**: Updates the PCD8544 display every 2 seconds, showing the date, time, WiFi status, MQTT status, and sensor readings.

## Future Improvements

- **Use of a More Accurate Sensor**: Consider replacing the DHT11 with a DHT22 for more reliable temperature and humidity readings.
- **Improved Display Interface**: Add more detailed information or a more advanced user interface on the display.
- **Error Handling and Reconnection Strategies**: Implement more robust error handling, including retries for WiFi and MQTT connections.

## License

This project is licensed under the MIT License. See the LICENSE file for more information.