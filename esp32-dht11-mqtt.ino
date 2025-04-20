/**
 * @file esp32-dht11-mqtt.ino
 * @brief ESP32 DHT11 MQTT and Display Application
 *
 * Version: 1.1
 * Updated: 2023-10-XX
 *
 * Reads data from a DHT sensor, publishes the data over MQTT,
 * and displays the status and sensor readings on a selected display.
 *
 * Hardware: ESP32, DHT11, Nokia 5110 LCD/PCD8544, optional DS1302 RTC.
 * Dependencies: WiFi, PubSubClient, DHT, Adafruit_GFX, Adafruit_PCD8544, ESP32Time, FreeRTOS.
 */

// ---------------------- Includes ----------------------

// Standard Arduino core
#include <Arduino.h>

#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif

// Network libraries
#include <WiFiUdp.h>
#include <PubSubClient.h>

// Sensor and time libraries
#include "DHT.h"
#include <ESP32Time.h>

// File system and string support
#include "SPIFFS.h"
#include "WString.h"

// Configuration headers (local)
#include "default_config.h"  // Include default credentials

// WiFi & MQTT setup helper libraries
#include <WiFiManager.h>  // https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>

// Screen selection - include display libraries based on your configuration
#if SCREEN_SELECTION == SCREEN_SELECTION_LCD
#include <LiquidCrystal.h>
#elif SCREEN_SELECTION == SCREEN_SELECTION_PCD8544
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#endif

// Screen selection macros
#define SCREEN_SELECTION_LCD 1
#define SCREEN_SELECTION_PCD8544 2

// Choose a screen: set to SCREEN_SELECTION_LCD or SCREEN_SELECTION_PCD8544
#ifndef SCREEN_SELECTION
#define SCREEN_SELECTION SCREEN_SELECTION_LCD
#endif

// Pin configuration for onboard LED and DHT sensor
#define LED_BUILTIN 2
#define DHTPIN 12      // GPIO pin for DHT sensor
#define DHTTYPE DHT11  // DHT sensor type

#ifndef WIFI_FAIL_THRESHOLD
#define WIFI_FAIL_THRESHOLD 3
#endif

#ifndef MQTT_FAIL_THRESHOLD
#define MQTT_FAIL_THRESHOLD 3
#endif

// NTP Time configuration
#define NTP_SERVER "pool.ntp.org"
#define UTC_OFFSET 0      // UTC offset (no offset here, adjust as needed)
#define UTC_OFFSET_DST 0  // Daylight saving time offset (0 means no DST)

// Interval for reading sensor data (20 seconds)
#define POST_INTERVAL 20000

// DS1302 RTC (Real-time clock) pins (if using RTC)
#define PIN_ENA 26  // Enable pin for RTC
#define PIN_CLK 14  // Clock pin for RTC
#define PIN_DAT 27  // Data pin for RTC

#if SCREEN_SELECTION == SCREEN_SELECTION_LCD
// ---------------------- LCD Pin Macros ----------------------
#define LCD_RS_PIN 19  // LCD RS pin
#define LCD_EN_PIN 23  // LCD Enable pin
#define LCD_D4_PIN 18  // LCD Data pin 4
#define LCD_D5_PIN 17  // LCD Data pin 5
#define LCD_D6_PIN 16  // LCD Data pin 6
#define LCD_D7_PIN 15  // LCD Data pin 7
#elif SCREEN_SELECTION == SCREEN_SELECTION_PCD8544
// ---------------------- Conditional PCD8544 Display Pins ----------------------
#define SCLK_PIN 18  // GPIO14 (D5)
#define DIN_PIN 23   // GPIO13 (D7)
#define DC_PIN 5     // GPIO12 (D6)
#define CS_PIN 17    // GPIO15 (D8)
#define RST_PIN 16   // GPIO2  (D4)
#endif

// ---------------------- Enumerations ----------------------
enum DeviceMode {
  NORMAL_MODE,
  CONFIG_MODE,
  SLEEP_MODE
};

// ---------------------- Global Configuration ----------------------
// (Defines/Constants that apply globally are already set above)

// ---------------------- Global Variables ----------------------

// Device mode
DeviceMode deviceMode;  // Holds current operational mode

// WiFi & MQTT configuration strings
String WifiSSID = "";
String WifiPass = "";
String MqttServer = "";
String MqttClientID = "";
String MqttUser = "";
String MqttPass = "";
String PublishTopic = "";
String SubscribeTopic = "";
int MqttPort;  // MQTT broker port

// Flags
bool timeInitialized = false;    // Flag to indicate if NTP is synchronized
bool updateDisplayFlag = false;  // Flag to trigger display updates

// Queue handles for inter-task communication
QueueHandle_t sensorQueue;   // For passing sensor data
QueueHandle_t displayQueue;  // For passing display update info

// ---------------------- Global Objects ----------------------

// Display objects based on screen selection
#if SCREEN_SELECTION == SCREEN_SELECTION_LCD
LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);
#elif SCREEN_SELECTION == SCREEN_SELECTION_PCD8544
SPIClass displaySPI(VSPI);                                       // SPI interface for the display
Adafruit_PCD8544 display(DC_PIN, CS_PIN, RST_PIN, &displaySPI);  // PCD8544 Display object
#endif

// Sensor instance
DHT dht(DHTPIN, DHTTYPE);  // DHT sensor

// WiFi & MQTT instances
WiFiManager wifiMqttManager;     // Manager for WiFi & MQTT configuration and operations
WiFiClient espClient;            // WiFi client instance
PubSubClient client(espClient);  // MQTT client using espClient

// RTC instance
ESP32Time rtc(3 * 3600);  // RTC object with specified GMT+3 offset

// ---------------------- Struct Definitions ----------------------

/**
 * @brief Structure to hold sensor readings.
 */
struct SensorData {
  float humidity;
  float temperature;
  float heatIndex;
};

// ---------------------- Helper Functions ----------------------

/**
 * @brief Prints the current WiFi connection status to Serial.
 *
 * Logs "WiFi connected!" if connected, otherwise logs "WiFi not connected!".
 */
void printWiFiStatus() {
  Serial.println(WiFi.status() == WL_CONNECTED ? F("WiFi connected!") : F("WiFi not connected!"));
}

/**
 * @brief Prints the current MQTT connection status to Serial.
 *
 * Logs "MQTT connected!" if connected, otherwise logs "MQTT not connected!".
 */
void printMQTTStatus() {
  Serial.println(client.connected() ? F("MQTT connected!") : F("MQTT not connected!"));
}

/**
 * @brief Loads the device mode configuration from SPIFFS.
 *
 * Reads the file "/device_mode.cfg" to determine the operation mode.
 * Accepted values are "NORMAL", "CONFIG", or "SLEEP". Defaults to NORMAL_MODE if not found.
 */
void loadDeviceModeConfig() {
  deviceMode = NORMAL_MODE;
  File file = SPIFFS.open("/device_mode.cfg", "r");
  if (!file) {
    Serial.println("Device mode config file not found. Using NORMAL mode.");
    deviceMode = NORMAL_MODE;
    return;
  }
  String modeStr = file.readStringUntil('\n');
  modeStr.trim();
  if (modeStr.equalsIgnoreCase("CONFIG")) {
    deviceMode = CONFIG_MODE;
  } else if (modeStr.equalsIgnoreCase("SLEEP")) {
    deviceMode = SLEEP_MODE;
  } else {
    deviceMode = NORMAL_MODE;
  }
  Serial.println("Selected Device Mode: " + modeStr);
  file.close();
}

/**
 * @brief Saves the current device mode configuration to SPIFFS.
 *
 * Writes the current device mode (NORMAL, CONFIG, or SLEEP) to the file "/device_mode.cfg".
 */
void saveDeviceModeConfig() {
  if (!SPIFFS.begin(true)) {
    Serial.println("Failed to mount SPIFFS for saving device mode!");
    return;
  }

  File file = SPIFFS.open("/device_mode.cfg", FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open device mode config file for writing!");
    return;
  }

  String modeStr;
  switch (deviceMode) {
    case CONFIG_MODE:
      modeStr = "CONFIG";
      break;
    case SLEEP_MODE:
      modeStr = "SLEEP";
      break;
    case NORMAL_MODE:
    default:
      modeStr = "NORMAL";
      break;
  }

  file.println(modeStr);
  file.close();
  Serial.println("Device mode saved: " + modeStr);
}

bool loadWiFiMQTTConfig() {
  // Correct the assignment by using the exact global variable names
  WifiSSID = WIFI_SSID;
  WifiPass = WIFI_PASS;
  MqttServer = MQTT_SERVER;
  MqttPort = MQTT_PORT;
  MqttClientID = MQTT_CLIENT_ID;
  MqttUser = MQTT_USER;
  MqttPass = MQTT_PASS;
  PublishTopic = PUBLISH_TOPIC;
  SubscribeTopic = SUBSCRIBE_TOPIC;

  File configFile = SPIFFS.open("/config.json", "r");
  if (configFile) {
    size_t size = configFile.size();
    std::unique_ptr<char[]> buf(new char[size]);
    configFile.readBytes(buf.get(), size);
    configFile.close();

    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, buf.get());
    if (!error) {
      WifiSSID = doc["wifiSSID"].as<String>();
      WifiPass = doc["wifiPass"].as<String>();
      MqttServer = doc["mqttServer"].as<String>();
      MqttClientID = doc["mqttClientID"].as<String>();
      MqttUser = doc["mqttUser"].as<String>();
      MqttPass = doc["mqttPass"].as<String>();
      PublishTopic = doc["publishTopic"].as<String>();
      SubscribeTopic = doc["subscribeTopic"].as<String>();
      Serial.println("Loaded MQTT configuration from SPIFFS");
      return true;
    } else {
      Serial.println("Failed to parse config.json");
    }
  }
  Serial.println("Failed to open config.json for reading");
  return false;
}

void saveWiFiMQTTConfig() {
  DynamicJsonDocument doc(512);
  doc["wifiSSID"] = WifiSSID;
  doc["wifiPass"] = WifiPass;
  doc["mqttServer"] = MqttServer;
  doc["mqttClientID"] = MqttClientID;
  doc["mqttUser"] = MqttUser;
  doc["mqttPass"] = MqttPass;
  doc["publishTopic"] = PublishTopic;
  doc["subscribeTopic"] = SubscribeTopic;

  File configFile = SPIFFS.open("/config.json", "w");
  if (configFile) {
    serializeJson(doc, configFile);
    configFile.close();
    Serial.println("Saved MQTT configuration to SPIFFS");
  } else {
    Serial.println("Failed to open config file for writing");
  }
}

void beginWiFiMQTTConfig() {
  // In case you need to load stored credentials first:
  // If you have a custom load function, call it here; otherwise you can call loadWiFiMQTTConfig()

  // Get the device MAC address to form the AP SSID.
  String _mac = String(WiFi.macAddress());
  _mac.toLowerCase();
  _mac.replace(":", "");
  _mac.replace("240ac4", "a");  // vendor = Espressif Inc.
  String apSSID = "ESP_" + _mac;

  // Define custom parameters for MQTT configuration (values from JSON take precedence)
  WiFiManagerParameter custom_mqtt_server("server", "MQTT Server", MqttServer.c_str(), 40);
  WiFiManagerParameter custom_mqtt_clientid("clientid", "MQTT Client ID", MqttClientID.c_str(), 40);
  WiFiManagerParameter custom_mqtt_user("mqttuser", "MQTT User", MqttUser.c_str(), 40);
  WiFiManagerParameter custom_mqtt_pass("mqttpass", "MQTT Password", MqttPass.c_str(), 40);
  WiFiManagerParameter custom_publish_topic("pubtopic", "Publish Topic", PublishTopic.c_str(), 40);
  WiFiManagerParameter custom_subscribe_topic("subtopic", "Subscribe Topic", SubscribeTopic.c_str(), 40);

  // Use the global WiFiManager instance instead of 'wm'
  wifiMqttManager.addParameter(&custom_mqtt_server);
  wifiMqttManager.addParameter(&custom_mqtt_clientid);
  wifiMqttManager.addParameter(&custom_mqtt_user);
  wifiMqttManager.addParameter(&custom_mqtt_pass);
  wifiMqttManager.addParameter(&custom_publish_topic);
  wifiMqttManager.addParameter(&custom_subscribe_topic);

  // Add a custom HTML head element to display the MQTT port on the configuration page.
  wifiMqttManager.setCustomHeadElement("<p style='color:blue;text-align:center;'>MQTT Port is set to: 1883</p>");

  // Run the configuration portal. Will block until connected.
  if (!wifiMqttManager.autoConnect(apSSID.c_str())) {
    Serial.println("Failed to connect and hit timeout");
    ESP.restart();
  }

  // Save WiFi credentials.
  WifiSSID = WiFi.SSID();
  WifiPass = WiFi.psk();

  // Save MQTT parameters received from the portal input.
  MqttServer = String(custom_mqtt_server.getValue());
  MqttClientID = String(custom_mqtt_clientid.getValue());
  MqttUser = String(custom_mqtt_user.getValue());
  MqttPass = String(custom_mqtt_pass.getValue());
  PublishTopic = String(custom_publish_topic.getValue());
  SubscribeTopic = String(custom_subscribe_topic.getValue());

  // Save the MQTT configuration to SPIFFS so it persists across reboots.
  saveWiFiMQTTConfig();

  Serial.println("Connected to WiFi with SSID: " + WifiSSID);
}

/**
 * @brief Initializes NTP time synchronization and updates the RTC.
 *
 * Configures NTP with the provided UTC offsets and sets the timeInitialized flag upon successful sync.
 */
void initializeNTP() {
  if (!timeInitialized) {
    configTime(UTC_OFFSET, UTC_OFFSET_DST, NTP_SERVER);
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      rtc.setTimeStruct(timeinfo);  // Update local RTC
      timeInitialized = true;
      Serial.println(F("NTP time initialized."));
    } else {
      Serial.println(F("Failed to initialize NTP time."));
    }
  }
}

/**
 * @brief Connects to the configured WiFi network.
 *
 * Attempts connection with predefined credentials and initializes NTP on success.
 */
void connectToWiFi() {
  Serial.print(F("Connecting to WiFi: "));
  Serial.print(WifiSSID);
  Serial.print(F(" with password: "));
  Serial.println(WifiPass);
  WiFi.begin(WifiSSID, WifiPass);
  for (int retryCount = 0; retryCount < 30; retryCount++) {
    if (WiFi.status() == WL_CONNECTED)
      break;
    Serial.print(F("."));
    vTaskDelay(pdMS_TO_TICKS(300));  // Wait for 300 ms before retrying
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("\nWiFi connected! IP: "));
    Serial.println(WiFi.localIP().toString());
    updateDisplayFlag = true;  // Trigger display update for WiFi status change
    initializeNTP();           // Initialize NTP after WiFi connection
  } else {
    Serial.println(F("\nWiFi connection failed, retrying..."));
  }
}

/**
 * @brief Connects to the MQTT broker and subscribes to topics.
 *
 * Attempts MQTT connection using predefined credentials and subscribes to a topic if successful.
 */
void connectToMQTT() {
  Serial.print("Connecting to MQTT...");
  if (client.connect(MqttClientID.c_str(), MqttUser.c_str(), MqttPass.c_str())) {
    Serial.println("Connected!");
    updateDisplayFlag = true;                  // Trigger display update for MQTT status change
    client.subscribe(SubscribeTopic.c_str());  // Subscribe to the topic
  } else {
    Serial.println("MQTT connection failed, retrying...");
  }
}

/**
 * @brief Callback function to handle incoming MQTT messages.
 *
 * Constructs the message from payload and toggles the onboard LED based on the received value.
 *
 * @param topic Pointer to the MQTT topic.
 * @param payload Array containing the message payload.
 * @param length Length of the payload.
 */
void mqttCallback(char *topic, byte *payload, unsigned int length) {
  String message(payload, length);
  Serial.print(F("Message received ["));
  Serial.print(topic);
  Serial.print(F("]: "));
  Serial.println(message);

  if (String(topic) == SubscribeTopic) {
    digitalWrite(LED_BUILTIN, message.equals(F("1")) ? LOW : HIGH);
  }
}

// ---------------------- FreeRTOS Tasks ----------------------

/**
 * @brief FreeRTOS task to maintain the WiFi connection.
 *
 * Checks the WiFi status and counts connection attempts. After WIFI_FAIL_THRESHOLD failures,
 * starts the WiFi/MQTT Manager mode.
 *
 * @param pvParameters Task parameters.
 */
void wifiTask(void *pvParameters) {
  int wifiFailCount = 0;
  WiFi.mode(WIFI_STA);  // Set WiFi mode to Station mode
  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      connectToWiFi();
      if (WiFi.status() != WL_CONNECTED) {
        wifiFailCount++;
        if (wifiFailCount >= WIFI_FAIL_THRESHOLD) {
          Serial.println(F("Multiple WiFi failures detected. Launching WiFi/MQTT Manager."));

          deviceMode = CONFIG_MODE;
          saveDeviceModeConfig();
          ESP.restart();
          wifiFailCount = 0;
        }
      } else {
        wifiFailCount = 0;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5000));  // Delay before the next check
  }
}

/**
 * @brief FreeRTOS task to manage the MQTT connection.
 *
 * Checks MQTT connection and counts failures. After MQTT_FAIL_THRESHOLD failures,
 * starts the WiFi/MQTT Manager mode.
 *
 * @param pvParameters Task parameters.
 */
void mqttTask(void *pvParameters) {
  int mqttFailCount = 0;
  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for WiFi connection
      continue;
    }
    if (!client.connected()) {
      connectToMQTT();
      if (!client.connected()) {
        mqttFailCount++;
        if (mqttFailCount >= MQTT_FAIL_THRESHOLD) {
          Serial.println(F("Multiple MQTT failures detected. Launching WiFi/MQTT Manager."));
          deviceMode = CONFIG_MODE;
          saveDeviceModeConfig();
          ESP.restart();
          mqttFailCount = 0;
        }
      } else {
        mqttFailCount = 0;
      }
    }
    client.loop();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/**
 * @brief FreeRTOS task to read sensor data from the DHT sensor.
 *
 * Reads sensor values, validates them, and sends the data to both sensorQueue and displayQueue.
 *
 * @param pvParameters Task parameters.
 */
void sensorTask(void *pvParameters) {
  SensorData sensorData;
  while (true) {
    // Read data from DHT sensor
    sensorData.humidity = dht.readHumidity();
    sensorData.temperature = dht.readTemperature();
    sensorData.heatIndex = dht.computeHeatIndex(sensorData.temperature, sensorData.humidity, false);

    // Check if sensor readings are valid
    if (!isnan(sensorData.humidity) && !isnan(sensorData.temperature)) {
      // Send sensor data to sensorQueue
      if (xQueueSend(sensorQueue, &sensorData, portMAX_DELAY) == pdTRUE) {
        Serial.println("Sensor data sent to sensorQueue");
      } else {
        Serial.println("Failed to send sensor data to sensorQueue");
      }

      // Overwrite the current sensor data in the displayQueue
      if (xQueueOverwrite(displayQueue, &sensorData) == pdTRUE) {
        Serial.println("Sensor data sent to displayQueue");
        // Set the flag to trigger a display update
        updateDisplayFlag = true;
      } else {
        Serial.println("Failed to send sensor data to displayQueue");
      }
    } else {
      Serial.println("DHT sensor read failed!");
    }

    // Delay for 20 seconds before the next sensor reading
    vTaskDelay(pdMS_TO_TICKS(20000));
  }
}

/**
 * @brief FreeRTOS task to publish sensor data over MQTT.
 *
 * Waits for sensor data from sensorQueue and publishes it to the MQTT topic.
 *
 * @param pvParameters Task parameters.
 */
void publishTask(void *pvParameters) {
  SensorData sensorData;
  char payload[100];
  while (true) {
    // Wait for data in the sensorQueue with a 200ms timeout
    if (xQueueReceive(sensorQueue, &sensorData, pdMS_TO_TICKS(200))) {
      Serial.println("Data received from sensorQueue: Hum=" + String(sensorData.humidity) + ", Temp=" + String(sensorData.temperature) + ", HeatIndex=" + String(sensorData.heatIndex));
      snprintf(payload, sizeof(payload), "field1=%.2f&field2=%.2f&field3=%.2f&status=MQTTPUBLISH",
               sensorData.humidity, sensorData.temperature, sensorData.heatIndex);

      // Publish data to the MQTT topic
      if (client.publish(PublishTopic.c_str(), payload)) {
        Serial.println("Published: " + String(payload));
      } else {
        Serial.println("Publish failed!");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(500));  // Delay to avoid overloading the task
  }
}

/**
 * @brief FreeRTOS task to update the display.
 *
 * Refreshes the PCD8544 display with the current date, time, WiFi/MQTT status, and sensor readings.
 *
 * @param pvParameters Task parameters.
 */
void displayTask(void *pvParameters) {
  SensorData sensorData;
#if SCREEN_SELECTION == SCREEN_SELECTION_PCD8544
  char formattedDate[20];
  char formattedTime[10];
#endif
  while (true) {
    // Only update the display when new data is available
    if (updateDisplayFlag) {
#if SCREEN_SELECTION == SCREEN_SELECTION_PCD8544
      // Update PCD8544 Display
      display.fillRect(0, 0, 84, 16, WHITE);
      display.setCursor(0, 0);
      display.setTextSize(1);
      display.setTextColor(BLACK);
      if (timeInitialized) {
        snprintf(formattedDate, sizeof(formattedDate), "%02d-%02d-%04d", rtc.getDay(), rtc.getMonth() + 1, rtc.getYear());
        snprintf(formattedTime, sizeof(formattedTime), "%02d:%02d:%02d", rtc.getHour(true), rtc.getMinute(), rtc.getSecond());
        display.println(formattedDate);
        display.println(formattedTime);
      }
      // Clear sensor info region (starting at y=16)
      display.fillRect(0, 16, 84, 32, WHITE);
      display.setCursor(0, 16);
      display.print("WiFi:" + String(WiFi.status() == WL_CONNECTED ? "OK" : "NO"));
      display.println(" MQT:" + String(client.connected() ? "OK" : "NO"));
      if (xQueuePeek(displayQueue, &sensorData, pdMS_TO_TICKS(200)) == pdTRUE) {
        display.println("Hum:" + String(sensorData.humidity) + "%");
        display.println("Temp:" + String(sensorData.temperature) + "C");
        display.println("Heat:" + String(sensorData.heatIndex) + "C");
      }
      display.display();
#elif SCREEN_SELECTION == SCREEN_SELECTION_LCD
      // Update LCD Display (assuming 16x2 LCD)
      lcd.clear();
      lcd.setCursor(0, 0);
      String status = "WiFi:" + String(WiFi.status() == WL_CONNECTED ? "OK" : "NO") + " MQTT:" + String(client.connected() ? "OK" : "NO");
      lcd.print(status);
      lcd.setCursor(0, 1);
      if (xQueuePeek(displayQueue, &sensorData, pdMS_TO_TICKS(200)) == pdTRUE) {
        lcd.print("T:" + String(sensorData.temperature, 1) + "C ");
        lcd.print("H:" + String(sensorData.humidity, 1) + "%");
      }
#endif
      // Clear the update flag after a successful screen update
      updateDisplayFlag = false;
    }
    // Check frequently for new data
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

/**
 * @brief Displays the opening screen during startup.
 */
void showOpeningScreen() {
#if SCREEN_SELECTION == SCREEN_SELECTION_PCD8544
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(0, 0);
  display.println("Welcome to");
  display.println("DHT Sensor App");
  display.println("Initializing...");
  display.display();
#elif SCREEN_SELECTION == SCREEN_SELECTION_LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Welcome to");
  lcd.setCursor(0, 1);
  lcd.print("DHT Sensor App");
#endif
  delay(3000);  // Show opening screen for 3 seconds
}

/**
 * @brief The setup function.
 */
void setup() {
  Serial.begin(115200);
  Serial.println(F("DHT Sensor with MQTT, Display Selection, and FreeRTOS"));

  // Initialize SPIFFS early if needed
  if (!SPIFFS.begin(true)) {
    Serial.println("Failed to mount SPIFFS in setup!");
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // Ensure LED starts off

#if SCREEN_SELECTION == SCREEN_SELECTION_LCD
  // Initialize LCD (adjust columns and rows as needed)
  lcd.begin(16, 2);
#elif SCREEN_SELECTION == SCREEN_SELECTION_PCD8544
  // Initialize PCD8544 Display
  display.begin();
  display.setContrast(75);
  display.display();
  delay(1);
  display.clearDisplay();
  display.display();
#endif

  showOpeningScreen();  // Display opening screen

  loadDeviceModeConfig();
  if (deviceMode == CONFIG_MODE) {
    beginWiFiMQTTConfig();
    deviceMode = NORMAL_MODE;
    saveDeviceModeConfig();
    ESP.restart();
  } else if (deviceMode == SLEEP_MODE) {

  } else {
  }
  loadWiFiMQTTConfig();

  WiFi.mode(WIFI_STA);

  // Set the MQTT server using defined MQTT_SERVER and MQTT_PORT.
  client.setServer(MqttServer.c_str(), MqttPort);

  client.setCallback(mqttCallback);
  dht.begin();

  sensorQueue = xQueueCreate(5, sizeof(SensorData));
  displayQueue = xQueueCreate(1, sizeof(SensorData));

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(wifiTask, "WiFiTask", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(mqttTask, "MQTTTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(publishTask, "PublishTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(displayTask, "DisplayTask", 2048, NULL, 1, NULL, 1);
}

/**
 * @brief The main loop function.
 *
 * Deletes itself to prevent execution, as FreeRTOS tasks handle the application logic.
 */
void loop() {
  // Keep the loop task alive while doing nothing
  vTaskDelay(pdMS_TO_TICKS(1000));
}
