/**
 * @file esp32-dht11-mqtt.ino
 * @brief ESP32 application reading DHT11 data, publishing via MQTT, and displaying info.
 *
 * Version: 1.2
 * Updated: 20/04/2025
 *
 * Reads data from a DHT sensor, publishes the data over MQTT,
 * and displays sensor readings on a selected display.
 *
 * Hardware: ESP32, DHT11, Nokia 5110 LCD/PCD8544, optional DS1302 RTC.
 * Dependencies: WiFi, PubSubClient, DHT, Adafruit_GFX, Adafruit_PCD8544, ESP32Time, FreeRTOS.
 */

// ---------------------- Includes ----------------------

// Standard Arduino core library
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
#include "default_config.h"  // Default credentials

// WiFi & MQTT setup helper libraries
#include <WiFiManager.h>  // https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>

// Display libraries based on configuration
#if SCREEN_SELECTION == SCREEN_SELECTION_LCD
#include <LiquidCrystal.h>
#elif SCREEN_SELECTION == SCREEN_SELECTION_PCD8544
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#endif

// ---------------------- Screen Selection ----------------------
#define SCREEN_SELECTION_LCD 1
#define SCREEN_SELECTION_PCD8544 2

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
#define UTC_OFFSET 0      // UTC offset; adjust if needed
#define UTC_OFFSET_DST 0  // DST offset; 0 if not applicable

// Interval for reading sensor data (20 seconds)
#define POST_INTERVAL 20000

// DS1302 RTC pins (if using RTC)
#define PIN_ENA 26  // Enable pin for RTC
#define PIN_CLK 14  // Clock pin for RTC
#define PIN_DAT 27  // Data pin for RTC

#if SCREEN_SELECTION == SCREEN_SELECTION_LCD
// ---------------------- LCD Pin Configuration ----------------------

#define LCD_RS_PIN 4   // LCD RS pin
#define LCD_EN_PIN 16  // LCD Enable pin
#define LCD_D4_PIN 18  // LCD Data pin 4
#define LCD_D5_PIN 19  // LCD Data pin 5
#define LCD_D6_PIN 5   // LCD Data pin 6
#define LCD_D7_PIN 17  // LCD Data pin 7
#elif SCREEN_SELECTION == SCREEN_SELECTION_PCD8544
// ---------------------- PCD8544 Display Pin Configuration ----------------------
#define SCLK_PIN 18  // SCLK
#define DIN_PIN 23   // DIN
#define DC_PIN 5     // DC
#define CS_PIN 17    // CS
#define RST_PIN 16   // RST
#endif

// ---------------------- Enumerations ----------------------
enum DeviceMode {
  NORMAL_MODE,
  CONFIG_MODE,
  SLEEP_MODE
};

// ---------------------- Global Configuration ----------------------
// (Constants and configuration defined above apply globally)

// ---------------------- Global Variables ----------------------

// Current device mode
DeviceMode deviceMode;

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
bool timeInitialized = false;    // True after successful NTP synchronization
bool updateDisplayFlag = false;  // Triggers display updates

// Queue handles for inter-task communication
QueueHandle_t sensorQueue;   // Transmit sensor data
QueueHandle_t displayQueue;  // Transmit display update information

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
WiFiManager wifiMqttManager;     // WiFi & MQTT configuration manager
WiFiClient espClient;            // WiFi client instance
PubSubClient client(espClient);  // MQTT client using the WiFi client

// RTC instance
ESP32Time rtc(3 * 3600);  // RTC object with GMT+3 offset

// ---------------------- Struct Definitions ----------------------
/**
 * @brief Structure for holding sensor readings.
 */
struct SensorData {
  float humidity;
  float temperature;
  float heatIndex;
};

// ---------------------- Helper Functions ----------------------
/**
 * @brief Prints the current WiFi connection status.
 */
void printWiFiStatus() {
  Serial.println(WiFi.status() == WL_CONNECTED ? F("WiFi connected!") : F("WiFi not connected!"));
}

/**
 * @brief Prints the current MQTT connection status.
 */
void printMQTTStatus() {
  Serial.println(client.connected() ? F("MQTT connected!") : F("MQTT not connected!"));
}

/**
 * @brief Loads the device mode configuration from SPIFFS.
 *
 * Reads "/device_mode.cfg" and sets the device mode. Valid values:
 * "NORMAL", "CONFIG", "SLEEP". Defaults to NORMAL_MODE if not found.
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
 * Writes the current mode ("NORMAL", "CONFIG", or "SLEEP") to "/device_mode.cfg".
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

/**
 * @brief Loads WiFi and MQTT configuration from SPIFFS.
 *
 * First, assigns default values from defined macros. Then, if a valid "/config.json"
 * is found, updates the configuration from the JSON document.
 *
 * @return true if the configuration file was parsed successfully, false otherwise.
 */
bool loadWiFiMQTTConfig() {
  // Use default configuration values
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

/**
 * @brief Saves the current WiFi and MQTT configuration to SPIFFS.
 *
 * Writes configuration parameters to "/config.json" in JSON format.
 */
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

/**
 * @brief Retrieves a shortened, unique chip ID in hexadecimal format.
 *
 * Extracts parts of the ESP32's MAC address to form a 32-bit chip ID,
 * converts it to a hexadecimal string, and pads it to 8 characters.
 *
 * @return A zero-padded, uppercase hexadecimal string representing the chip ID.
 */
String getChipIdHex() {
  uint32_t chipId = 0;

  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }

  String chipIdHex = String(chipId, HEX);
  chipIdHex.toUpperCase(); // Optional: make it uppercase
  chipIdHex = chipIdHex.length() < 8 ? String("00000000").substring(chipIdHex.length()) + chipIdHex : chipIdHex; // zero pad
  return chipIdHex;
}

/**
 * @brief Begins the WiFi/MQTT configuration portal.
 *
 * Displays a configuration portal to update WiFi and MQTT settings.
 * After obtaining input, saves new settings and restarts the device.
 */
void beginWiFiMQTTConfig() {
  // Use stored credentials if available
  // (Call loadWiFiMQTTConfig() if desired before proceeding)

  // Generate an AP SSID based on the device's MAC address.
  String apSSID = "ESP_" + getChipIdHex();

  // Define custom MQTT parameters for the configuration portal.
  WiFiManagerParameter custom_mqtt_server("server", "MQTT Server", MqttServer.c_str(), 40);
  WiFiManagerParameter custom_mqtt_clientid("clientid", "MQTT Client ID", MqttClientID.c_str(), 40);
  WiFiManagerParameter custom_mqtt_user("mqttuser", "MQTT User", MqttUser.c_str(), 40);
  WiFiManagerParameter custom_mqtt_pass("mqttpass", "MQTT Password", MqttPass.c_str(), 40);
  WiFiManagerParameter custom_publish_topic("pubtopic", "Publish Topic", PublishTopic.c_str(), 40);
  WiFiManagerParameter custom_subscribe_topic("subtopic", "Subscribe Topic", SubscribeTopic.c_str(), 40);

  // Add custom parameters to the WiFiManager.
  wifiMqttManager.addParameter(&custom_mqtt_server);
  wifiMqttManager.addParameter(&custom_mqtt_clientid);
  wifiMqttManager.addParameter(&custom_mqtt_user);
  wifiMqttManager.addParameter(&custom_mqtt_pass);
  wifiMqttManager.addParameter(&custom_publish_topic);
  wifiMqttManager.addParameter(&custom_subscribe_topic);

  // Add a custom HTML head element to display the MQTT port.
  wifiMqttManager.setCustomHeadElement("<p style='color:blue;text-align:center;'>MQTT Port is set to: 1883</p>");

  // Launch the configuration portal; blocks until connection is successful.
  if (!wifiMqttManager.autoConnect(apSSID.c_str())) {
    Serial.println("Failed to connect and hit timeout");
    ESP.restart();
  }

  // Save the received WiFi credentials.
  WifiSSID = WiFi.SSID();
  WifiPass = WiFi.psk();

  // Save updated MQTT parameters.
  MqttServer = String(custom_mqtt_server.getValue());
  MqttClientID = String(custom_mqtt_clientid.getValue());
  MqttUser = String(custom_mqtt_user.getValue());
  MqttPass = String(custom_mqtt_pass.getValue());
  PublishTopic = String(custom_publish_topic.getValue());
  SubscribeTopic = String(custom_subscribe_topic.getValue());

  // Save new configuration to SPIFFS.
  saveWiFiMQTTConfig();

  Serial.println("Connected to WiFi with SSID: " + WifiSSID);
}

/**
 * @brief Initializes NTP time synchronization and updates the RTC.
 *
 * Configures NTP with the provided UTC offsets and sets the local RTC.
 */
void initializeNTP() {
  if (!timeInitialized) {
    configTime(UTC_OFFSET, UTC_OFFSET_DST, NTP_SERVER);
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      rtc.setTimeStruct(timeinfo);  // Update RTC with local time
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
 * Attempts a connection using stored credentials and initializes NTP on success.
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
    vTaskDelay(pdMS_TO_TICKS(300));  // Wait for 300 ms before next attempt
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("\nWiFi connected! IP: "));
    Serial.println(WiFi.localIP().toString());
    updateDisplayFlag = true;  // Trigger display update for WiFi status
    initializeNTP();           // Initialize NTP after connection
  } else {
    Serial.println(F("\nWiFi connection failed, retrying..."));
  }
}

/**
 * @brief Connects to the MQTT broker and subscribes to a topic.
 *
 * Attempts an MQTT connection using stored credentials and subscribes to the configured topic.
 */
void connectToMQTT() {
  Serial.print("Connecting to MQTT...");
  if (client.connect(MqttClientID.c_str(), MqttUser.c_str(), MqttPass.c_str())) {
    Serial.println("Connected!");
    updateDisplayFlag = true;                  // Update display for MQTT status
    client.subscribe(SubscribeTopic.c_str());  // Subscribe to incoming messages
  } else {
    Serial.println("MQTT connection failed, retrying...");
  }
}

/**
 * @brief Callback to handle incoming MQTT messages.
 *
 * Constructs the message from payload and toggles the onboard LED based on the received value.
 *
 * @param topic Topic on which the message was received.
 * @param payload Message payload.
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
 * @brief Task to maintain the WiFi connection.
 *
 * Checks connection status and, after a certain number of failures, restarts
 * the device in configuration mode.
 *
 * @param pvParameters Task parameters.
 */
void wifiTask(void *pvParameters) {
  int wifiFailCount = 0;
  WiFi.mode(WIFI_STA);  // Set WiFi mode to Station
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
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

/**
 * @brief Task to maintain the MQTT connection.
 *
 * Checks the MQTT connection and loops the client. On consecutive failures,
 * restarts the device in configuration mode.
 *
 * @param pvParameters Task parameters.
 */
void mqttTask(void *pvParameters) {
  int mqttFailCount = 0;
  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(1000));
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
 * @brief Task to read sensor data from the DHT sensor.
 *
 * Reads sensor values and sends the data to the sensor and display queues.
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

    // Validate sensor readings
    if (!isnan(sensorData.humidity) && !isnan(sensorData.temperature)) {
      if (xQueueSend(sensorQueue, &sensorData, portMAX_DELAY) == pdTRUE) {
        Serial.println("Sensor data sent to sensorQueue");
      } else {
        Serial.println("Failed to send sensor data to sensorQueue");
      }

      if (xQueueOverwrite(displayQueue, &sensorData) == pdTRUE) {
        Serial.println("Sensor data sent to displayQueue");
        updateDisplayFlag = true;
      } else {
        Serial.println("Failed to send sensor data to displayQueue");
      }
    } else {
      Serial.println("DHT sensor read failed!");
    }
    vTaskDelay(pdMS_TO_TICKS(20000));  // Delay before the next reading
  }
}

/**
 * @brief Task to publish sensor data over MQTT.
 *
 * Waits for sensor data from the sensorQueue and publishes it.
 *
 * @param pvParameters Task parameters.
 */
void publishTask(void *pvParameters) {
  SensorData sensorData;
  char payload[100];
  while (true) {
    if (xQueueReceive(sensorQueue, &sensorData, pdMS_TO_TICKS(200))) {
      Serial.println("Data received from sensorQueue: Hum=" + String(sensorData.humidity) + ", Temp=" + String(sensorData.temperature) + ", HeatIndex=" + String(sensorData.heatIndex));
      snprintf(payload, sizeof(payload), "field1=%.2f&field2=%.2f&field3=%.2f&status=MQTTPUBLISH",
               sensorData.humidity, sensorData.temperature, sensorData.heatIndex);
      if (client.connected()) {
        if (client.publish(PublishTopic.c_str(), payload)) {
          Serial.println("Published: " + String(payload));
        } else {
          Serial.println("Publish failed!");
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

/**
 * @brief Task to update the display.
 *
 * Refreshes the display with the current date, time, connection status, and sensor data.
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
      // Clear sensor information region
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
      // Update 16x2 LCD Display
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
      updateDisplayFlag = false;
    }
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
  delay(3000);  // Show for 3 seconds
}

/**
 * @brief Application setup.
 */
void setup() {
  Serial.begin(115200);
  Serial.println(F("DHT Sensor with MQTT, Display Selection, and FreeRTOS"));

  // Initialize SPIFFS early if needed
  if (!SPIFFS.begin(true)) {
    Serial.println("Failed to mount SPIFFS in setup!");
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // Ensure LED is off initially

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

  showOpeningScreen();  // Display the opening screen

  loadDeviceModeConfig();
  if (deviceMode == CONFIG_MODE) {
    beginWiFiMQTTConfig();
    deviceMode = NORMAL_MODE;
    saveDeviceModeConfig();
    ESP.restart();
  } else if (deviceMode == SLEEP_MODE) {
    // Implement sleep mode if necessary
  }
  loadWiFiMQTTConfig();

  WiFi.mode(WIFI_STA);
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
 * @brief Main loop.
 *
 * The loop is left empty as FreeRTOS tasks handle the application logic.
 */
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));  // Idle delay for main loop
}