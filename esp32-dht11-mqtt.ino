#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif

#include <WiFiUdp.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <ESP32Time.h>

// ---------------------- Configuration ----------------------
// Pin configuration for onboard LED and DHT sensor
#define LED_BUILTIN 2
#define DHTPIN 12      // GPIO pin for DHT sensor
#define DHTTYPE DHT11  // DHT sensor type

// WiFi credentials
#ifndef WIFI_SSID
#define WIFI_SSID "your_wifi_ssid"
#endif

#ifndef WIFI_PASS
#define WIFI_PASS "your_wifi_password"
#endif

// MQTT Broker configuration
#ifndef MQTT_SERVER
#define MQTT_SERVER "your_mqtt_server"
#endif

#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif

#ifndef MQTT_CLIENT_ID
#define MQTT_CLIENT_ID "your_mqtt_client_id"
#endif

#ifndef MQTT_USER
#define MQTT_USER "your_mqtt_username"
#endif

#ifndef MQTT_PASS
#define MQTT_PASS "your_mqtt_password"
#endif

#ifndef PUBLISH_TOPIC
#define PUBLISH_TOPIC "your_publish_topic"
#endif

#ifndef SUBSCRIBE_TOPIC
#define SUBSCRIBE_TOPIC "your_subscribe_topic"
#endif

// NTP Time configuration
#define NTP_SERVER "pool.ntp.org"
#define UTC_OFFSET 0      // UTC offset (no offset here, adjust as needed)
#define UTC_OFFSET_DST 0  // Daylight saving time offset (0 means no DST)

// Interval for reading sensor data (20 seconds)
#define POST_INTERVAL 20000

// ---------------------- PCD8544 Display Pins ----------------------
#define SCLK_PIN 18  // GPIO14 (D5)
#define DIN_PIN 23   // GPIO13 (D7)
#define DC_PIN 5     // GPIO12 (D6)
#define CS_PIN 17    // GPIO15 (D8)
#define RST_PIN 16   // GPIO2  (D4)

// DS1302 RTC (Real-time clock) pins (if using RTC)
#define PIN_ENA 26  // Enable pin for RTC
#define PIN_CLK 14  // Clock pin for RTC
#define PIN_DAT 27  // Data pin for RTC

// ---------------------- Global Objects ----------------------
DHT dht(DHTPIN, DHTTYPE);                                                           // DHT sensor instance
WiFiClient espClient;                                                               // WiFi client
PubSubClient client(espClient);                                                     // MQTT client
SPIClass displaySPI(VSPI);                                                          // SPI display
Adafruit_PCD8544 display = Adafruit_PCD8544(DC_PIN, CS_PIN, RST_PIN, &displaySPI);  // Display object

QueueHandle_t sensorQueue;   // Queue for sensor data
QueueHandle_t displayQueue;  // Queue for display data

// ---------------------- NTP Setup ----------------------
bool timeInitialized = false;  // Flag to check if NTP time has been initialized

ESP32Time rtc(3 * 3600);  // RTC instance with GMT+3 offset (adjust as needed)

// ---------------------- Global SensorData Structure ----------------------
struct SensorData {
  float humidity;
  float temperature;
  float heatIndex;
};

// ---------------------- Helper Functions ----------------------
void printWiFiStatus() {
  Serial.println(WiFi.status() == WL_CONNECTED ? "WiFi connected!" : "WiFi not connected!");
}

void printMQTTStatus() {
  Serial.println(client.connected() ? "MQTT connected!" : "MQTT not connected!");
}

void initializeNTP() {
  if (!timeInitialized) {
    configTime(UTC_OFFSET, UTC_OFFSET_DST, NTP_SERVER);
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      rtc.setTimeStruct(timeinfo);  // Set RTC time
      timeInitialized = true;
      Serial.println("NTP time initialized.");
    } else {
      Serial.println("Failed to initialize NTP time.");
    }
  }
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi: " + String(WIFI_SSID) + " with password: " + String(WIFI_PASS));
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  for (int retryCount = 0; retryCount < 30; retryCount++) {
    if (WiFi.status() == WL_CONNECTED)
      break;
    Serial.print(".");
    vTaskDelay(pdMS_TO_TICKS(300));  // Wait for 300 ms before retrying
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected! IP: " + WiFi.localIP().toString());
    initializeNTP();  // Initialize NTP after WiFi connection
  } else {
    Serial.println("\nWiFi connection failed, retrying...");
  }
}

void connectToMQTT() {
  Serial.print("Connecting to MQTT...");
  if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS)) {
    Serial.println("Connected!");
    client.subscribe(SUBSCRIBE_TOPIC);  // Subscribe to the topic
  } else {
    Serial.println("MQTT connection failed, retrying...");
  }
}

// ---------------------- WiFi Task ----------------------
void wifiTask(void *pvParameters) {
  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      connectToWiFi();
    }
    // Delay for 5 seconds before the next check
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// ---------------------- MQTT Reconnect Task ----------------------
void mqttTask(void *pvParameters) {
  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for a second and then check again
      continue;
    }
    if (!client.connected()) {
      connectToMQTT();
    }
    client.loop();                   // Process incoming MQTT messages
    vTaskDelay(pdMS_TO_TICKS(100));  // Check every 100 ms
  }
}

// ---------------------- MQTT Callback ----------------------
void callback(char *topic, byte *payload, unsigned int length) {
  String message;
  message.reserve(length + 1);

  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.println("Message received [" + String(topic) + "]: " + message);

  if (String(topic) == SUBSCRIBE_TOPIC) {
    digitalWrite(LED_BUILTIN, message.equals("1") ? LOW : HIGH);  // Toggle LED based on the message
  }
}

// ---------------------- Sensor Task ----------------------
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

// ---------------------- Publish Task ----------------------
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
      if (client.publish(PUBLISH_TOPIC, payload)) {
        Serial.println("Published: " + String(payload));
      } else {
        Serial.println("Publish failed!");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(500));  // Delay to avoid overloading the task
  }
}

// ---------------------- Display Task ----------------------
void displayTask(void *pvParameters) {
  SensorData sensorData;
  char formattedDate[20];
  char formattedTime[10];
  while (true) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(BLACK);

    // Update time every second after WiFi is connected
    if (timeInitialized) {
      // Format and display date and time
      snprintf(formattedDate, sizeof(formattedDate), "%02d-%02d-%04d", rtc.getDay(), rtc.getMonth() + 1, rtc.getYear());
      snprintf(formattedTime, sizeof(formattedTime), "%02d:%02d:%02d", rtc.getHour(true), rtc.getMinute(), rtc.getSecond());

      // Display updated date and time
      Serial.println(formattedDate);
      Serial.println(formattedTime);
      display.println(formattedDate);
      display.println(formattedTime);
    }

    // Display WiFi and MQTT connection status
    display.print("WiFi:" + String(WiFi.status() == WL_CONNECTED ? "OK" : "NO"));
    display.println(" MQT:" + String(client.connected() ? "OK" : "NO"));

    // Display sensor data if available
    if (xQueueReceive(displayQueue, &sensorData, pdMS_TO_TICKS(200))) {
      Serial.println("Displaying data: Hum=" + String(sensorData.humidity) + ", Temp=" + String(sensorData.temperature) + ", HeatIndex=" + String(sensorData.heatIndex));
    }
    display.println("Hum:" + String(sensorData.humidity) + "%");
    display.println("Temp:" + String(sensorData.temperature) + "C");
    display.println("HeatIdx:" + String(sensorData.heatIndex) + "C");
    display.display();

    vTaskDelay(pdMS_TO_TICKS(2000));  // Update display every 2 seconds
  }
}

// ---------------------- Setup ----------------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("DHT Sensor with MQTT, PCD8544, and FreeRTOS"));

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // Ensure LED starts off

  WiFi.mode(WIFI_STA);
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);
  dht.begin();

  // Initialize Display
  display.begin();
  display.setContrast(75);
  display.display();
  delay(1);
  display.clearDisplay();
  display.display();

  // Create queues for sensor data and display updates
  sensorQueue = xQueueCreate(5, sizeof(SensorData));
  displayQueue = xQueueCreate(1, sizeof(SensorData));

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(wifiTask, "WiFiTask", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(mqttTask, "MQTTTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(publishTask, "PublishTask", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(displayTask, "DisplayTask", 2048, NULL, 1, NULL, 1);
}

// ---------------------- Loop (Unused) ----------------------
void loop() {
  vTaskDelete(NULL);  // Delete loop to prevent execution
}
