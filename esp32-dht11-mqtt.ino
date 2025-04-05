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
#define LED_BUILTIN 2
#define DHTPIN 12      // GPIO pin for DHT sensor
#define DHTTYPE DHT11  // DHT sensor type

#define WIFI_SSID "TTNET_ZyXEL_YV37"
#define WIFI_PASS "c42416eF4AdEb"

#define MQTT_SERVER "mqtt3.thingspeak.com"
#define MQTT_PORT 1883

#define MQTT_CLIENT_ID "AQUxLCcFDzsHIQkKESwlPR4"
#define MQTT_USER "AQUxLCcFDzsHIQkKESwlPR4"
#define MQTT_PASS "rW4XzewfWqCu3QOm64Wspo5N"

#define PUBLISH_TOPIC "channels/2835504/publish"
#define SUBSCRIBE_TOPIC "channels/2835504/subscribe/fields/field1"

#define NTP_SERVER "pool.ntp.org"
#define UTC_OFFSET (3 * 3600)
#define UTC_OFFSET_DST 0

#define POST_INTERVAL 20000  // 20 seconds between sensor data reads

// ---------------------- PCD8544 Display Pins ----------------------
#define SCLK_PIN 18  // GPIO14 (D5)
#define DIN_PIN 23   // GPIO13 (D7)
#define DC_PIN 5     // GPIO12 (D6)
#define CS_PIN 17    // GPIO15 (D8)
#define RST_PIN 16   // GPIO2  (D4)

// DS1302 RTC instance (Define your RTC pins)
#define PIN_ENA 26  // Define your EN pin
#define PIN_CLK 14  // Define your CLK pin
#define PIN_DAT 27  // Define your DAT pin

// ---------------------- Global Objects ----------------------
DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);
SPIClass displaySPI(VSPI);
Adafruit_PCD8544 display = Adafruit_PCD8544(DC_PIN, CS_PIN, RST_PIN, &displaySPI);

QueueHandle_t sensorQueue;
QueueHandle_t displayQueue;

// ---------------------- NTP Setup ----------------------
// Flag to indicate if time has been initialized
bool timeInitialized = false;

// ---------------------- Global SensorData Structure ----------------------
struct SensorData {
  float humidity;
  float temperature;
  float heatIndex;
};

// ---------------------- WiFi Task ----------------------
void wifiTask(void *pvParameters) {
  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.print("Connecting to WiFi...");
      WiFi.begin(WIFI_SSID, WIFI_PASS);

      const TickType_t retryDelay = pdMS_TO_TICKS(300);
      for (int retryCount = 0; retryCount < 30; retryCount++) {
        if (WiFi.status() == WL_CONNECTED) break;
        Serial.print(".");
        vTaskDelay(retryDelay);
      }
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected! IP: " + WiFi.localIP().toString());

        // Initialize NTP once WiFi is connected
        if (!timeInitialized) {
          configTime(UTC_OFFSET, UTC_OFFSET_DST, NTP_SERVER);
          timeInitialized = true;
        }
      } else {
        Serial.println("\nWiFi failed, retrying...");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5000));  // Check every 5 seconds
  }
}

// ---------------------- MQTT Reconnect Task ----------------------
void mqttTask(void *pvParameters) {
  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for a second and then check again
      continue;                         // Skip the rest of the loop and check WiFi again
    }
    if (!client.connected()) {
      Serial.print("Connecting to MQTT...");

      if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS)) {
        Serial.println("Connected!");
        client.subscribe(SUBSCRIBE_TOPIC);
      } else {
        Serial.println("Failed, retrying...");
      }
    }
    client.loop();
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
    digitalWrite(LED_BUILTIN, message.equals("1") ? LOW : HIGH);  // Control the LED state based on the message
  }
}

// ---------------------- Sensor Task ----------------------
void sensorTask(void *pvParameters) {
  SensorData sensorData;

  while (true) {
    // Read sensor data from the DHT sensor
    sensorData.humidity = dht.readHumidity();
    sensorData.temperature = dht.readTemperature();
    sensorData.heatIndex = dht.computeHeatIndex(sensorData.temperature, sensorData.humidity, false);

    if (!isnan(sensorData.humidity) && !isnan(sensorData.temperature)) {
      // Send sensor data to the sensorQueue
      if (xQueueSend(sensorQueue, &sensorData, portMAX_DELAY) == pdTRUE) {
        Serial.println("Sensor data sent to sensorQueue");
      } else {
        Serial.println("Failed to send sensor data to sensorQueue");
      }

      // Send sensor data to the displayQueue (overwrite the current value)
      if (xQueueOverwrite(displayQueue, &sensorData) == pdTRUE) {
        Serial.println("Sensor data sent to displayQueue");
      } else {
        Serial.println("Failed to send sensor data to displayQueue");
      }
    } else {
      Serial.println("DHT sensor read failed!");
    }

    vTaskDelay(pdMS_TO_TICKS(POST_INTERVAL));  // Wait for the next data read (20 seconds)
  }
}

// ---------------------- Publish Task ----------------------
void publishTask(void *pvParameters) {
  SensorData sensorData;
  char payload[100];
  while (true) {
    if (xQueueReceive(sensorQueue, &sensorData, pdMS_TO_TICKS(200))) {  // 200ms timeout
      Serial.println("Data received from sensorQueue: Hum=" + String(sensorData.humidity) + ", Temp=" + String(sensorData.temperature) + ", HeatIndex=" + String(sensorData.heatIndex));
      snprintf(payload, sizeof(payload), "field1=%.2f&field2=%.2f&field3=%.2f&status=MQTTPUBLISH",
               sensorData.humidity, sensorData.temperature, sensorData.heatIndex);

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
  struct tm timeinfo;
  char formattedDate[20];
  char formattedTime[10];
  while (true) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(BLACK);

    // Update time every second after WiFi is connected
    if (timeInitialized) {
      if (getLocalTime(&timeinfo)) {
        // Format and display date and time
        snprintf(formattedDate, sizeof(formattedDate), "%02d-%02d-%04d", timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
        snprintf(formattedTime, sizeof(formattedTime), "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

        // Display updated date and time
        Serial.println(formattedDate);
        Serial.println(formattedTime);
        display.println(formattedDate);
        display.println(formattedTime);
      }
    }

    // Display WiFi and MQTT connection status
    display.print("WiFi:" + String(WiFi.status() == WL_CONNECTED ? "OK" : "NO"));
    display.println(" MQT:" + String(client.connected() ? "OK" : "NO"));

    if (xQueueReceive(displayQueue, &sensorData, pdMS_TO_TICKS(200))) {  // 200ms timeout
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
