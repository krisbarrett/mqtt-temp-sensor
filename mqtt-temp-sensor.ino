#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include "secrets.h"

Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

WiFiClient net = WiFiClient();
MQTTClient client = MQTTClient(256);

void setupSensor() {
  Serial.print("Initializing BME280 sensor...");
  if (!bme.begin()) {
    Serial.println("failed");
    while (1) delay(10);
  }
  Serial.println("OK");
}

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("OK");
}

void setupMQTT() {
  digitalWrite(LED_BUILTIN, HIGH);

  if(WiFi.status() != WL_CONNECTED) {
    setupWiFi();
  }

  client.begin(MQTT_BROKER, 1883, net);
  Serial.print("Connecting to MQTT broker...");

  while (!client.connect(MQTT_CLIENT_ID)) {
    Serial.print(".");
    delay(100);
  }

  if(!client.connected()){
    Serial.println("fail");
    return;
  }

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("OK");
}

void setup() {
  Serial.begin(9600);
  delay(1500); // extra time to allow host to connect

  pinMode(LED_BUILTIN, OUTPUT);
  
  setupSensor();
}

void publishMeasurement(const char* topic, char* measurement, float val) {
  StaticJsonDocument<200> doc;
  doc["time"] = millis();
  doc[measurement] = val;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client

  Serial.print("PUBLISH ");
  Serial.print(topic);
  Serial.print(" - "); 
  Serial.print(jsonBuffer);
  Serial.print(" - ");

  if(!client.publish(topic, jsonBuffer)) {
    Serial.println("fail");
    return;
  };

  Serial.println("OK");
}

void loop() {
  if(!client.connected()) {
    setupMQTT();
  }

  sensors_event_t temp_event, humidity_event;
  bme_temp->getEvent(&temp_event);
  bme_humidity->getEvent(&humidity_event);
  
  publishMeasurement(TEMP_TOPIC, "temperature", temp_event.temperature);
  publishMeasurement(HUMIDITY_TOPIC, "humidity", humidity_event.relative_humidity);

  client.loop();
  delay(LOOP_DELAY);
}
