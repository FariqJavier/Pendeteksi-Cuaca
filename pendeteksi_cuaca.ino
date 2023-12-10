// for Arduino Microcontroller
//#define rainAnalog A1
//#define rainDigital 2

// for ESP8266 Microcontroller
//#define rainAnalog A0
//#define rainDigital D1

#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MQ135.h>

#define SEALEVELPRESSURE_HPA (1013.25)

// for ESP32 Microcontroller
#define rainAnalog 35
//#define rainDigital 34
#define mqAnalog 34
//const int sm = 4;

#define WIFI_SSID "Galaxy A32"
#define WIFI_PASSWORD "sdnk5012"

// Raspberry Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(192, 168, 18, 54)
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883

// Temperature MQTT Topic
#define MQTT_PUB_TEMP "esp32/bme280/temperature"
#define MQTT_PUB_TEMP_2 "esp32/bme280/humidity"
#define MQTT_PUB_TEMP_3 "esp32/bme280/pressure"
#define MQTT_PUB_TEMP_4 "esp32/mq135/ppm"
#define MQTT_PUB_TEMP_5 "esp32/dsk-mhrd/rain"

Adafruit_BME280 bme; // I2C
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

MQ135 mq135_sensor(mqAnalog);

// float temperature = 35.0; // Assume current temperature. Recommended to measure with DHT22
// float humidity =  35.0;// Assume current humidity. Recommended to measure with DHT22

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time sensor was published
const long interval = 10000;        // Interval at which to publish sensor readings

unsigned long delayTime;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

/*void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}
void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}*/

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
    Serial.begin(9600);
    while(!Serial);    // time to get serial running
    Serial.println(F("BME280 test"));

    unsigned status;
    
    // default settings
    status = bme.begin(0x76);  
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }

  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();
  pinMode(rainAnalog,INPUT);
  // pinMode(mqDigital,INPUT);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  mqttClient.setCredentials("thomson12", "imoetica");
  connectToWifi();
    
    Serial.println("-- Default Test --");
    delayTime = 1000;

    Serial.println();
}


void loop() {
    unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;

    sensors_event_t temp_event, pressure_event, humidity_event;
    bme_temp->getEvent(&temp_event);
    bme_pressure->getEvent(&pressure_event);
    bme_humidity->getEvent(&humidity_event);
    float rainAnalogVal = analogRead(rainAnalog);
    float rainFloat = rainAnalogVal/4095;
    int rain = 100 - (rainFloat * 100);
    //int rainDigitalVal = digitalRead(rainDigital);
    float PPM = mq135_sensor.getPPM();
    //float correctedPPM = mq135_sensor.getCorrectedPPM(temperature, humidity);
    
    // Publish an MQTT message on topic esp32/ds18b20/temperature
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temp_event.temperature).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_TEMP);
    Serial.println(packetIdPub1);
    Serial.printf("Message: %.2f /n", temp_event.temperature);

    // Publish an MQTT message on topic esp32/ds18b20/humidity
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_TEMP_2, 1, true, String(humidity_event.relative_humidity).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 2, packetId: ", MQTT_PUB_TEMP_2);
    Serial.println(packetIdPub2);
    Serial.printf("Message: %.2f /n", humidity_event.relative_humidity);

    // Publish an MQTT message on topic esp32/ds18b20/pressure
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_TEMP_3, 1, true, String(pressure_event.pressure).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 3, packetId: ", MQTT_PUB_TEMP_3);
    Serial.println(packetIdPub3);
    Serial.printf("Message: %.2f /n", pressure_event.pressure);

    // Publish an MQTT message on topic esp32/mq135/ppm
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_TEMP_4, 1, true, String(PPM).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 4, packetId: ", MQTT_PUB_TEMP_4);
    Serial.println(packetIdPub4);
    Serial.printf("Message: %.2f /n", PPM);

    // Publish an MQTT message on topic esp32/mq135/ppm
    uint16_t packetIdPub5 = mqttClient.publish(MQTT_PUB_TEMP_5, 1, true, String(rain).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 5, packetId: ", MQTT_PUB_TEMP_5);
    Serial.println(packetIdPub5);
    Serial.printf("Message: %d /n", rain);
  }
     
    //printValues();
    //delay(delayTime);
}
