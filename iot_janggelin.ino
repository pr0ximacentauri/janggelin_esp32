#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <ArduinoJson.h>

// ========== KONFIGURASI WIFI ==========
const char* ssid = "IQOOZ7";
const char* password = "inisemuasalahrenjana";

// ========== KONFIGURASI MQTT ==========
const char* mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;
const char* mqtt_topic_pub = "janggelin/sensor-dht22";
const char* mqtt_topic_pub_control = "janggelin/device-control";
const char* mqtt_topic_sub_limit = "janggelin/optimal-limit";

WiFiClient espClient;
PubSubClient client(espClient);

// ========== KONFIGURASI DHT22 ==========
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// ========== KONFIGURASI RELAY ==========
const int relayPins[4] = {13, 14, 27, 32};
bool relayStates[4] = {false, false, false, false};

// ========== BATAS OPTIMAL (default) ==========
float minTemp = 26.0;
float maxTemp = 30.0;
float minHumid = 70.0;
float maxHumid = 90.0;
bool limitReceived = false;

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  dht.begin();

  for (int i = 0; i < 4; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], HIGH);
  }

  setupWiFi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
}

// ========== LOOP ==========
void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  static unsigned long lastReadTime = 0;
  if (millis() - lastReadTime >= 5000) {
    lastReadTime = millis();
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t)) {
      Serial.println("Gagal membaca sensor DHT!");
      return;
    }

    publishSensorData(t, h);
    autoActuatorControl(t, h);
  }
}

// ========== WIFI ==========
void setupWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Menghubungkan ke WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi terhubung. IP: " + WiFi.localIP().toString());
}

// ========== MQTT ==========
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Menghubungkan ke MQTT...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("Berhasil!");
      client.subscribe(mqtt_topic_sub_limit); 
    } else {
      Serial.print("Gagal (rc=");
      Serial.print(client.state());
      Serial.println("), coba lagi dalam 5 detik...");
      delay(5000);
    }
  }
}

// ========== KIRIM DATA SENSOR ==========
void publishSensorData(float t, float h) {
  StaticJsonDocument<100> doc;
  doc["suhu"] = t;
  doc["kelembaban"] = h;

  char buffer[100];
  serializeJson(doc, buffer);
  client.publish(mqtt_topic_pub, buffer);
  Serial.println("Data terkirim: " + String(buffer));
}

// ========== KONTROL OTOMATIS BERDASARKAN BATAS ==========
void autoActuatorControl(float t, float h) {
  bool isTempOptimal = (t >= minTemp && t <= maxTemp);
  bool isHumidOptimal = (h >= minHumid && h <= maxHumid);

  // Relay 1 = pompa, Relay 2 = kipas, Relay 3 = humidifier
  if (isTempOptimal && isHumidOptimal) {
    setRelay(1, false);
    setRelay(2, false);
    // setRelay(3, false);
    return;
  }

  // if (!isTempOptimal && t <= minTemp) {
  //   setRelay(3, true);
  // } else {
  //   setRelay(3, false);
  // }

  if (!isTempOptimal && t >= maxTemp) {
    setRelay(2, true);
  } else if (isTempOptimal && h <= maxHumid) {
    setRelay(2, false);
  }

  if (!isHumidOptimal && h >= maxHumid) {
    setRelay(2, true);
  }

  if (!isHumidOptimal && h <= minHumid) {
    setRelay(1, true);
  } else {
    setRelay(1, false);
  }
}

// ========== ATUR RELAY ==========
void setRelay(int relayNum, bool turnOn) {
  int index = relayNum - 1;
  if (index < 0 || index >= 4) return;

  digitalWrite(relayPins[index], turnOn ? LOW : HIGH);
  relayStates[index] = turnOn;

  Serial.println("Relay " + String(relayNum) + " -> " + (turnOn ? "ON" : "OFF"));

  // Kirim status kontrol ke MQTT
  StaticJsonDocument<100> doc;
  doc["id_kontrol"] = relayNum;
  doc["status"] = turnOn ? "ON" : "OFF";

  char buffer[100];
  serializeJson(doc, buffer);
  client.publish(mqtt_topic_pub_control, buffer);
  Serial.println("Status kontrol dikirim: " + String(buffer));
}

// ========== CALLBACK MQTT ==========
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Pesan MQTT diterima [");
  Serial.print(topic);
  Serial.print("]: ");

  payload[length] = '\0';
  String message = String((char*)payload);
  Serial.println(message);

  // Topik batas optimal
  if (String(topic) == mqtt_topic_sub_limit) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
      Serial.println("Gagal parsing batas optimal: " + String(error.c_str()));
      return;
    }

    minTemp = doc["minTemperature"] | minTemp;
    maxTemp = doc["maxTemperature"] | maxTemp;
    minHumid = doc["minHumidity"] | minHumid;
    maxHumid = doc["maxHumidity"] | maxHumid;
    limitReceived = true;

    Serial.println("Batas optimal diterima:");
    Serial.printf("- Suhu: %.2f ~ %.2f\n", minTemp, maxTemp);
    Serial.printf("- Kelembapan: %.2f ~ %.2f\n", minHumid, maxHumid);
  }
}
