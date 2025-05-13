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
const char* mqtt_topic_sub = "janggelin/relay-control";

WiFiClient espClient;
PubSubClient client(espClient);

// ========== KONFIGURASI DHT22 ==========
#define DHTPIN 5
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// ========== KONFIGURASI RELAY ==========
const int relayPins[4] = {13, 14, 27, 32};
bool relayStates[4] = {false, false, false, false};

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  dht.begin();

  for (int i = 0; i < 4; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], HIGH); // relay OFF (aktif LOW)
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
    publishSensorData();
  }
}

// ========== KONEKSI WIFI ==========
void setupWiFi() {
  Serial.print("Menghubungkan ke WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi terhubung. IP: " + WiFi.localIP().toString());
}

// ========== KONEKSI MQTT ==========
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Menghubungkan ke MQTT...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("Berhasil!");
      client.subscribe(mqtt_topic_sub);
    } else {
      Serial.print("Gagal (rc=");
      Serial.print(client.state());
      Serial.println("), coba lagi dalam 5 detik...");
      delay(5000);
    }
  }
}

// ========== KIRIM DATA SENSOR ==========
void publishSensorData() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Gagal membaca sensor DHT!");
    return;
  }

  StaticJsonDocument<128> doc;
  doc["suhu"] = t;
  doc["kelembapan"] = h;

  String payload;
  serializeJson(doc, payload);

  Serial.println("Publishing: " + payload);
  if (client.publish(mqtt_topic_pub, payload.c_str(), true)) {
    Serial.println("Publish berhasil.");
  } else {
    Serial.println("Publish gagal.");
  }
}

// ========== TERIMA PERINTAH RELAY ==========
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Pesan diterima [");
  Serial.print(topic);
  Serial.print("]: ");

  payload[length] = '\0'; // pastikan string diakhiri null
  String message = String((char*)payload);
  Serial.println(message);

  StaticJsonDocument<100> doc;
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    Serial.println("JSON Parsing Error: ");
    Serial.println(error.c_str());
    return;
  }

  int relayNum = doc["relay"];
  String state = doc["state"];

  if (relayNum >= 1 && relayNum <= 4 && (state == "ON" || state == "OFF")) {
    int index = relayNum - 1;
    digitalWrite(relayPins[index], (state == "ON") ? LOW : HIGH);
    relayStates[index] = (state == "ON");
    Serial.println("Relay " + String(relayNum) + " diatur ke " + state);
  } else {
    Serial.println("Pesan tidak valid atau relay di luar batas");
  }
}
