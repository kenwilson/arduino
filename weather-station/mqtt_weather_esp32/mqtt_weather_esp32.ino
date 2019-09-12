#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFiClientSecure.h>
#include <MQTT.h>
#include <FS.h>
#include <SPIFFS.h>
#include <SPIFFSIniFile.h>

#define BUCKETPIN         3         // Tipping Bucket Input
#define BUCKETSIZE        0.2
#define BUCKETDEBOUNCEMS  25
#define SEALEVELPRESSURE_HPA (1013.25)


Adafruit_BME280 bme;
WiFiClientSecure net;
MQTTClient client;

// Configuration data
String wifi_ssid;
String wifi_passphrase;
String mqtt_server;
String auth_method;
String mqtt_username;
String mqtt_password;
String cafile;
String certfile;
String certkey;

char cacert_buf[2048];
char cert_buf[2048];
char certkey_buf[2048];

String mqtt_id;
uint16_t mqtt_port;
int update_interval;


// State Data
float temp = 0.0f;
float humidity = 0.0f;
float pressure = 0.0f;
float rainfall = 0.0f;
unsigned long last_reading = 0;
const long interval = 2000;
volatile long tip_events = 0;
volatile unsigned long contact_time = 0;
unsigned long lastMillis = 0;


void rainfall_event();
void mqtt_msg(String &topic, String &payload);
void update_reading();
void setup();
void loop();


/* Interrupt for the Rainfall Tip */
// Tip event - increment counter
void rainfall_event() {
  if ((millis() - contact_time) > BUCKETDEBOUNCEMS) {
    tip_events++;
    rainfall = tip_events * BUCKETSIZE;
    Serial.print("Bucket!");
    contact_time = millis();
  }
}

// Helpers to make config reading less clunky
String getValue(SPIFFSIniFile &file, const char *section, const char *key) {
    String ret = "";
    int buffer_len = 80;
    char buffer[buffer_len];

    if (file.getValue(section, key, buffer, buffer_len))
      ret = String(buffer);

    return ret;
}

int getInt(SPIFFSIniFile &file, const char *section, const char *key) {
    String val = getValue(file, section, key);
    return val.toInt();
}

// message handler for incoming messages
void mqtt_msg(String &topic, String &payload) {
  Serial.println("Incoming: " + topic + " - " + payload);
}

// Fetch a reading - only recalculate every 2 seconds
// Handle invalid reads by keeping the last value
void update_reading() {
  unsigned long currentMillis = millis();
  if ((currentMillis - last_reading) >= interval) {
    last_reading = currentMillis;
    float reading = bme.readTemperature();
    Serial.print("Read temp: ");
    Serial.println(reading);
    if (!isnan(reading))
      temp = reading;
    reading = bme.readHumidity();
    Serial.print("Read humid: ");
    Serial.println(reading);
    if (!isnan(reading))
      humidity = reading;
    reading = bme.readPressure() / 100.0F;
    Serial.print("Read pressure: ");
    Serial.println(reading);
    if (!isnan(reading))
      pressure = reading;
  }
}

void setup() {
  const char *settings_file = "/settings.ini";

  // Initialise Serial
  Serial.begin(9600);
  // Initialize the weather senitysor.
  if (!bme.begin()) {
    Serial.println("Couldn't find BME280");
    return;
  }

  /* Initialise the SPI FFS, and load our config file */
  if (!SPIFFS.begin()) {
    Serial.println("Couldn't load SPIFFS");
    return;
  }

  SPIFFSIniFile ini(settings_file);

  if (!ini.open()) {
    Serial.println("Couldn't load settings");
  }

  wifi_ssid = getValue(ini, "wifi", "ssid");
  wifi_passphrase = getValue(ini, "wifi", "passphrase");
  mqtt_server = getValue(ini, "mqtt", "server");
  mqtt_port = getInt(ini, "mqtt", "port");
  mqtt_id = getValue(ini, "mqtt", "topic");
  update_interval = getInt(ini, "mqtt", "update_interval");
  Serial.println("checking for cert auth");
  auth_method = getValue(ini, "mqtt", "auth");
  if (auth_method == "cert") {
    Serial.println("doing cert auth");
    cafile = getValue(ini, "mqtt", "cafile");
    certfile = getValue(ini, "mqtt", "cert");
    certkey = getValue(ini, "mqtt", "cert_key");

    /* Load certificates into buffers */
    File file = SPIFFS.open(cafile, "r");
    file.readBytes(cacert_buf, 2048);
    file = SPIFFS.open(certfile, "r");
    file.readBytes(cert_buf, 2048);
    file = SPIFFS.open(certkey, "r");
    file.readBytes(certkey_buf, 2048);
    
  } else {
    mqtt_username = getValue(ini, "mqtt", "username");
    mqtt_password = getValue(ini, "mqtt", "password");
  }

  Serial.print("Read configuration - connecting to Network: ");
  Serial.print(wifi_ssid);

  WiFi.begin(wifi_ssid.c_str(), wifi_passphrase.c_str());
  Serial.print("\r\nConnecting to network");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("BME Weather Reading Server");
  Serial.print("Connected to ");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Initialising readings and tipping bucket");
  temp = 0.0f;
  humidity = 0.0f;
  pressure = 0.0f;
  pinMode(BUCKETPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUCKETPIN), rainfall_event, FALLING);

  Serial.println("Connecting to MQTT Server: " + mqtt_server + ":" + mqtt_port);
  Serial.println("Updates every " + String(update_interval) + " seconds");

  if (auth_method == "cert") {
    //net.setCACert(cacert_buf);
    Serial.println("Setting Client Cert");
    net.setCertificate(cert_buf);
    net.setPrivateKey(certkey_buf);
  }
  client.begin(mqtt_server.c_str(), mqtt_port, net);
  client.onMessage(mqtt_msg);
  bool connected = false;
  while (!connected) {
    if (auth_method == "cert") {
      connected = client.connect(mqtt_id.c_str());
    } else {
      connected = client.connect(mqtt_id.c_str(), mqtt_username.c_str(), mqtt_password.c_str());
    }
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nConnected");
  return;
}

void loop() {
  client.loop();
  delay(10);


  if (!client.connected()) {
    client.connect(mqtt_id.c_str(), mqtt_username.c_str(), mqtt_password.c_str());
  }

  // Publish an update
  if (millis() - lastMillis > (update_interval * 1000)) {
    Serial.println("Sending update");
    lastMillis = millis();
    update_reading();
    client.publish("/home/weather/" + mqtt_id + "/temp", String(temp));
    client.publish("/home/weather/" + mqtt_id + "/humidity", String(humidity));
    client.publish("/home/weather/" + mqtt_id + "/pressure", String(pressure));
    client.publish("/home/weather/" + mqtt_id + "/culm_rainfall", String(rainfall));
    Serial.println("Sent update");

  }

}
