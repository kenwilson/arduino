#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFiClientSecure.h>
#include <MQTT.h>
#include <FS.h>

#define D3 3
#define BUCKETPIN         D3         // Tipping Bucket Input
#define BUCKETSIZE        0.2
#define BUCKETDEBOUNCEMS  25
#define SEALEVELPRESSURE_HPA (1013.25)

// I2C is D1 and D2

Adafruit_BME280 bme;
WiFiClientSecure net;
MQTTClient client;

// Configuration data
String wifi_ssid = "xxxx";
String wifi_passphrase = "xxxx";
String mqtt_server = "xxxx";
String mqtt_username = "esp_inside";
String mqtt_password = "ninjaninja";
String mqtt_id = "kens_desk";
uint16_t mqtt_port = 8883;
int update_interval = 10;


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
  // Initialise Serial
  Serial.begin(9600);
  // Initialize the weather senitysor.
  if (!bme.begin()) {
    Serial.println("Couldn't find BME280");
    return;
  }
#if 0
  /* Initialise the SPI FFS, and load our config file */
  if (!SPIFFS.begin()) {
    Serial.println("Couldn't load SPIFFS");
    return;
  }

  /* Open the configuration file */
  File conf = SPIFFS.open("/weather.ini", "r");
  if (!conf) {
    Serial.println("Couldn't load config");
    return;
  }

  /* Parse the file */
  while (true) {
    String key;
    /* Format is "key value\n" */
    key = conf.readStringUntil(' ');
    if (key == "ssid") {
      wifi_ssid = conf.readStringUntil('\n');
    } else if (key == "passphrase") {
      wifi_passphrase = conf.readStringUntil('\n');
    } else if (key == "mqtt_server") {
      mqtt_server = conf.readStringUntil('\n');
    } else if (key ==  "mqtt_username") {
      mqtt_username = conf.readStringUntil('\n');
    } else if (key == "mqtt_password") {
      mqtt_password = conf.readStringUntil('\n');
    } else if (key == "mqtt_id") {
      mqtt_id = conf.readStringUntil('\n');
    } else if (key == "mqtt_port") {
      mqtt_port = conf.parseInt();
      conf.readStringUntil('\n');
    } else if (key == "update_interval") {
      update_interval = conf.parseInt();
      conf.readStringUntil('\n');
    } else {
      break;
    }
  }
#endif
  Serial.print("Read configuration - connecting to Network: ");
  Serial.print(wifi_ssid);
  Serial.print(" with passphrase: ");
  Serial.println(wifi_passphrase);

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

  Serial.println("Connecting to MQTT Server: " + mqtt_server + ":" + mqtt_port + " with credentials: " + mqtt_username + ":" + mqtt_password);
  Serial.println("Updates every " + String(update_interval) + " seconds");

  client.begin(mqtt_server.c_str(), mqtt_port, net);
  client.onMessage(mqtt_msg);
  while (!client.connect(mqtt_id.c_str(), mqtt_username.c_str(), mqtt_password.c_str())) {
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
