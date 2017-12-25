

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

#define BUCKETPIN         D3         // Tipping Bucket Input
#define BUCKETSIZE        0.2
#define BUCKETDEBOUNCEMS  25
#define SEALEVELPRESSURE_HPA (1013.25)

// I2C is D1 and D2

Adafruit_BME280 bme;

uint32_t delayMS;

const char *ssid = "kenz0r";
const char *passphrase = "ninjaninja";
float temp = 0.0f;
float humidity = 0.0f;
float pressure = 0.0f;
float rainfall = 0.0f;
unsigned long last_reading = 0;
const long interval = 2000;
volatile long tip_events = 0;
volatile unsigned long contact_time = 0;
ESP8266WebServer server(80);


// Tip event - increment counter
void rainfall_event() {
  if ((millis() - contact_time) > BUCKETDEBOUNCEMS) {
     tip_events++;
     rainfall = tip_events * BUCKETSIZE;
     contact_time = millis();
  }
}

// Root endpoint - just a placeholder for now.
void handle_root() {
  server.send(200, "text/plain", "weatherstation bro!!");
  delay(100);
}

// Fetch a reading - only recalculate every 2 seconds
// Handle invalid reads by keeping the last value
void update_reading() {
  unsigned long currentMillis = millis();
  if ((currentMillis - last_reading) >= interval) {
    last_reading = currentMillis;
    float reading = bme.readTemperature();
    if (!isnan(reading))
      temp = reading;
    reading = bme.readHumidity();
    if (!isnan(reading))
      humidity = reading;
    reading = bme.readPressure() / 100.0F;
    if (!isnan(reading))
      pressure = reading;
  }
}

// Temperature endpoint
void handle_temp() {
  update_reading();
  server.send(200, "text/plain", String(temp) + String(" C"));
  delay(100);
}

// Humidity endpoint
void handle_humidity() {
  update_reading();
  server.send(200, "text/plain", String(humidity) + String(" %"));
  delay(100);
}

// Pressure endpoint
void handle_pressure() {
  update_reading();
  server.send(200, "text/plain", String(pressure) + String(" hPA"));
  delay(100);
}

// Rainfall endpoint - just bucket tip events for now
void handle_rainfall() {
  server.send(200, "text/plain", String(rainfall) + String(" mm"));
}

// System UptimereadHumidity
void handle_uptime() {
  server.send(200, "text/plain", String(millis()) + String(" ms"));
}

void setup() {
  Serial.begin(9600);
  // Initialize device.
  if (!bme.begin()) {
    Serial.println("Couldn't find BME280");
  } 
  WiFi.begin(ssid, passphrase);
  Serial.print("\r\nConnecting to network");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("BME Weather Reading Server");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  temp = 0.0f;
  humidity = 0.0f;
  pressure = 0.0f;
  pinMode(BUCKETPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUCKETPIN), rainfall_event, FALLING);
  server.on("/", handle_root);
  server.on("/temp", handle_temp);
  server.on("/humidity", handle_humidity);
  server.on("/rainfall", handle_rainfall);
  server.on("/uptime", handle_uptime);
  server.on("/pressure", handle_pressure);

  server.begin();
}

void loop() {
  server.handleClient();
}
