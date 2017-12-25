

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

#define DHTPIN            D2         // Pin which is connected to the DHT sensor.
#define BUCKETPIN         D1         // Tipping Bucket Input

// Uncomment the type of sensor in use:
//#define DHTTYPE           DHT11     // DHT 11 
#define DHTTYPE           DHT22     // DHT 22 (AM2302)
//#define DHTTYPE           DHT21     // DHT 21 (AM2301)


DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

const char *ssid = "kenz0r";
const char *passphrase = "ninjaninja";
float temp, humidity;
unsigned long last_reading = 0;
const long interval = 2000; 
volatile long tip_events = 0;
ESP8266WebServer server(80);


// Tip event - increment counter
void rainfall_event() {
  tip_events++;  
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
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (!isnan(event.temperature))
      temp = event.temperature;
    dht.humidity().getEvent(&event);
    if (!isnan(event.relative_humidity))
      humidity = event.relative_humidity;
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

// Rainfall endpoint - just bucket tip events for now
void handle_rainfall() {
  server.send(200, "text/plain", String(tip_events) + String(" Events"));
}

// System Uptime
void handle_uptime() {
  server.send(200, "text/plain", String(millis()) + String(" ms"));
}

void setup() {
  Serial.begin(9600); 
  // Initialize device.
  dht.begin();
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");  
  Serial.println("------------------------------------");
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");  
  Serial.println("------------------------------------");
 
  WiFi.begin(ssid, passphrase);
  Serial.print("\r\nConnecting to network");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("DHT Weather Reading Server");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  temp = 0.0f;
  humidity = 0.0f;
  pinMode(BUCKETPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUCKETPIN), rainfall_event, FALLING);
  server.on("/", handle_root);
  server.on("/temp", handle_temp);
  server.on("/humidity", handle_humidity);
  server.on("/rainfall", handle_rainfall);
  server.on("/uptime", handle_uptime);

  server.begin();
}

void loop() {
  server.handleClient();
}
