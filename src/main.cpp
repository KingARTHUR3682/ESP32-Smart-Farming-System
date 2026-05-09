#include <Arduino.h>
#include "DHT.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <BH1750.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "secrets.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 // Reset pin

#define DHTPIN 4
#define DHTTYPE 11
#define FANPIN 25

#define SOILPIN 32
#define PUMPPIN 26

#define LIGHTPIN 27

#define TRIGPIN 18
#define ECHOPIN 19

DHT dht(DHTPIN, DHTTYPE); // Assign dht sensor
BH1750 gy302; // Assign light sensor
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Fixed sensors value
const float tankEmptyDistance = 16.0;
const float tankFullDistance = 5.0;

float lastValidWaterLevel = 100.0;

const int soilDryValue = 3165;
const int soilWetValue = 1050;

// Output states
bool fanState = false;
bool pumpState = false;
bool lightState = false;
bool tankEmpty = false;

bool manualMode = false;

unsigned long pumpStartTime = 0;
const unsigned long pumpRunTimeLimit = 10000; // 10 second
struct SensorSetting{
  float triggerOn;
  float space;
};

// Assign dynamic variables (Dafaults)
SensorSetting temp = {30.0, 2.0};
SensorSetting hum = {75.0, 5.0};
SensorSetting moi = {30.0, 5.0};
SensorSetting lux = {500.0, 200.0};

float triggerTankEmpty = 15.0;


// WiFi settings
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;

WiFiClientSecure espClient; // Assign WiFi connection worker
// WiFiClient espClient;
PubSubClient client(espClient); // Assign MQTT worker and allow it to access WiFi

// MQTT broker setting (Able to change url and port later)
const char* mqtt_server = MQTT_BROKER_URL;
const int mqtt_port = MQTT_BROKER_PORT;

// MQTT Topics
const char* topic_sensors_data = "sfs/node_1/sensors/data"; // Topic to send data to web server
const char* topic_sensors_settings = "sfs/node_1/sensors/settings"; // Topic to get settings from web server
const char* topic_settings_status = "sfs/node_1/sensors/status"; // Topic to send currenty setting value and status

// Setting for wait time
struct Timer {
  unsigned long previous;
  unsigned long interval;
};

Timer sensorTimer = {0, 30000};
Timer wifiTimer = {0, 10000};
Timer mqttTimer = {0, 5000};
Timer displayTimer = {0, 2000};

// ---------------------------------------------------------------------------------------------------

float getWaterLevel() {
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);

  long duration = pulseIn(ECHOPIN, HIGH, 30000);

  if (duration == 0) {
    return lastValidWaterLevel;
  }

  float distanceCm = duration * 0.034 / 2;

  if (distanceCm > 400) {
    return lastValidWaterLevel; 
  }

  int levelPercent = map(distanceCm, tankEmptyDistance, tankFullDistance, 0, 100);
  lastValidWaterLevel = constrain(levelPercent, 0, 100);

  return lastValidWaterLevel;
}

float getSoilMoisture() {
  int rawMoisture = analogRead(SOILPIN);
  
  // Map and constrain the value
  int moisturePercent = map(rawMoisture, soilDryValue, soilWetValue, 0, 100);
  return constrain(moisturePercent, 0, 100);
}

// Function to turn payload to string message from web server and update the dynamic variables
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println(F("\n--- New Settings Received ---")); 

  // Parse the JSON
  JsonDocument doc; // Assign a memory named doc for JSON message
  DeserializationError error = deserializeJson(doc, payload, length); // Unpack and organize the message to doc and report error

  if(error) {
    Serial.print(F("JSON Parse failed: "));
    Serial.println(error.f_str()); // print error message using flash memory in string
    return;
  }

  // Update the dynamic variables if the JSON contains them
  if (doc["temp_on"].is<float>()) temp.triggerOn = doc["temp_on"];
  if (doc["temp_space"].is<float>()) temp.space = doc["temp_space"];
  if (doc["hum_on"].is<float>()) hum.triggerOn = doc["hum_on"];
  if (doc["hum_space"].is<float>()) hum.space = doc["hum_space"];
  if (doc["moi_on"].is<float>()) moi.triggerOn = doc["moi_on"];
  if (doc["moi_space"].is<float>()) moi.space = doc["moi_space"];
  if (doc["light_on"].is<float>()) lux.triggerOn = doc["light_on"];
  if (doc["light_space"].is<float>()) lux.space = doc["light_space"];

  if (doc["manual_mode"].is<bool>()) manualMode = doc["manual_mode"];

  if(manualMode == true) {
    // maunual mode read trigger

    // Manual Fan Control
    if(doc["manual_fan"].is<bool>()) {
      fanState = doc["manual_fan"];
      digitalWrite(FANPIN, fanState ? LOW : HIGH); // active low
      Serial.println(fanState ? F("Manual Command: Fan ON") : F("Manual Command: Fan OFF"));
    }

    // Manual Light Control
    if (doc["manual_light"].is<bool>()) {
      lightState = doc["manual_light"];
      digitalWrite(LIGHTPIN, lightState ? LOW : HIGH); // Active-low relay
      Serial.println(lightState ? F("Manual Command: Light ON") : F("Manual Command: Light OFF"));
    }

    // Manual Pump Control
    if (doc["manual_pump"].is<bool>()) {
      bool requestedPumpState = doc["manual_pump"];
      
      // The manual override safety check!
      if (requestedPumpState == true && tankEmpty == true) {
         Serial.println(F("Manual Override FAILED: Tank is empty! Cannot turn on pump."));
      } else {
         pumpState = requestedPumpState;
         digitalWrite(PUMPPIN, pumpState ? LOW : HIGH); // Active-low relay
         Serial.println(pumpState ? F("Manual Command: Pump ON") : F("Manual Command: Pump OFF"));
      }
    }
  }

  Serial.println(F("Variables updated successfully!"));
}

// Non-blocking MQTT Reconnect
boolean MQTTReconnect() {
  // Assign a client ID per connection for MQTT broker
  Serial.print(F("Attempting MQTT connection..."));

  char clientId[20]; // Create a buffer
  // %04X means "Print as a minimum 4-digit uppercase Hexadecimal" 
  snprintf(clientId, sizeof(clientId), "SFS-Node1-%04X", random(0xffff));
  
  if (client.connect(clientId, MQTT_USER, MQTT_PASS)) { // Username and password for MQTT broker
    Serial.println(F("connected"));
    // Subscribe to the settings topic to receive updates
    client.subscribe(topic_sensors_settings);
    return true;
  } else {
    Serial.print(F("failed, Return Code: ")); // Print return code
    Serial.println(client.state());
    return false;
  }
}

void fan(float humidity, float temperature) {
  if(manualMode == false) {
    // Calculate dynamic off point
    float triggerTempOff = temp.triggerOn - temp.space;
    float triggerHumOff = hum.triggerOn - hum.space;

    // On logic (Fan will be on when one of the value is true)
    if(!fanState && (temperature >= temp.triggerOn || humidity >= hum.triggerOn)) {
      digitalWrite(FANPIN, LOW); // Turn on fan
      fanState = true; // Update fan's state
      Serial.println(F("Fan opened"));

    //Off logic (Using AND in if() to ensure the fan will be off when all the values are low)
    } else if(fanState && (temperature <= triggerTempOff && humidity <= triggerHumOff)) {
      digitalWrite(FANPIN, HIGH); //Turn off fan
      fanState = false; // Update fan's state
      Serial.println(F("Fan Closed"));
    }
  }
}

void waterPump(float moisture, float tankLevel) {
  // Safety kill if water tank is low
  if (tankLevel <= triggerTankEmpty) {
    digitalWrite(PUMPPIN, HIGH); // FORCE THE PUMP OFF!
    pumpState = false;
    tankEmpty = true; // Trigger the alert
    Serial.println(F("WARNING: Tank is empty! Pump disabled to prevent damage."));
    return;
  } else {
    tankEmpty = false; // Tank is safe, clear the alert
  }

  if(manualMode == false) {
    // Calculate dynamic off point
    float triggerMoiOff = moi.triggerOn + moi.space;

    // On logic
    if(!pumpState && moisture <= moi.triggerOn) {
      digitalWrite(PUMPPIN, LOW); // Turn on water pump
      pumpState = true; // Update state
      pumpStartTime = millis(); // Record water pump start time
      Serial.println(F("Water pump opened"));

    //Off logic
    } else if(pumpState && moisture >= triggerMoiOff) {
      digitalWrite(PUMPPIN, HIGH); // Turn off water pump
      pumpState = false; // Update state
      Serial.println(F("Water pump closed"));
    }
  }
}

void growLight(float light) {
  if(manualMode == false) {
    float triggerLightOff = lux.triggerOn + lux.space;
    // On logic
    if(!lightState && light <= lux.triggerOn) {
      digitalWrite(LIGHTPIN, LOW); // Turn on grow light
      lightState = true; // Update state
      Serial.println(F("Grow light opened"));
    } else if(lightState && light >= triggerLightOff) {
      digitalWrite(LIGHTPIN, HIGH); // Turn off grow light
      lightState = false; // Update state
      Serial.println(F("Grow light closed"));
    }
  }
}

void updateDisplay(float t, float h, float m, float wl, float l) {
  display.clearDisplay(); // Wipe the screen clean before drawing
  display.ssd1306_command(0xD3);
  display.ssd1306_command(0x00);
  display.ssd1306_command(0x40);
  display.setTextSize(1); // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text

  // --- ROW 1: Temp & Humidity ---
  display.setCursor(0, 0); // (X, Y) coordinates
  display.print(F("Temp: ")); display.print(t, 1); display.print(F("C"));
  display.setCursor(70, 0); // Shift right for humidity
  display.print(F("Hum: ")); display.print(h, 0); display.print(F("%"));

  // --- ROW 2: Soil & Tank Level ---
  display.setCursor(0, 16);
  display.print(F("Soil: ")); display.print(m, 0); display.print(F("%"));
  display.setCursor(70, 16);
  display.print(F("Tnk: ")); display.print(wl, 0); display.print(F("%"));

  // --- ROW 3: Light ---
  display.setCursor(0, 32);
  display.print(F("Lux:  ")); display.print(l, 0);

  // --- ROW 4: Relay Status (F=Fan, P=Pump, L=Light) ---
  display.setCursor(0, 50);
  display.print(F("F:")); display.print(fanState ? "ON " : "OFF");
  display.print(F(" P:")); display.print(pumpState ? "ON " : "OFF");
  display.print(F(" L:")); display.print(lightState ? "ON" : "OFF");

  display.display(); // Actually push all to the physical screen!
}

void setup() {
  Serial.begin(115200);

  pinMode(FANPIN, OUTPUT);
  pinMode(PUMPPIN, OUTPUT);
  pinMode(LIGHTPIN, OUTPUT);
  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);
  digitalWrite(FANPIN, HIGH); //Active-low on relay module
  digitalWrite(PUMPPIN, HIGH);
  digitalWrite(LIGHTPIN, HIGH);
  
  

  dht.begin();
  Wire.begin();
  if (gy302.begin()) {
    Serial.println(F("BH1750 Light Sensor initialized"));
  } else {
    Serial.println(F("Error initializing BH1750"));
  }

  //Setup Screen
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
  } else {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 25);
    display.setTextSize(2); // Big text for the boot screen
    display.print(F("Booting.."));
    display.display();
    Serial.println(F("OLED initialized"));
  }

  // Connect to Wi-Fi
  Serial.print(F("Connecting to WiFi..."));
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println(F(" Connected!"));

  espClient.setInsecure();

  // Setup MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setBufferSize(512);
  client.setCallback(callback);
}

void loop() {
  // Using millis rather than delay to ensure non-blocking on ESP32
  unsigned long currentMillis = millis();


  if (manualMode == false) {
    sensorTimer.interval = 10000;
    if (pumpState && (currentMillis - pumpStartTime >= pumpRunTimeLimit)) {
      digitalWrite(PUMPPIN, HIGH);
      pumpState = false;
      Serial.println(F("Safety Timeout: Pump stopped after 10 seconds."));
    }
  } else {
    sensorTimer.interval = 5000; // Sensor timer set to 5s in manual mode
  }

  // Read every 2 second for display update
  if (currentMillis - displayTimer.previous >= displayTimer.interval) {
    displayTimer.previous = currentMillis;
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    float m = getSoilMoisture();
    float l = gy302.readLightLevel();
    float wl = getWaterLevel();
    updateDisplay(t, h, m, wl, l);
  }
  
  if (WiFi.status() != WL_CONNECTED) { // Check connect status for WiFi
    if (currentMillis - wifiTimer.previous > wifiTimer.interval) {
      wifiTimer.previous = currentMillis;
      Serial.println(F("Wi-Fi lost. Reconnecting..."));
      WiFi.disconnect();
      WiFi.reconnect();
    }
  } else if (!client.connected()) { // Check connect status for MQTT
    if (currentMillis - mqttTimer.previous > mqttTimer.interval) {
      mqttTimer.previous = currentMillis;
      MQTTReconnect();
    }
  } else {
    client.loop(); // Process incoming message (Call callback function)
  }
  
  // Read time
  if(currentMillis - sensorTimer.previous >= sensorTimer.interval) {
    sensorTimer.previous  = currentMillis;
    
    // Assign value from sensor
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    float m = getSoilMoisture();
    float l = gy302.readLightLevel();
    float wl = getWaterLevel();

    if(isnan(h) || isnan(t)) {
      Serial.println(F("Failed to read from DHT sensor! Auto Fan logic skipped."));
      // Only in Auto Mode will a sensor error force the fan to shut down.
      // If in Manual Mode, sensor errors are not allowed to interfere with the fan status.
      if (manualMode == false) { 
        digitalWrite(FANPIN, HIGH);
        fanState = false;
      }
    } else {
      fan(h, t); 
    }

    waterPump(m, wl);
    growLight(l);

    Serial.print(F("Temp: "));
    Serial.print(t);
    Serial.print(F("C | Hum: "));
    Serial.print(h);
    Serial.print(F("% | Moi: "));
    Serial.print(m);
    Serial.print(F("% | Tank: "));
    Serial.print(wl);
    Serial.print(F("% | Lux: "));
    Serial.println(l);

    updateDisplay(t, h, m, wl, l);

    if (client.connected()) {
      JsonDocument doc; // Assign memory space for JSON named doc
      // Assign value into JSON variable
      doc["temperature"] = t;
      doc["humidity"] = h;
      doc["fan_status"] = fanState;
      doc["moisture"] = m;
      doc["water_pump_status"] = pumpState;
      doc["light_lux"] = l;
      doc["grow_light_state"] = lightState;
      doc["tank_level"] = wl;
      doc["tank_empty_alert"] = tankEmpty;

      char jsonBuffer[512]; // Space for JSON message
      serializeJson(doc, jsonBuffer); // Pack the message into line format
      
      client.publish(topic_sensors_data, jsonBuffer); // Publish message to MQTT
      Serial.print(F("Published Data: "));
      Serial.println(jsonBuffer); // Print the message in line format
    }

    if (client.connected()) {
      JsonDocument doc; // Assign memory space for JSON named doc
      // Assign value into JSON variable
      doc["manual_mode"] = manualMode;

      
      doc["temp_on"] = temp.triggerOn;
      doc["temp_space"] = temp.space;
      doc["hum_on"] = hum.triggerOn;
      doc["hum_space"] = hum.space;
      doc["moi_on"] = moi.triggerOn;
      doc["moi_space"] = moi.space;
      doc["light_on"] = lux.triggerOn;
      doc["light_space"] = lux.space;

      
      doc["manual_fan"] = fanState;
      doc["manual_pump"] = pumpState;
      doc["manual_light"] = lightState;

      char jsonBuffer[512]; // Space for JSON message
      serializeJson(doc, jsonBuffer); // Pack the message into line format
      
      client.publish(topic_settings_status, jsonBuffer); // Publish message to MQTT
      Serial.print(F("Published Setting Status: "));
      Serial.println(jsonBuffer); // Print the message in line format
    }
  }  
}