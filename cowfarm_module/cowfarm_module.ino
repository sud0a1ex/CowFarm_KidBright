#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include "DHT.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

// Wi-Fi Access Point Credentials
const char* ssid = "CowFarm_Robot";
const char* password = "smt1029384756";

// Web Server & WebSocket setup
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Hardware Pin & Address Definitions
#define MCP_ADDR    0x20
#define BUTTON_S1   16  // KidBright 32iP S1 button
#define MQ135_PIN   34  // KidBright 32iP IN3 terminal (GPIO 34)
#define DHTPIN      23  // DHT22 connected to IO23
#define DHTTYPE     DHT22
#define TRIG_PIN    18  // HC-SR04 Trig on IO18
#define ECHO_PIN    19  // HC-SR04 Echo on IO19
#define PUMP_OUT1   26  // Mini Pump on OUT1 terminal (GPIO 26)

#define NH3_THRESHOLD_PPM 25.0 // Ammonia threshold to activate pump

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
DHT dht(DHTPIN, DHTTYPE);

bool isRobotOn = false;          // Toggle state for robot movement
bool lastButtonState = HIGH;     // Button state tracking
unsigned long lastSensorRead = 0;// Telemetry update timer
unsigned long lastDHTRead = 0;   // DHT22 timer
unsigned long lastSonarRead = 0; // Sonar timer while driving

// Cached sensor & calibration values
float r0 = 76.63;
float dhtTemp = 0.0;
float dhtHumidity = 0.0;
float sonarDistanceCM = -1.0;
float ammoniaPPM = 0.0;
float objectC = 0.0;

// Embedded HTML/JS Dashboard
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Telemetry Dashboard</title>
  <style>
    body { font-family: Arial, sans-serif; background: #121212; color: #fff; text-align: center; margin:0; padding:20px; }
    h1 { color: #00E676; margin-bottom: 20px; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 15px; max-width: 800px; margin: auto; }
    .card { background: #1E1E1E; padding: 20px; border-radius: 12px; box-shadow: 0 4px 10px rgba(0,0,0,0.5); }
    .val { font-size: 1.8em; font-weight: bold; margin-top: 10px; color: #4FC3F7; }
    .status-on { color: #00E676; font-weight: bold; }
    .status-off { color: #FF5252; font-weight: bold; }
  </style>
</head>
<body>
  <h1>KidBright Robot Dashboard</h1>
  <div class="grid">
    <div class="card"><div>Robot State</div><div id="robot" class="val status-off">STOPPED</div></div>
    <div class="card"><div>Distance</div><div id="sonar" class="val">-- cm</div></div>
    <div class="card"><div>Ammonia (NH3)</div><div id="nh3" class="val">-- PPM</div></div>
    <div class="card"><div>Mini Pump</div><div id="pump" class="val status-off">OFF</div></div>
    <div class="card"><div>IR Temp</div><div id="irtemp" class="val">-- &deg;C</div></div>
    <div class="card"><div>Air Temp</div><div id="airtemp" class="val">-- &deg;C</div></div>
    <div class="card"><div>Humidity</div><div id="humidity" class="val">-- %</div></div>
  </div>

  <script>
    var gateway = `ws://${window.location.hostname}/ws`;
    var websocket;

    function initWebSocket() {
      websocket = new WebSocket(gateway);
      websocket.onmessage = onMessage;
      websocket.onclose = function() { setTimeout(initWebSocket, 2000); };
    }

    function onMessage(event) {
      var data = JSON.parse(event.data);
      document.getElementById('sonar').innerHTML = data.sonar > 0 ? data.sonar.toFixed(1) + " cm" : "Out of Range";
      document.getElementById('nh3').innerHTML = data.nh3.toFixed(1) + " PPM";
      document.getElementById('irtemp').innerHTML = data.irTemp.toFixed(1) + " &deg;C";
      document.getElementById('airtemp').innerHTML = data.airTemp.toFixed(1) + " &deg;C";
      document.getElementById('humidity').innerHTML = data.humidity.toFixed(1) + " %";
      
      var pumpEl = document.getElementById('pump');
      pumpEl.innerHTML = data.pump ? "ON" : "OFF";
      pumpEl.className = data.pump ? "val status-on" : "val status-off";

      var robotEl = document.getElementById('robot');
      robotEl.innerHTML = data.robot ? "RUNNING" : "STOPPED";
      robotEl.className = data.robot ? "val status-on" : "val status-off";
    }

    window.addEventListener('load', initWebSocket);
  </script>
</body>
</html>
)rawliteral";

// Broadcasts JSON sensor data to all connected web clients
void notifyClients() {
  String json = "{";
  json += "\"sonar\":" + String(sonarDistanceCM) + ",";
  json += "\"nh3\":" + String(ammoniaPPM) + ",";
  json += "\"irTemp\":" + String(objectC) + ",";
  json += "\"airTemp\":" + String(dhtTemp) + ",";
  json += "\"humidity\":" + String(dhtHumidity) + ",";
  json += "\"pump\":" + String(digitalRead(PUMP_OUT1) == LOW ? "true" : "false") + ",";
  json += "\"robot\":" + String(isRobotOn ? "true" : "false");
  json += "}";
  ws.textAll(json);
}

// Measures sonar distance in centimeters
float getSonarDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); 
  if (duration == 0) return -1.0;
  return (duration * 0.0343) / 2.0;
}

void autoZeroMQ135() {
  Serial.println("Auto-zeroing MQ135... Keep ambient air steady for 3s.");
  float totalRs = 0;
  int samples = 30;

  for (int i = 0; i < samples; i++) {
    int rawValue = analogRead(MQ135_PIN);
    float vOut = (rawValue / 4095.0) * 3.3;
    if (vOut < 0.05) vOut = 0.05;

    float rS = ((5.0 - vOut) / vOut) * 10.0;
    totalRs += rS;
    delay(100);
  }

  float avgRs = totalRs / samples;
  float targetRatio = pow(1.0 / 102.2, -1.0 / 2.473);
  r0 = avgRs / targetRatio;
}

float getMQ135PPM(int rawValue) {
  if (rawValue <= 0) return 0.0;
  float vOut = (rawValue / 4095.0) * 3.3;
  if (vOut <= 0.05) return 0.0;
  if (vOut >= 3.3)  vOut = 3.29;

  float rS = ((5.0 - vOut) / vOut) * 10.0;
  float ratio = rS / r0;
  return 102.2 * pow(ratio, -2.473);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  dht.begin();
  mlx.begin();

  // Pin configurations
  pinMode(BUTTON_S1, INPUT_PULLUP);
  pinMode(MQ135_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  pinMode(PUMP_OUT1, OUTPUT);
  digitalWrite(PUMP_OUT1, HIGH); // Pump starts OFF (Active-LOW)

  // Configure MCP23017 outputs
  Wire.beginTransmission(MCP_ADDR);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();

  stopRobot();
  autoZeroMQ135();

  // Start Wi-Fi Access Point
  WiFi.softAP(ssid, password);
  Serial.print("Wi-Fi AP Started. Dashboard IP: ");
  Serial.println(WiFi.softAPIP());

  // Serve static HTML dashboard & attach WebSocket handler
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  server.addHandler(&ws);
  server.begin();
}

void loop() {
  ws.cleanupClients();
  checkButton();

  if (isRobotOn) {
    moveForward();
    readSonarOnly();

    if (sonarDistanceCM > 0.0 && sonarDistanceCM < 25.0) {
      stopRobot();
      Serial.println("Obstacle detected (< 25cm)! Stopped motors.");

      unsigned long stopStartTime = millis();
      while (millis() - stopStartTime < 3000) {
        ws.cleanupClients();
        checkButton();
        readAllSensorsNonBlocking();
        if (!isRobotOn) {
          stopRobot();
          digitalWrite(PUMP_OUT1, HIGH);
          return;
        }
        delay(50);
      }

      if (ammoniaPPM > NH3_THRESHOLD_PPM) {
        Serial.println("Ammonia > 25 PPM! Activating pump...");
        digitalWrite(PUMP_OUT1, LOW); // Active-LOW

        while (isRobotOn && (ammoniaPPM > NH3_THRESHOLD_PPM)) {
          ws.cleanupClients();
          checkButton();
          readAllSensorsNonBlocking();
          delay(100);
        }

        digitalWrite(PUMP_OUT1, HIGH); // Deactivate pump
        Serial.println("Ammonia level clear (< 25 PPM). Pump stopped.");

        unsigned long postPumpTime = millis();
        while (millis() - postPumpTime < 2000) {
          ws.cleanupClients();
          checkButton();
          readAllSensorsNonBlocking();
          if (!isRobotOn) {
            stopRobot();
            digitalWrite(PUMP_OUT1, HIGH);
            return;
          }
          delay(50);
        }
      }
    }
  } else {
    stopRobot();
    digitalWrite(PUMP_OUT1, HIGH);
  }
}

void readSonarOnly() {
  if (millis() - lastSonarRead >= 100) {
    lastSonarRead = millis();
    sonarDistanceCM = getSonarDistanceCM();
    notifyClients();
  }
}

void readAllSensorsNonBlocking() {
  if (millis() - lastDHTRead >= 2000) {
    lastDHTRead = millis();
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(h) && !isnan(t)) {
      dhtHumidity = h;
      dhtTemp = t;
    }
  }

  if (millis() - lastSensorRead >= 500) {
    lastSensorRead = millis();
    objectC = mlx.readObjectTempC();
    ammoniaPPM = getMQ135PPM(analogRead(MQ135_PIN));
    sonarDistanceCM = getSonarDistanceCM();
    notifyClients();
  }
}

void checkButton() {
  bool currentButtonState = digitalRead(BUTTON_S1);
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    isRobotOn = !isRobotOn;
    notifyClients();
    delay(200);
  }
  lastButtonState = currentButtonState;
}

// Motor Routines via MCP23017
void moveForward()  { setMotors(HIGH, LOW, HIGH, LOW); }
void moveBackward() { setMotors(LOW, HIGH, LOW, HIGH); }
void turnLeft()     { setMotors(LOW, LOW, HIGH, LOW);  }
void turnRight()    { setMotors(HIGH, LOW, LOW, LOW);  }
void stopRobot()    { setMotors(LOW, LOW, LOW, LOW);   }

void setMotors(bool in1, bool in2, bool in3, bool in4) {
  byte value = 0;
  if (in1) value |= (1 << 0);
  if (in2) value |= (1 << 1);
  if (in3) value |= (1 << 2);
  if (in4) value |= (1 << 3);

  Wire.beginTransmission(MCP_ADDR);
  Wire.write(0x12);
  Wire.write(value);
  Wire.endTransmission();
}