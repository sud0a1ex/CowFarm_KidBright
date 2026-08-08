#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include "DHT.h"

// Hardware Pin & Address Definitions
#define MCP_ADDR    0x20
#define BUTTON_S1   16  // KidBright 32iP S1 button
#define MQ135_PIN   34  // KidBright 32iP IN3 terminal (GPIO 34)
#define DHTPIN      23  // DHT22 connected to IO23
#define DHTTYPE     DHT22
#define TRIG_PIN    18  // HC-SR04 Trig on IO18
#define ECHO_PIN    19  // HC-SR04 Echo on IO19
#define PUMP_OUT1   26  // Mini Pump on OUT1 terminal (GPIO 26)

#define NH3_THRESHOLD_PPM 25.0 // Ammonia threshold to turn off pump

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
DHT dht(DHTPIN, DHTTYPE);

bool isRobotOn = false;          // Toggle state for robot movement
bool lastButtonState = HIGH;     // Button state tracking
unsigned long lastSensorRead = 0;// Telemetry update timer
unsigned long lastDHTRead = 0;   // DHT22 timer (2s required interval)

// Cached sensor & calibration values
float r0 = 76.63;
float dhtTemp = 0.0;
float dhtHumidity = 0.0;
float sonarDistanceCM = -1.0;

// Measures sonar distance in centimeters
float getSonarDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo pulse with a 30ms timeout (~500 cm max range)
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); 
  if (duration == 0) return -1.0; // Out of range / timeout

  // Speed of sound = 0.0343 cm/us (divided by 2 for round-trip)
  return (duration * 0.0343) / 2.0;
}

// Auto-zeros MQ135 baseline over 3 seconds (~1.0 PPM default)
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

  Serial.print("Auto-zero complete! Dynamic R0: ");
  Serial.print(r0);
  Serial.println(" kOhm");
}

// Calculates Ammonia (NH3) PPM
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

  if (!mlx.begin()) {
    Serial.println("WARNING: MLX90614 not found!");
  }

  // Pin configurations
  pinMode(BUTTON_S1, INPUT_PULLUP);
  pinMode(MQ135_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  pinMode(PUMP_OUT1, OUTPUT);

  // Set Port A on MCP23017 to outputs
  Wire.beginTransmission(MCP_ADDR);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();

  stopRobot();
  autoZeroMQ135();
}

void loop() {
  checkButton();
  readSensorsNonBlocking();

  if (isRobotOn) {
    moveForward();
    if (delayAndCheck(2000)) return;

    turnLeft();
    if (delayAndCheck(1000)) return;

    moveBackward();
    if (delayAndCheck(2000)) return;

    turnRight();
    if (delayAndCheck(1000)) return;

    stopRobot();
    if (delayAndCheck(2000)) return;
  } else {
    stopRobot();
  }
}

// Updates telemetry non-blockingly & handles safety logic
void readSensorsNonBlocking() {
  // DHT22 sampling (minimum 2000ms interval)
  if (millis() - lastDHTRead >= 2000) {
    lastDHTRead = millis();
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(h) && !isnan(t)) {
      dhtHumidity = h;
      dhtTemp = t;
    }
  }

  // General Telemetry (500ms interval)
  if (millis() - lastSensorRead >= 500) {
    lastSensorRead = millis();

    float objectC = mlx.readObjectTempC();
    float ammoniaPPM = getMQ135PPM(analogRead(MQ135_PIN));
    sonarDistanceCM = getSonarDistanceCM();

    // Pump Safety Logic: Turn OFF if Ammonia exceeds threshold
    if (ammoniaPPM > NH3_THRESHOLD_PPM) {
      digitalWrite(PUMP_OUT1, HIGH);  // Disconnects GND -> Pump turns OFF
    } else {
      digitalWrite(PUMP_OUT1, LOW);   // Connects GND -> Pump stays ON
    }

    // Serial Stream Output
    Serial.print("Distance: ");
    if (sonarDistanceCM > 0) {
      Serial.print(sonarDistanceCM, 1);
      Serial.print(" cm");
    } else {
      Serial.print("Out of Range");
    }
    Serial.print(" | IR Temp: ");
    Serial.print(objectC);
    Serial.print(" °C | Air Temp: ");
    Serial.print(dhtTemp);
    Serial.print(" °C | Humidity: ");
    Serial.print(dhtHumidity);
    Serial.print(" % | NH3: ");
    Serial.print(ammoniaPPM, 1);
    Serial.print(" PPM | Pump: ");
    Serial.println(ammoniaPPM > NH3_THRESHOLD_PPM ? "OFF (HIGH NH3)" : "ON");
  }
}

void checkButton() {
  bool currentButtonState = digitalRead(BUTTON_S1);
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    isRobotOn = !isRobotOn;
    delay(200); // Debounce
  }
  lastButtonState = currentButtonState;
}

bool delayAndCheck(int ms) {
  int elapsed = 0;
  while (elapsed < ms) {
    checkButton();
    readSensorsNonBlocking();
    if (!isRobotOn) {
      stopRobot();
      return true;
    }
    delay(50);
    elapsed += 50;
  }
  return false;
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