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

#define NH3_THRESHOLD_PPM 25.0 // Ammonia threshold to activate pump

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
float ammoniaPPM = 0.0;         // Global cache for ammonia level

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
  digitalWrite(PUMP_OUT1, HIGH); // Ensure pump starts OFF (Active-LOW)

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
    // 1. Walk forward
    moveForward();

    // 2. Detect object closer than 25cm
    if (sonarDistanceCM > 0.0 && sonarDistanceCM < 25.0) {
      stopRobot();
      Serial.println("Obstacle detected (< 25cm)! Stopped.");

      // 3. Wait 3 seconds while keeping sensors and button responsive
      unsigned long stopStartTime = millis();
      while (millis() - stopStartTime < 3000) {
        checkButton();
        readSensorsNonBlocking();
        if (!isRobotOn) {
          stopRobot();
          digitalWrite(PUMP_OUT1, HIGH); // Turn off pump if robot toggled off
          return;
        }
        delay(50);
      }

      // Check Ammonia PPM after 3-second wait
      if (ammoniaPPM > NH3_THRESHOLD_PPM) {
        Serial.println("Ammonia > 25 PPM! Activating pump...");
        digitalWrite(PUMP_OUT1, LOW); // Activate pump (Active-LOW)

        // Keep pumping until PPM drops below threshold
        while (isRobotOn && (ammoniaPPM > NH3_THRESHOLD_PPM)) {
          checkButton();
          readSensorsNonBlocking();
          delay(100);
        }

        digitalWrite(PUMP_OUT1, HIGH); // Deactivate pump
        Serial.println("Ammonia level clear (< 25 PPM). Pump deactivated.");
      }
    }
  } else {
    stopRobot();
    digitalWrite(PUMP_OUT1, HIGH); // Ensure pump stays OFF when robot is stopped
  }
}

// Updates telemetry non-blockingly
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
    ammoniaPPM = getMQ135PPM(analogRead(MQ135_PIN));
    sonarDistanceCM = getSonarDistanceCM();

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
    Serial.println(digitalRead(PUMP_OUT1) == LOW ? "ON" : "OFF");
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