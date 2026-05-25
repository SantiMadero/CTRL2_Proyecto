/*
   ESP32-S3 Wind Tunnel Controller
   --------------------------------

   FEATURES
   - BLDC PWM output on GPIO 7
   - VL53L0X sensor on I2C
   - Continuous rotation servo on GPIO 12
   - Linear calibrated sensor output
   - Moving average filtering
   - Serial Plotter compatible output
   - Debug toggle for sensor printing

   SERIAL COMMANDS
   ----------------

   MOTOR <0-100>
      Set BLDC duty cycle %

   SERVO <0-360>
      Move servo to absolute angle

   SERVO_SPEED <0-100>
      Set servo speed %

   STATUS
      Print system status

   STOP
      Stop servo motion

   DEBUG 0
      Disable sensor printing

   DEBUG 1
      Enable sensor printing
*/

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <ESP32Servo.h>

// ======================================================
// PIN DEFINITIONS
// ======================================================

#define BLDC_PWM_PIN      7
#define SERVO_PIN         12

#define I2C_SDA           5
#define I2C_SCL           4

// ======================================================
// BLDC PWM CONFIG
// ======================================================

#define BLDC_PWM_FREQ     50
#define BLDC_PWM_RES      12

// ======================================================
// SENSOR FILTER CONFIG
// ======================================================

#define FILTER_SIZE 15

float distanceBuffer[FILTER_SIZE];

uint8_t bufferIndex = 0;

bool bufferFilled = false;

// ======================================================
// SENSOR DEBUG
// ======================================================

bool sensorDebug = true;

// ======================================================
// SERVO CONFIG
// ======================================================

Servo irisServo;

const float SERVO_NEUTRAL_US = 1500.0;
const float SERVO_MIN_US     = 1000.0;
const float SERVO_MAX_US     = 2000.0;

const float MAX_SERVO_SPEED_DPS = 50.0;

float servoSpeedPercent = 100.0;

// ======================================================
// SENSOR
// ======================================================

VL53L0X tof;

// ======================================================
// GLOBAL VARIABLES
// ======================================================

float currentServoAngle = 0.0;
float targetServoAngle  = 0.0;

bool servoMoving = false;

unsigned long lastServoUpdate = 0;
unsigned long lastSensorPrint = 0;

float motorDuty = 5.5;

// ======================================================
// FUNCTION DECLARATIONS
// ======================================================

void setMotorDuty(float percent);

void updateServo();

void stopServo();

void setServoDirection(float speed);

void processSerial();

void printStatus();

float getFilteredDistance();

// ======================================================
// SETUP
// ======================================================

void setup() {

  Serial.begin(115200);

  // ====================================================
  // BLDC PWM
  // ====================================================

  ledcAttach(BLDC_PWM_PIN, BLDC_PWM_FREQ, BLDC_PWM_RES);

  setMotorDuty(5.5);

  // ====================================================
  // I2C + VL53L0X
  // ====================================================

  Wire.begin(I2C_SDA, I2C_SCL);

  tof.setTimeout(500);

  if (!tof.init()) {

    Serial.println("VL53L0X INIT FAILED");
  }
  else {

    Serial.println("VL53L0X READY");

    tof.startContinuous();

    // Initialize filter buffer
    for (int i = 0; i < FILTER_SIZE; i++) {
      distanceBuffer[i] = 0;
    }
  }

  // ====================================================
  // SERVO
  // ====================================================

  irisServo.setPeriodHertz(50);

  irisServo.attach(
    SERVO_PIN,
    SERVO_MIN_US,
    SERVO_MAX_US
  );

  stopServo();

  Serial.println("\n=== WIND TUNNEL CONTROLLER READY ===");
}

// ======================================================
// LOOP
// ======================================================

void loop() {

  processSerial();

  updateServo();

  // ====================================================
  // SENSOR OUTPUT
  // ====================================================

  if (sensorDebug && millis() - lastSensorPrint > 20) {

    lastSensorPrint = millis();

    float dist = getFilteredDistance();

    // Serial Plotter compatible
    Serial.print("Height:");
    Serial.println(dist);
  }
}

// ======================================================
// SENSOR FILTER + LINEAR CALIBRATION
// ======================================================

float getFilteredDistance() {

  uint16_t raw = tof.readRangeContinuousMillimeters();

  if (tof.timeoutOccurred()) {
    return -1;
  }

  // ====================================================
  // LINEAR CALIBRATION
  //
  // corrected = 1.430977 * raw - 135.655228
  // ====================================================

  float corrected =
      (1.430977f * raw) - 135.655228f;

  // ====================================================
  // BUFFER STORAGE
  // ====================================================

  distanceBuffer[bufferIndex] = corrected;

  bufferIndex++;

  if (bufferIndex >= FILTER_SIZE) {

    bufferIndex = 0;

    bufferFilled = true;
  }

  // ====================================================
  // MOVING AVERAGE FILTER
  // ====================================================

  uint8_t count =
      bufferFilled ? FILTER_SIZE : bufferIndex;

  if (count == 0)
    return corrected;

  float sum = 0;

  for (int i = 0; i < count; i++) {
    sum += distanceBuffer[i];
  }

  float filtered = sum / count;

  return filtered;
}

// ======================================================
// MOTOR CONTROL
// ======================================================

void setMotorDuty(float percent) {

  percent = constrain(percent, 0, 100);

  motorDuty = percent;

  uint32_t maxPWM = 4095;

  uint32_t pwm =
      (percent / 100.0f) * maxPWM;

  ledcWrite(BLDC_PWM_PIN, pwm);

  Serial.print("Motor duty set to: ");
  Serial.print(percent);
  Serial.println("%");
}

// ======================================================
// SERVO CONTROL
// ======================================================

void stopServo() {

  irisServo.writeMicroseconds((int)SERVO_NEUTRAL_US);

  servoMoving = false;
}

void setServoDirection(float speed) {

  speed = constrain(speed, -1.0f, 1.0f);

  float us =
      SERVO_NEUTRAL_US + (speed * 500.0f);

  us = constrain(us,
                 SERVO_MIN_US,
                 SERVO_MAX_US);

  irisServo.writeMicroseconds((int)us);
}

void updateServo() {

  if (!servoMoving)
    return;

  unsigned long now = millis();

  float dt =
      (now - lastServoUpdate) / 1000.0f;

  lastServoUpdate = now;

  float direction =
      (targetServoAngle > currentServoAngle)
      ? 1.0f
      : -1.0f;

  float speedFactor =
      servoSpeedPercent / 100.0f;

  float deltaAngle =
      direction *
      MAX_SERVO_SPEED_DPS *
      speedFactor *
      dt;

  currentServoAngle += deltaAngle;

  if ((direction > 0 &&
       currentServoAngle >= targetServoAngle) ||

      (direction < 0 &&
       currentServoAngle <= targetServoAngle)) {

    currentServoAngle = targetServoAngle;

    stopServo();

    Serial.print("Servo reached angle: ");
    Serial.println(currentServoAngle);

    return;
  }

  setServoDirection(direction * speedFactor);
}

// ======================================================
// SERIAL COMMANDS
// ======================================================

void processSerial() {

  if (!Serial.available())
    return;

  String cmd = Serial.readStringUntil('\n');

  cmd.trim();

  // ====================================================
  // MOTOR
  // ====================================================

  if (cmd.startsWith("MOTOR")) {

    float value =
        cmd.substring(6).toFloat();

    setMotorDuty(value);
  }

  // ====================================================
  // SERVO SPEED
  // ====================================================

  else if (cmd.startsWith("SERVO_SPEED")) {

    float value =
        cmd.substring(12).toFloat();

    servoSpeedPercent =
        constrain(value, 1, 100);

    Serial.print("Servo speed set to: ");
    Serial.print(servoSpeedPercent);
    Serial.println("%");
  }

  // ====================================================
  // SERVO POSITION
  // ====================================================

  else if (cmd.startsWith("SERVO")) {

    float angle =
        cmd.substring(6).toFloat();

    angle = constrain(angle, 0, 360);

    targetServoAngle = angle;

    servoMoving = true;

    lastServoUpdate = millis();

    Serial.print("Moving servo to: ");
    Serial.println(targetServoAngle);
  }

  // ====================================================
  // DEBUG
  // ====================================================

  else if (cmd.startsWith("DEBUG")) {

    int state =
        cmd.substring(6).toInt();

    sensorDebug = (state != 0);

    Serial.print("Sensor debug: ");

    if (sensorDebug)
      Serial.println("ON");
    else
      Serial.println("OFF");
  }

  // ====================================================
  // STATUS
  // ====================================================

  else if (cmd == "STATUS") {

    printStatus();
  }

  // ====================================================
  // STOP
  // ====================================================

  else if (cmd == "STOP") {

    stopServo();

    Serial.println("Servo stopped");
  }

  else {

    Serial.println("Unknown command");
  }
}

// ======================================================
// STATUS
// ======================================================

void printStatus() {

  Serial.println("\n====== STATUS ======");

  Serial.print("Motor Duty: ");
  Serial.print(motorDuty);
  Serial.println("%");

  Serial.print("Servo Angle: ");
  Serial.println(currentServoAngle);

  Serial.print("Target Angle: ");
  Serial.println(targetServoAngle);

  Serial.print("Servo Moving: ");

  Serial.println(
      servoMoving ? "YES" : "NO");

  Serial.print("Filtered Distance: ");
  Serial.print(getFilteredDistance(), 1);
  Serial.println(" mm");

  Serial.print("Sensor Debug: ");

  Serial.println(
      sensorDebug ? "ON" : "OFF");

  Serial.println("====================\n");
}