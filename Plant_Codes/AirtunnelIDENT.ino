/*
   ESP32-S3 WIND TUNNEL CONTROLLER
   --------------------------------

   FEATURES
   - BLDC PWM control
   - Gradual motor stop
   - VL53L0X sensing
   - Continuous rotation servo
   - Linear calibrated sensor
   - Moving average filter
   - LittleFS CSV logging
   - Persistent settings using Preferences
   - SBPA / PRBS identification
   - Runtime configurable system
   - Interruptable SBPA
   - Persistent configuration after reboot

   SETTINGS STORED IN FLASH
   --------------------------------
   - SBPA HIGH
   - SBPA LOW
   - SBPA SEED
   - SAMPLE_MS
   - IDENT_MS
   - STAB_MS
   - DEBUG

   SERIAL COMMANDS
   --------------------------------

   MOTOR <0-100>

   MOTOR_STOP

   SERVO <0-360>

   SERVO_SPEED <0-100>

   LOG_START
   LOG_STOP

   PRINT

   RESET

   DEBUG 0
   DEBUG 1

   SEED <number>

   SBPA_HIGH <duty>

   SBPA_LOW <duty>

   STAB_MS <ms>

   SAMPLE_MS <ms>

   IDENT_MS <ms>

   SBPA_START

   SBPA_STOP

   STATUS

   STOP
*/

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <ESP32Servo.h>
#include <LittleFS.h>
#include <Preferences.h>

// ======================================================
// PIN DEFINITIONS
// ======================================================

#define BLDC_PWM_PIN      7
#define SERVO_PIN         12

#define I2C_SDA           5
#define I2C_SCL           4

// ======================================================
// PWM CONFIG
// ======================================================

#define BLDC_PWM_FREQ     50
#define BLDC_PWM_RES      12

// ======================================================
// SENSOR FILTER
// ======================================================

#define FILTER_SIZE 15

float distanceBuffer[FILTER_SIZE];

uint8_t bufferIndex = 0;

bool bufferFilled = false;

// ======================================================
// LOGGER
// ======================================================

#define LOG_BUFFER_SIZE 50

struct LogEntry {

  uint32_t timeMs;

  float pwm;

  float height;
};

LogEntry logBuffer[LOG_BUFFER_SIZE];

uint16_t logIndex = 0;

bool loggingEnabled = false;

// ======================================================
// PREFERENCES
// ======================================================

Preferences prefs;

// ======================================================
// SENSOR DEBUG
// ======================================================

bool sensorDebug = true;

// ======================================================
// SERVO
// ======================================================

Servo irisServo;

const float SERVO_NEUTRAL_US = 1500.0;

const float SERVO_MIN_US = 1000.0;

const float SERVO_MAX_US = 2000.0;

const float MAX_SERVO_SPEED_DPS = 50.0;

float servoSpeedPercent = 100.0;

// ======================================================
// SENSOR
// ======================================================

VL53L0X tof;

// ======================================================
// SBPA CONFIG
// ======================================================

float dutyHigh = 7.10;

float dutyLow = 6.86;

uint32_t sbpaSeed = 12345;

uint32_t identTimeMs = 20000;

uint32_t samplePeriodMs = 100;

uint32_t stabilizationMs = 2000;

#define MAX_SBPA_SAMPLES 2000

uint8_t sbpaVector[MAX_SBPA_SAMPLES];

int numSamples = 0;

bool sbpaRunning = false;

uint32_t sbpaStartTime = 0;

uint32_t sbpaLastUpdate = 0;

int sbpaSampleIndex = 0;

// ======================================================
// IDENTIFICATION TIME BASE
// ======================================================

uint32_t identificationStartMillis = 0;

// ======================================================
// GLOBAL VARIABLES
// ======================================================

float currentServoAngle = 0.0;

float targetServoAngle = 0.0;

bool servoMoving = false;

unsigned long lastServoUpdate = 0;

unsigned long lastSensorPrint = 0;

unsigned long lastLogTime = 0;

float motorDuty = 5.5;

// ======================================================
// FUNCTION DECLARATIONS
// ======================================================

void setMotorDuty(float percent);

void gradualMotorStop();

void updateServo();

void stopServo();

void setServoDirection(float speed);

void processSerial();

void printStatus();

float getFilteredDistance();

void addLogEntry(float pwm, float height);

void flushLogBuffer();

void createCSVHeader();

void printCSVFile();

void resetCSVFile();

void generateSBPA();

void startSBPA();

void stopSBPA();

void runSBPA();

void saveSettings();

void loadSettings();

// ======================================================
// SETTINGS SAVE
// ======================================================

void saveSettings() {

  prefs.putFloat("dHigh", dutyHigh);

  prefs.putFloat("dLow", dutyLow);

  prefs.putULong("seed", sbpaSeed);

  prefs.putULong("ident", identTimeMs);

  prefs.putULong("sample", samplePeriodMs);

  prefs.putULong("stab", stabilizationMs);

  prefs.putBool("debug", sensorDebug);
}

// ======================================================
// SETTINGS LOAD
// ======================================================

void loadSettings() {

  dutyHigh =
      prefs.getFloat("dHigh", 7.10);

  dutyLow =
      prefs.getFloat("dLow", 6.86);

  sbpaSeed =
      prefs.getULong("seed", 12345);

  identTimeMs =
      prefs.getULong("ident", 20000);

  samplePeriodMs =
      prefs.getULong("sample", 100);

  stabilizationMs =
      prefs.getULong("stab", 2000);

  sensorDebug =
      prefs.getBool("debug", true);
}

// ======================================================
// SETUP
// ======================================================

void setup() {

  Serial.begin(115200);

  // ====================================================
  // PREFERENCES
  // ====================================================

  prefs.begin("windTunnel", false);

  loadSettings();

  // ====================================================
  // LITTLEFS
  // ====================================================

  if (!LittleFS.begin(true)) {

    Serial.println("LittleFS mount failed");

    while (1);
  }

  Serial.println("LittleFS mounted");

  if (!LittleFS.exists("/plant_data.csv")) {

    createCSVHeader();
  }

  // ====================================================
  // PWM
  // ====================================================

  ledcAttach(
      BLDC_PWM_PIN,
      BLDC_PWM_FREQ,
      BLDC_PWM_RES
  );

  setMotorDuty(5.5);

  // ====================================================
  // SENSOR
  // ====================================================

  Wire.begin(I2C_SDA, I2C_SCL);

  tof.setTimeout(500);

  if (!tof.init()) {

    Serial.println("VL53L0X INIT FAILED");
  }
  else {

    Serial.println("VL53L0X READY");

    tof.startContinuous();

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

  Serial.println("\n=== SYSTEM READY ===");
}

// ======================================================
// LOOP
// ======================================================

void loop() {

  processSerial();

  updateServo();

  runSBPA();

  float dist = getFilteredDistance();

  if (sensorDebug &&
      millis() - lastSensorPrint > 20) {

    lastSensorPrint = millis();

    Serial.print("Height:");

    Serial.println(dist);
  }

  if (loggingEnabled &&
      millis() - lastLogTime > 20) {

    lastLogTime = millis();

    addLogEntry(motorDuty, dist);
  }
}

// ======================================================
// SENSOR
// ======================================================

float getFilteredDistance() {

  uint16_t raw =
      tof.readRangeContinuousMillimeters();

  if (tof.timeoutOccurred()) {

    return -1;
  }

  float corrected =
      (1.430977f * raw) - 135.655228f;

  distanceBuffer[bufferIndex] = corrected;

  bufferIndex++;

  if (bufferIndex >= FILTER_SIZE) {

    bufferIndex = 0;

    bufferFilled = true;
  }

  uint8_t count =
      bufferFilled ? FILTER_SIZE : bufferIndex;

  if (count == 0)
    return corrected;

  float sum = 0;

  for (int i = 0; i < count; i++) {

    sum += distanceBuffer[i];
  }

  return sum / count;
}

// ======================================================
// CSV
// ======================================================

void createCSVHeader() {

  File file =
      LittleFS.open("/plant_data.csv", "w");

  if (file) {

    file.println("time_ms,pwm,height_mm");

    file.close();
  }
}

void addLogEntry(float pwm, float height) {

  uint32_t relativeTime =
      millis() - identificationStartMillis;

  logBuffer[logIndex].timeMs = relativeTime;

  logBuffer[logIndex].pwm = pwm;

  logBuffer[logIndex].height = height;

  logIndex++;

  if (logIndex >= LOG_BUFFER_SIZE) {

    flushLogBuffer();
  }
}

void flushLogBuffer() {

  File file =
      LittleFS.open("/plant_data.csv", "a");

  if (!file)
    return;

  for (int i = 0; i < logIndex; i++) {

    file.print(logBuffer[i].timeMs);

    file.print(",");

    file.print(logBuffer[i].pwm, 3);

    file.print(",");

    file.println(logBuffer[i].height, 3);
  }

  file.close();

  logIndex = 0;
}

void printCSVFile() {

  if (logIndex > 0) {

    flushLogBuffer();
  }

  Serial.print("# Motor Duty: ");
  Serial.println(motorDuty);

  Serial.print("# SBPA HIGH: ");
  Serial.println(dutyHigh, 3);

  Serial.print("# SBPA LOW: ");
  Serial.println(dutyLow, 3);

  Serial.print("# Seed: ");
  Serial.println(sbpaSeed);

  Serial.print("# Stabilization: ");
  Serial.println(stabilizationMs);

  Serial.print("# SamplePeriod: ");
  Serial.println(samplePeriodMs);

  Serial.print("# IdentTime: ");
  Serial.println(identTimeMs);

  File file =
      LittleFS.open("/plant_data.csv", "r");

  if (!file)
    return;

  while (file.available()) {

    Serial.write(file.read());
  }

  file.close();
}

void resetCSVFile() {

  LittleFS.remove("/plant_data.csv");

  createCSVHeader();

  logIndex = 0;
}

// ======================================================
// SBPA
// ======================================================

void generateSBPA() {

  randomSeed(sbpaSeed);

  numSamples =
      identTimeMs / samplePeriodMs;

  if (numSamples > MAX_SBPA_SAMPLES) {

    numSamples = MAX_SBPA_SAMPLES;
  }

  for (int i = 0; i < numSamples; i++) {

    sbpaVector[i] = random(0, 2);
  }
}

void startSBPA() {

  generateSBPA();

  resetCSVFile();

  loggingEnabled = true;

  sbpaRunning = true;

  sbpaStartTime = millis();

  identificationStartMillis = millis();

  sbpaLastUpdate = millis();

  sbpaSampleIndex = 0;

  setMotorDuty(dutyHigh);

  Serial.println("SBPA started");
}

void stopSBPA() {

  if (!sbpaRunning) {

    Serial.println("SBPA not running");

    return;
  }

  sbpaRunning = false;

  loggingEnabled = false;

  flushLogBuffer();

  Serial.println("SBPA interrupted");
}

void runSBPA() {

  if (!sbpaRunning)
    return;

  uint32_t now = millis();

  if ((now - sbpaStartTime) < stabilizationMs) {

    setMotorDuty(dutyHigh);

    return;
  }

  if (sbpaSampleIndex >= numSamples) {

    sbpaRunning = false;

    loggingEnabled = false;

    flushLogBuffer();

    setMotorDuty(dutyHigh);

    Serial.println("SBPA finished");

    return;
  }

  if ((now - sbpaLastUpdate) >= samplePeriodMs) {

    sbpaLastUpdate = now;

    uint8_t bit =
        sbpaVector[sbpaSampleIndex];

    float duty =
        bit ? dutyHigh : dutyLow;

    setMotorDuty(duty);

    sbpaSampleIndex++;
  }
}

// ======================================================
// MOTOR
// ======================================================

void setMotorDuty(float percent) {

  percent = constrain(percent, 0, 100);

  motorDuty = percent;

  uint32_t maxPWM = 4095;

  uint32_t pwm =
      (percent / 100.0f) * maxPWM;

  ledcWrite(BLDC_PWM_PIN, pwm);
}

void gradualMotorStop() {

  while (motorDuty > 5.5f) {

    motorDuty -= 0.2f;

    if (motorDuty < 5.5f)
      motorDuty = 5.5f;

    setMotorDuty(motorDuty);

    delay(300);
  }
}

// ======================================================
// SERVO
// ======================================================

void stopServo() {

  irisServo.writeMicroseconds(
      (int)SERVO_NEUTRAL_US
  );

  servoMoving = false;
}

void setServoDirection(float speed) {

  speed = constrain(speed, -1.0f, 1.0f);

  float us =
      SERVO_NEUTRAL_US +
      (speed * 500.0f);

  us = constrain(
      us,
      SERVO_MIN_US,
      SERVO_MAX_US
  );

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

    return;
  }

  setServoDirection(direction * speedFactor);
}

// ======================================================
// SERIAL
// ======================================================

void processSerial() {

  if (!Serial.available())
    return;

  String cmd =
      Serial.readStringUntil('\n');

  cmd.trim();

  if (cmd.startsWith("MOTOR ")) {

    float value =
        cmd.substring(6).toFloat();

    setMotorDuty(value);
  }

  else if (cmd == "MOTOR_STOP") {

    gradualMotorStop();
  }

  else if (cmd.startsWith("DEBUG")) {

    int state =
        cmd.substring(6).toInt();

    sensorDebug = (state != 0);

    saveSettings();
  }

  else if (cmd.startsWith("SEED")) {

    sbpaSeed =
        cmd.substring(5).toInt();

    saveSettings();
  }

  else if (cmd.startsWith("SBPA_HIGH")) {

    dutyHigh =
        cmd.substring(10).toFloat();

    saveSettings();
  }

  else if (cmd.startsWith("SBPA_LOW")) {

    dutyLow =
        cmd.substring(9).toFloat();

    saveSettings();
  }

  else if (cmd.startsWith("STAB_MS")) {

    stabilizationMs =
        cmd.substring(8).toInt();

    saveSettings();
  }

  else if (cmd.startsWith("SAMPLE_MS")) {

    samplePeriodMs =
        cmd.substring(10).toInt();

    saveSettings();
  }

  else if (cmd.startsWith("IDENT_MS")) {

    identTimeMs =
        cmd.substring(9).toInt();

    saveSettings();
  }

  else if (cmd == "SBPA_START") {

    startSBPA();
  }

  else if (cmd == "SBPA_STOP") {

    stopSBPA();
  }

  else if (cmd == "LOG_START") {

    resetCSVFile();

    identificationStartMillis = millis();

    loggingEnabled = true;
  }

  else if (cmd == "LOG_STOP") {

    loggingEnabled = false;

    flushLogBuffer();
  }

  else if (cmd == "PRINT") {

    printCSVFile();
  }

  else if (cmd == "RESET") {

    resetCSVFile();
  }

  else if (cmd.startsWith("SERVO_SPEED")) {

    servoSpeedPercent =
        cmd.substring(12).toFloat();
  }

  else if (cmd.startsWith("SERVO ")) {

    targetServoAngle =
        cmd.substring(6).toFloat();

    servoMoving = true;

    lastServoUpdate = millis();
  }

  else if (cmd == "STOP") {

    stopServo();
  }

  else if (cmd == "STATUS") {

    printStatus();
  }
}

// ======================================================
// STATUS
// ======================================================

void printStatus() {

  Serial.println("\n====== STATUS ======");

  Serial.print("Motor Duty: ");
  Serial.println(motorDuty);

  Serial.print("Height: ");
  Serial.println(getFilteredDistance());

  Serial.print("SBPA HIGH: ");
  Serial.println(dutyHigh, 3);

  Serial.print("SBPA LOW: ");
  Serial.println(dutyLow, 3);

  Serial.print("Seed: ");
  Serial.println(sbpaSeed);

  Serial.print("Stabilization: ");
  Serial.println(stabilizationMs);

  Serial.print("Sample Period: ");
  Serial.println(samplePeriodMs);

  Serial.print("Identification Time: ");
  Serial.println(identTimeMs);

  Serial.print("Samples: ");
  Serial.println(numSamples);

  Serial.print("SBPA Running: ");
  Serial.println(sbpaRunning ? "YES" : "NO");

  Serial.println("====================");
}