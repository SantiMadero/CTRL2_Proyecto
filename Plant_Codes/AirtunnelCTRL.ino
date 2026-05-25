/*
   ESP32-S3 WIND TUNNEL CONTROLLER
   --------------------------------

   REFACTORED STATUS SYSTEM

   STATUS
     -> General system status

   CTRL_STATUS
     -> Active controller parameters

   SBPA_STATUS
     -> Identification configuration
        + appended to CSV print
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

#define BLDC_PWM_PIN 7
#define SERVO_PIN    12

#define I2C_SDA      5
#define I2C_SCL      4

// ======================================================
// PWM CONFIG
// ======================================================

#define BLDC_PWM_FREQ 50
#define BLDC_PWM_RES  12

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
// DEBUG
// ======================================================

bool sensorDebug = true;

bool controllerTestMode = false;

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
// CONTROLLERS
// ======================================================

enum ControllerType {

  CONTROLLER_NONE = 0,
  CONTROLLER_PID,
  CONTROLLER_P
};

ControllerType activeController =
    CONTROLLER_NONE;

bool controllerEnabled = false;

// PID

float pidKp = 0.01f;

float pidKi = 0.0005f;

float pidKd = 0.0f;

// P

float pGain = 0.0085f;

// States

float pidError = 0.0f;

float pidPrevError = 0.0f;

float pidIntegral = 0.0f;

float pidDerivative = 0.0f;

float pidOutput = 5.5f;

// Reference

float controlReference = 300.0f;

// Saturation

float controlOutputMin = 5.5f;

float controlOutputMax = 8.0f;

// Timing

uint32_t controlPeriodMs = 20;

uint32_t lastControlUpdate = 0;

// Warmup

float warmupPWM = 7.15f;

uint32_t warmupTimeMs = 5000;

// ======================================================
// IDENTIFICATION TIME BASE
// ======================================================

uint32_t identificationStartMillis = 0;

// ======================================================
// GLOBALS
// ======================================================

float currentServoAngle = 0.0;

float targetServoAngle = 0.0;

bool servoMoving = false;

unsigned long lastServoUpdate = 0;

unsigned long lastSensorPrint = 0;

unsigned long lastLogTime = 0;

float motorDuty = 5.5;

// ======================================================
// DECLARATIONS
// ======================================================

void setMotorDuty(float percent);

void gradualMotorStop();

void updateServo();

void stopServo();

void setServoDirection(float speed);

void processSerial();

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

void updateController();

float runPID(float reference, float measurement);

float runP(float reference, float measurement);

void setController(ControllerType type);

void stopController();

void controllerWarmup();

void printStatus();

void printControllerStatus();

void printSBPAStatus();

// ======================================================
// SETTINGS
// ======================================================

void saveSettings() {

  prefs.putFloat("dHigh", dutyHigh);

  prefs.putFloat("dLow", dutyLow);

  prefs.putULong("seed", sbpaSeed);

  prefs.putULong("ident", identTimeMs);

  prefs.putULong("sample", samplePeriodMs);

  prefs.putULong("stab", stabilizationMs);

  prefs.putBool("debug", sensorDebug);

  prefs.putFloat("kp", pidKp);

  prefs.putFloat("ki", pidKi);

  prefs.putFloat("kd", pidKd);

  prefs.putFloat("pgain", pGain);

  prefs.putULong("ctrlms", controlPeriodMs);

  prefs.putFloat("warmup", warmupPWM);

  prefs.putFloat("outmin", controlOutputMin);

  prefs.putFloat("outmax", controlOutputMax);
}

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

  pidKp =
      prefs.getFloat("kp", 0.01);

  pidKi =
      prefs.getFloat("ki", 0.0005);

  pidKd =
      prefs.getFloat("kd", 0.0);

  pGain =
      prefs.getFloat("pgain", 0.0085);

  controlPeriodMs =
      prefs.getULong("ctrlms", 20);

  warmupPWM =
      prefs.getFloat("warmup", 7.15);

  controlOutputMin =
      prefs.getFloat("outmin", 5.5);

  controlOutputMax =
      prefs.getFloat("outmax", 8.0);
}

// ======================================================
// CONTROLLER
// ======================================================

void controllerWarmup() {

  pidIntegral = 0.0f;

  pidPrevError = 0.0f;

  pidDerivative = 0.0f;

  pidOutput = warmupPWM;

  uint32_t warmupStart = millis();

  while ((millis() - warmupStart) <
         warmupTimeMs) {

    setMotorDuty(warmupPWM);

    float measurement =
        getFilteredDistance();

    switch (activeController) {

      case CONTROLLER_PID:

        runPID(
            controlReference,
            measurement
        );

        break;

      case CONTROLLER_P:

        runP(
            controlReference,
            measurement
        );

        break;

      default:
        break;
    }

    delay(controlPeriodMs);
  }
}

void updateController() {

  if (!controllerEnabled)
    return;

  uint32_t now = millis();

  if ((now - lastControlUpdate) <
      controlPeriodMs)
    return;

  lastControlUpdate = now;

  float measurement =
      getFilteredDistance();

  float controlSignal = motorDuty;

  switch (activeController) {

    case CONTROLLER_PID:

      controlSignal =
          runPID(
              controlReference,
              measurement
          );

      break;

    case CONTROLLER_P:

      controlSignal =
          runP(
              controlReference,
              measurement
          );

      break;

    default:
      return;
  }

  controlSignal = constrain(
      controlSignal,
      controlOutputMin,
      controlOutputMax
  );

  pidOutput = controlSignal;

  if (!controllerTestMode) {

    setMotorDuty(controlSignal);
  }
}

float runPID(
    float reference,
    float measurement
) {

  pidError =
      reference - measurement;

  pidIntegral += pidError;

  pidDerivative =
      pidError - pidPrevError;

  float output =
      (pidKp * pidError) +
      (pidKi * pidIntegral) +
      (pidKd * pidDerivative);

  pidPrevError = pidError;

  pidOutput += output;

  return pidOutput;
}

float runP(
    float reference,
    float measurement
) {

  float error =
      reference - measurement;

  return warmupPWM + (pGain * error);
}

void setController(
    ControllerType type
) {

  activeController = type;
}

void stopController() {

  controllerEnabled = false;
}

// ======================================================
// SETUP
// ======================================================

void setup() {

  Serial.begin(115200);

  prefs.begin("windTunnel", false);

  loadSettings();

  LittleFS.begin(true);

  if (!LittleFS.exists("/plant_data.csv")) {

    createCSVHeader();
  }

  ledcAttach(
      BLDC_PWM_PIN,
      BLDC_PWM_FREQ,
      BLDC_PWM_RES
  );

  setMotorDuty(5.5);

  Wire.begin(I2C_SDA, I2C_SCL);

  tof.setTimeout(500);

  if (tof.init()) {

    tof.startContinuous();
  }

  irisServo.setPeriodHertz(50);

  irisServo.attach(
      SERVO_PIN,
      SERVO_MIN_US,
      SERVO_MAX_US
  );

  stopServo();

  Serial.println("SYSTEM READY");
}

// ======================================================
// LOOP
// ======================================================

void loop() {

  processSerial();

  updateServo();

  runSBPA();

  updateController();

  float dist = getFilteredDistance();

  if (sensorDebug &&
      millis() - lastSensorPrint > 20) {

    lastSensorPrint = millis();

    Serial.print("Height:");

    Serial.print(dist);

    Serial.print(",");

    Serial.print("Control:");

    Serial.print(pidOutput);

    Serial.print(",");

    Serial.print("Motor:");

    Serial.println(motorDuty);
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

  File file =
      LittleFS.open("/plant_data.csv", "r");

  if (!file)
    return;

  while (file.available()) {

    Serial.write(file.read());
  }

  file.close();

  // Append SBPA configuration

  printSBPAStatus();
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
}

void stopSBPA() {

  sbpaRunning = false;

  loggingEnabled = false;

  flushLogBuffer();
}

void runSBPA() {

  if (!sbpaRunning)
    return;

  uint32_t now = millis();

  if ((now - sbpaStartTime) <
      stabilizationMs) {

    setMotorDuty(dutyHigh);

    return;
  }

  if (sbpaSampleIndex >= numSamples) {

    sbpaRunning = false;

    loggingEnabled = false;

    flushLogBuffer();

    setMotorDuty(dutyHigh);

    return;
  }

  if ((now - sbpaLastUpdate) >=
      samplePeriodMs) {

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
// STATUS
// ======================================================

void printStatus() {

  Serial.println("\n=== STATUS ===");

  Serial.print("Motor Duty: ");
  Serial.println(motorDuty);

  Serial.print("Height: ");
  Serial.println(getFilteredDistance());

  Serial.print("Controller Enabled: ");
  Serial.println(
      controllerEnabled ? "YES" : "NO"
  );

  Serial.print("Test Mode: ");
  Serial.println(
      controllerTestMode ? "YES" : "NO"
  );

  Serial.print("Controller Output: ");
  Serial.println(pidOutput);

  Serial.println("================");
}

void printControllerStatus() {

  Serial.println("\n=== CTRL STATUS ===");

  Serial.print("Controller: ");

  switch (activeController) {

    case CONTROLLER_NONE:
      Serial.println("NONE");
      break;

    case CONTROLLER_PID:
      Serial.println("PID");
      break;

    case CONTROLLER_P:
      Serial.println("P");
      break;
  }

  Serial.print("Reference: ");
  Serial.println(controlReference);

  Serial.print("Output Min: ");
  Serial.println(controlOutputMin);

  Serial.print("Output Max: ");
  Serial.println(controlOutputMax);

  Serial.print("Warmup PWM: ");
  Serial.println(warmupPWM);

  Serial.print("Control Period: ");
  Serial.println(controlPeriodMs);

  if (activeController ==
      CONTROLLER_PID) {

    Serial.print("Kp: ");
    Serial.println(pidKp, 6);

    Serial.print("Ki: ");
    Serial.println(pidKi, 6);

    Serial.print("Kd: ");
    Serial.println(pidKd, 6);
  }

  if (activeController ==
      CONTROLLER_P) {

    Serial.print("P Gain: ");
    Serial.println(pGain, 6);
  }

  Serial.println("====================");
}

void printSBPAStatus() {

  Serial.println("\nSBPA_STATUS");

  Serial.print("HIGH=");
  Serial.println(dutyHigh, 4);

  Serial.print("LOW=");
  Serial.println(dutyLow, 4);

  Serial.print("SEED=");
  Serial.println(sbpaSeed);

  Serial.print("IDENT_MS=");
  Serial.println(identTimeMs);

  Serial.print("SAMPLE_MS=");
  Serial.println(samplePeriodMs);

  Serial.print("STAB_MS=");
  Serial.println(stabilizationMs);

  Serial.println("END_SBPA_STATUS");
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

  // ==================================================
  // STATUS COMMANDS
  // ==================================================

  if (cmd == "STATUS") {

    printStatus();
  }

  else if (cmd == "CTRL_STATUS") {

    printControllerStatus();
  }

  else if (cmd == "SBPA_STATUS") {

    printSBPAStatus();
  }

  // ==================================================
  // CONTROLLER
  // ==================================================

  else if (cmd == "CTRL_ON") {

    controllerEnabled = true;

    lastControlUpdate = millis();

    controllerWarmup();
  }

  else if (cmd == "CTRL_OFF") {

    stopController();
  }

  else if (cmd == "TEST_ON") {

    controllerTestMode = true;
  }

  else if (cmd == "TEST_OFF") {

    controllerTestMode = false;
  }

  else if (cmd.startsWith("CTRL ")) {

    String type =
        cmd.substring(5);

    type.trim();

    if (type == "PID") {

      setController(
          CONTROLLER_PID
      );
    }

    else if (type == "P") {

      setController(
          CONTROLLER_P
      );
    }

    else if (type == "NONE") {

      setController(
          CONTROLLER_NONE
      );
    }
  }

  else if (cmd.startsWith("KP ")) {

    pidKp =
        cmd.substring(3).toFloat();

    saveSettings();
  }

  else if (cmd.startsWith("KI ")) {

    pidKi =
        cmd.substring(3).toFloat();

    saveSettings();
  }

  else if (cmd.startsWith("KD ")) {

    pidKd =
        cmd.substring(3).toFloat();

    saveSettings();
  }

  else if (cmd.startsWith("PGAIN ")) {

    pGain =
        cmd.substring(6).toFloat();

    saveSettings();
  }

  else if (cmd.startsWith("REF ")) {

    controlReference =
        cmd.substring(4).toFloat();
  }

  else if (cmd.startsWith("CTRL_MS ")) {

    controlPeriodMs =
        cmd.substring(8).toInt();

    saveSettings();
  }

  else if (cmd.startsWith("OUT_MIN ")) {

    controlOutputMin =
        cmd.substring(8).toFloat();

    saveSettings();
  }

  else if (cmd.startsWith("OUT_MAX ")) {

    controlOutputMax =
        cmd.substring(8).toFloat();

    saveSettings();
  }

  else if (cmd.startsWith("WARMUP_PWM ")) {

    warmupPWM =
        cmd.substring(11).toFloat();

    saveSettings();
  }

  // ==================================================
  // MOTOR
  // ==================================================

  else if (cmd.startsWith("MOTOR ")) {

    setMotorDuty(
        cmd.substring(6).toFloat()
    );
  }

  else if (cmd == "MOTOR_STOP") {

    gradualMotorStop();
  }

  // ==================================================
  // DEBUG
  // ==================================================

  else if (cmd.startsWith("DEBUG")) {

    sensorDebug =
        cmd.substring(6).toInt();

    saveSettings();
  }

  // ==================================================
  // SBPA
  // ==================================================

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

  // ==================================================
  // LOGGING
  // ==================================================

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

  // ==================================================
  // SERVO
  // ==================================================

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
}