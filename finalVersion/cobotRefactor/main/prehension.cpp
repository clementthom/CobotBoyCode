#include <Arduino.h>
#include "prehension.h"
#include "userInterfaceHardware.h"

// ====================================================
// [prehension.cpp]  GLOBALS — setpoints, PID state, gripper FSM
// ====================================================

double consigneOuverture = 1.80;
double consigneFermeture = 1.40;

double seuilOuvertureMax = 2.30;
double seuilFermetureMax = 2.60;

GripperState gripperState = GRIPPER_STOPPED;

double gripperPWM_PID = 0.0;
double measuredTension = 0.0;
double gripperSetpoint = 0.0;

double filteredTension = 0.0;
const double alphaGrip = 0.25;

double previousGripPWM = 0.0;
const double maxGripPWMVariation = 15.0;

bool gripperDone = false;
bool objectDetected = false;
bool openLimitDetected = false;

AutoPID pidGripper(
  &measuredTension,
  &gripperSetpoint,
  &gripperPWM_PID,
  0.0,
  255.0,
  KP_GRIP,
  KI_GRIP,
  KD_GRIP);


// ====================================================
// [prehension.cpp]  VOLTAGE READING
// ====================================================

double readVoltage(int pin) {
  return ((double)analogRead(pin) * 5.0) / 1023.0;
}


// ====================================================
// [prehension.cpp]  GRIPPER ACTUATOR COMMANDS
// ====================================================

void applyGripperOpening(double pwm) {
  digitalWrite(PIN_GRIP_INA, HIGH);
  digitalWrite(PIN_GRIP_INB, LOW);
  analogWrite(PIN_GRIP_PWM, constrain((int)pwm, 0, 255));
}

void applyGripperClosing(double pwm) {
  digitalWrite(PIN_GRIP_INA, LOW);
  digitalWrite(PIN_GRIP_INB, HIGH);
  analogWrite(PIN_GRIP_PWM, constrain((int)pwm, 0, 255));
}

void stopGripper() {
  gripperState = GRIPPER_STOPPED;

  analogWrite(PIN_GRIP_PWM, 0);
  digitalWrite(PIN_GRIP_INA, LOW);
  digitalWrite(PIN_GRIP_INB, LOW);

  gripperPWM_PID = 0.0;
  previousGripPWM = 0.0;
}

void openGripper() {
  gripperState = GRIPPER_OPENING;
  gripperDone = false;
  openLimitDetected = false;
  objectDetected = false;

  gripperSetpoint = consigneOuverture;
  gripperPWM_PID = 0.0;
  previousGripPWM = 0.0;
  filteredTension = 0.0;
}

void closeGripper() {
  gripperState = GRIPPER_CLOSING;
  gripperDone = false;
  objectDetected = false;
  openLimitDetected = false;

  gripperSetpoint = consigneFermeture;
  gripperPWM_PID = 0.0;
  previousGripPWM = 0.0;
  filteredTension = 0.0;
}


// ====================================================
// [prehension.cpp]  GRIPPER PID UPDATE LOOP
// ====================================================

void updateGripperPID() {
  if (gripperState == GRIPPER_STOPPED) return;

  double voltageOpen = readVoltage(CURRENT_OPEN_PIN);
  double voltageClose = readVoltage(CURRENT_CLOSE_PIN);

  double rawTension;

  if (gripperState == GRIPPER_OPENING) {
    rawTension = voltageOpen;
    gripperSetpoint = consigneOuverture;
  } else {
    rawTension = voltageClose;
    gripperSetpoint = consigneFermeture;
  }

  filteredTension = filteredTension + alphaGrip * (rawTension - filteredTension);
  measuredTension = filteredTension;

  pidGripper.run();

  if (gripperPWM_PID > previousGripPWM + maxGripPWMVariation) {
    gripperPWM_PID = previousGripPWM + maxGripPWMVariation;
  }

  if (gripperPWM_PID < previousGripPWM - maxGripPWMVariation) {
    gripperPWM_PID = previousGripPWM - maxGripPWMVariation;
  }

  previousGripPWM = gripperPWM_PID;

  if (gripperState == GRIPPER_OPENING) {
    applyGripperOpening(gripperPWM_PID);
  } else if (gripperState == GRIPPER_CLOSING) {
    applyGripperClosing(gripperPWM_PID);
  }
}

