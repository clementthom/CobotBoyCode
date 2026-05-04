#include <Arduino.h>
#include "maintenance.h"
#include "prehension.h"
#include "cinematics.h"
#include "trajectories.h"
#include "userInterfaceScreen.h"
#include "userInterfaceHardware.h"

// ====================================================
// [maintenance.cpp]  PYTHON TERMINAL SERIAL API
// ====================================================

// Command protocol documentation preserved from original:
// =====================================================
// PYTHON TERMINAL SERIAL API
// This is what makes the Python INIT / PLAY / PAUSE buttons work.
// Commands expected from Python:
//   TERMINAL_ON
//   TERMINAL_OFF
//   PING
//   GET_STATUS
//   GET_CONFIG
//   GET_GRIP_CONFIG
//   SET_CYCLE 0/1
//   SET_PREHENSEUR 0/1
//   SET_OBJECT 0/1/2
//   SET_MODE 0/1/2
//   SET_PIECES 1..5
//   SET_SPEED 0.50..3.00
//   SET_GRIP_CONFIG prehenseur object openSetpoint closeSetpoint
//   GRIP_OPEN / GRIP_CLOSE / GRIP_STOP
//   GRIPPER_OPEN / GRIPPER_CLOSE / GRIPPER_STOP
//   GRIPPER_GOTO_C1_ENTREE_TEST_HEIGHT
//   GRIPPER_AUTO_TEST object
//   INITIALIZE
//   RUN_CYCLE
//   PAUSE_TOGGLE
//   GO_REPOS
//   SERVO_SET A/B/Z angle
//   SERVO_STEP A/B/Z delta
//   BUTTON_TEST_ON / BUTTON_TEST_OFF
//   LED_SET INIT/RUN/PAUSE/POWER 0/1
//   LED_PULSE INIT/RUN/PAUSE/POWER
//   GOTO_POSITION cycle prehenseur object position
//     cycle: 0=Cycle 1, 1=Cycle 2
//     prehenseur: 0=Prehenseur 1 avec offset, 1=Prehenseur 2 centre sans offset outil
//     object: 0=Gomme, 1=Gobelet, 2=Cylindre
//     position: 0=POS_convoyeurEntree, 1=POS_machineA, 2=POS_machineB, 3=POS_convoyeurSortie
// =====================================================
String apiBuffer = "";


void sendStatus() {
  Serial.print(F("STATUS;"));
  Serial.print(F("STATE="));
  Serial.print(getRunStateText());
  Serial.print(F(";"));
  Serial.print(F("INIT="));
  Serial.print(isInitialized ? 1 : 0);
  Serial.print(F(";"));
  Serial.print(F("INITIALIZING="));
  Serial.print(isInitializing ? 1 : 0);
  Serial.print(F(";"));
  Serial.print(F("RUN="));
  Serial.print(isRunning ? 1 : 0);
  Serial.print(F(";"));
  Serial.print(F("PAUSE="));
  Serial.print(isPaused ? 1 : 0);
  Serial.print(F(";"));
  Serial.print(F("CYCLE="));
  Serial.print(currentCycleType);
  Serial.print(F(";"));
  Serial.print(F("CYCLE_NAME="));
  Serial.print(getCycleText());
  Serial.print(F(";"));
  Serial.print(F("PREHENSEUR="));
  Serial.print(currentPrehenseur);
  Serial.print(F(";"));
  Serial.print(F("PREHENSEUR_NAME="));
  Serial.print(getPrehenseurText());
  Serial.print(F(";"));
  Serial.print(F("OBJECT="));
  Serial.print(currentObject);
  Serial.print(F(";"));
  Serial.print(F("OBJECT_NAME="));
  Serial.print(getObjectText());
  Serial.print(F(";"));
  Serial.print(F("MODE="));
  Serial.print(currentMode);
  Serial.print(F(";"));
  Serial.print(F("MODE_NAME="));
  Serial.print(getModeText());
  Serial.print(F(";"));
  Serial.print(F("PIECES="));
  Serial.print(getCombineValue());
  Serial.print(F(";"));
  Serial.print(F("CURRENT_PIECE="));
  Serial.print(currentPiece);
  Serial.print(F(";"));
  Serial.print(F("SPEED_FACTOR="));
  Serial.print(executionSpeedFactor, 2);
  Serial.print(F(";"));
  Serial.print(F("SERVO_A="));
  Serial.print(currentCmdA, 1);
  Serial.print(F(";"));
  Serial.print(F("SERVO_B="));
  Serial.print(currentCmdB, 1);
  Serial.print(F(";"));
  Serial.print(F("SERVO_Z="));
  Serial.print(currentCmdZ, 1);
  Serial.print(F(";"));
  Serial.print(F("GRIP_OPEN="));
  Serial.print(consigneOuverture, 2);
  Serial.print(F(";"));
  Serial.print(F("GRIP_CLOSE="));
  Serial.print(consigneFermeture, 2);
  Serial.print(F(";"));
  Serial.print(F("GRIP_STATE="));
  Serial.println((int)gripperState);
}
void sendConfig() {
  Serial.print(F("CONFIG;"));
  Serial.print(F("CYCLE="));
  Serial.print(currentCycleType);
  Serial.print(F(";"));
  Serial.print(F("PREHENSEUR="));
  Serial.print(currentPrehenseur);
  Serial.print(F(";"));
  Serial.print(F("OBJECT="));
  Serial.print(currentObject);
  Serial.print(F(";"));
  Serial.print(F("MODE="));
  Serial.print(currentMode);
  Serial.print(F(";"));
  Serial.print(F("PIECES="));
  Serial.print(getCombineValue());
  Serial.print(F(";"));
  Serial.print(F("SPEED_FACTOR="));
  Serial.print(executionSpeedFactor, 2);
  Serial.print(F(";"));
  Serial.print(F("GRIP_OPEN="));
  Serial.print(consigneOuverture, 2);
  Serial.print(F(";"));
  Serial.print(F("GRIP_CLOSE="));
  Serial.println(consigneFermeture, 2);
}
void sendGripConfig() {
  applyGripperSettingsForObject(currentObject);

  Serial.print(F("GRIP_CONFIG;"));
  Serial.print(F("PREHENSEUR="));
  Serial.print(currentPrehenseur);
  Serial.print(F(";"));
  Serial.print(F("PREHENSEUR_NAME="));
  Serial.print(getPrehenseurText());
  Serial.print(F(";"));
  Serial.print(F("OBJECT="));
  Serial.print(currentObject);
  Serial.print(F(";"));
  Serial.print(F("OBJECT_NAME="));
  Serial.print(getObjectText());
  Serial.print(F(";"));
  Serial.print(F("OPEN="));
  Serial.print(consigneOuverture, 2);
  Serial.print(F(";"));
  Serial.print(F("CLOSE="));
  Serial.println(consigneFermeture, 2);
}
String getArg(String cmd) {
  int sp = cmd.indexOf(' ');
  if (sp < 0) return "";
  String arg = cmd.substring(sp + 1);
  arg.trim();
  return arg;
}

int parseSpaceSeparatedFloats(String args, float values[], int maxValues) {
  args.trim();
  int count = 0;

  while (args.length() > 0 && count < maxValues) {
    int sp = args.indexOf(' ');
    String token;

    if (sp < 0) {
      token = args;
      args = "";
    } else {
      token = args.substring(0, sp);
      args = args.substring(sp + 1);
      args.trim();
    }

    token.trim();
    if (token.length() > 0) {
      values[count] = token.toFloat();
      count++;
    }
  }

  return count;
}

void apiSetGripConfig(String cmd) {
  if (isRunning || isPaused) {
    Serial.println(F("ERR;SET_GRIP_CONFIG_NOT_ALLOWED_WHILE_RUNNING"));
    sendStatus();
    return;
  }

  float values[4] = { 0, 0, 1.80, 1.40 };
  int count = parseSpaceSeparatedFloats(getArg(cmd), values, 4);

  if (count != 4) {
    Serial.println(F("ERR;SET_GRIP_CONFIG_FORMAT_USE_SET_GRIP_CONFIG_PREHENSEUR_OBJECT_OPEN_CLOSE"));
    sendStatus();
    return;
  }

  int prehenseurIndex = constrain((int)values[0], 0, NB_PREHENSEURS - 1);
  int objectIndex = constrain((int)values[1], 0, NB_OBJECTS - 1);
  float openValue = constrain(values[2], 0.50, 3.50);
  float closeValue = constrain(values[3], 0.50, 3.50);

  currentPrehenseur = prehenseurIndex;
  currentObject = objectIndex;

  gripSettings[prehenseurIndex][objectIndex].openSetpoint = openValue;
  gripSettings[prehenseurIndex][objectIndex].closeSetpoint = closeValue;

  applyFullConfiguration();

  Serial.println(F("OK;SET_GRIP_CONFIG"));
  sendGripConfig();
  sendStatus();
}

void apiGripOpen() {
  if (isRunning || isPaused) {
    Serial.println(F("ERR;GRIP_TEST_NOT_ALLOWED_WHILE_RUNNING"));
    sendStatus();
    return;
  }

  applyGripperSettingsForObject(currentObject);
  openGripper();
  Serial.println(F("OK;GRIP_OPEN"));
  sendGripConfig();
  sendStatus();
}

void apiGripClose() {
  if (isRunning || isPaused) {
    Serial.println(F("ERR;GRIP_TEST_NOT_ALLOWED_WHILE_RUNNING"));
    sendStatus();
    return;
  }

  applyGripperSettingsForObject(currentObject);
  closeGripper();
  Serial.println(F("OK;GRIP_CLOSE"));
  sendGripConfig();
  sendStatus();
}

void apiGripStop() {
  stopGripper();
  Serial.println(F("OK;GRIP_STOP"));
  sendStatus();
}

void apiPauseToggle() {
  if (!isInitialized) {
    Serial.println(F("ERR;NOT_INITIALIZED"));
    sendStatus();
    return;
  }

  if (isRunning && !isPaused) {
    pauseReturnX = currentX;
    pauseReturnY = currentY;
    pauseReturnZ = currentZ;

    isPaused = true;
    isRunning = false;
    screenState = SCREEN_STATUS;
    if (terminalModeActive) drawTerminalModeScreen();
    else drawStatusScreen();

    pauseMoveInProgress = true;
    moveToReposPosition();
    pauseMoveInProgress = false;

    Serial.println(F("OK;PAUSED"));
    sendStatus();
  } else if (isPaused) {
    isPaused = false;
    isRunning = true;
    screenState = SCREEN_STATUS;
    if (terminalModeActive) drawTerminalModeScreen();
    else drawStatusScreen();

    pauseMoveInProgress = true;
    moveRectangular(pauseReturnX, pauseReturnY, pauseReturnZ, 80, 500, 700, 500);
    pauseMoveInProgress = false;

    Serial.println(F("OK;RESUMED"));
    sendStatus();
  } else {
    Serial.println(F("ERR;PAUSE_NOT_AVAILABLE"));
    sendStatus();
  }
}


void apiGotoPosition(String cmd) {
  if (isRunning || isPaused) {
    Serial.println(F("ERR;GOTO_POSITION_NOT_ALLOWED_WHILE_RUNNING"));
    sendStatus();
    return;
  }

  String args = getArg(cmd);
  args.trim();

  int values[4] = { 0, currentPrehenseur, currentObject, 0 };
  int valueCount = 0;

  while (args.length() > 0 && valueCount < 4) {
    int sp = args.indexOf(' ');
    String token;
    if (sp < 0) {
      token = args;
      args = "";
    } else {
      token = args.substring(0, sp);
      args = args.substring(sp + 1);
      args.trim();
    }
    token.trim();
    if (token.length() > 0) {
      values[valueCount] = token.toInt();
      valueCount++;
    }
  }

  if (valueCount != 3 && valueCount != 4) {
    Serial.println(F("ERR;GOTO_POSITION_FORMAT_USE_GOTO_POSITION_CYCLE_PREHENSEUR_OBJECT_POSITION"));
    sendStatus();
    return;
  }

  int cycleIndex;
  int prehenseurIndex;
  int objectIndex;
  int positionIndex;

  if (valueCount == 4) {
    cycleIndex = constrain(values[0], 0, NB_CYCLES - 1);
    prehenseurIndex = constrain(values[1], 0, NB_PREHENSEURS - 1);
    objectIndex = constrain(values[2], 0, NB_OBJECTS - 1);
    positionIndex = constrain(values[3], 0, NB_POSITIONS - 1);
  } else {
    cycleIndex = constrain(values[0], 0, NB_CYCLES - 1);
    prehenseurIndex = currentPrehenseur;
    objectIndex = constrain(values[1], 0, NB_OBJECTS - 1);
    positionIndex = constrain(values[2], 0, NB_POSITIONS - 1);
  }

  currentCycleType = cycleIndex;
  currentPrehenseur = prehenseurIndex;
  currentObject = objectIndex;
  applyFullConfiguration();

  isRunning = false;
  isPaused = false;
  isInitializing = false;
  currentPiece = 0;
  stopGripper();

  Serial.print(F("OK;GOTO_POSITION_START;CYCLE="));
  Serial.print(cycleIndex);
  Serial.print(F(";PREHENSEUR="));
  Serial.print(prehenseurIndex);
  Serial.print(F(";OBJECT="));
  Serial.print(objectIndex);
  Serial.print(F(";POSITION="));
  Serial.print(positionIndex);
  Serial.print(F(";POSITION_NAME="));
  Serial.println(positionNames[cycleIndex][positionIndex]);

  moveToPrehensionPositionForObject(cycleIndex, objectIndex, positionIndex, 100, 1000, 1000, 1000);

  isInitialized = true;
  screenState = terminalModeActive ? SCREEN_STATUS : SCREEN_READY;
  updateLEDs();

  if (terminalModeActive) drawTerminalModeScreen();
  else refreshScreen();

  Serial.print(F("OK;GOTO_POSITION_DONE;CYCLE="));
  Serial.print(cycleIndex);
  Serial.print(F(";PREHENSEUR="));
  Serial.print(prehenseurIndex);
  Serial.print(F(";OBJECT="));
  Serial.print(objectIndex);
  Serial.print(F(";POSITION="));
  Serial.print(positionIndex);
  Serial.print(F(";POSITION_NAME="));
  Serial.print(positionNames[cycleIndex][positionIndex]);
  Serial.print(F(";X="));
  Serial.print(currentX, 1);
  Serial.print(F(";Y="));
  Serial.print(currentY, 1);
  Serial.print(F(";Z="));
  Serial.println(currentZ, 1);

  sendStatus();
}


void apiGotoXYZ(String cmd) {
  if (isRunning || isPaused) {
    Serial.println(F("ERR;GOTO_XYZ_NOT_ALLOWED_WHILE_RUNNING"));
    sendStatus();
    return;
  }

  float values[3] = { currentX, currentY, currentZ };
  int count = parseSpaceSeparatedFloats(getArg(cmd), values, 3);

  if (count != 3) {
    Serial.println(F("ERR;GOTO_XYZ_FORMAT_USE_GOTO_XYZ_X_Y_Z"));
    sendStatus();
    return;
  }

  float x = values[0];
  float y = values[1];
  float z = values[2];

  isRunning = false;
  isPaused = false;
  isInitializing = false;
  currentPiece = 0;
  stopGripper();

  Serial.print(F("OK;GOTO_XYZ_START;POSITION_NAME=XYZ manuel;X="));
  Serial.print(x, 1);
  Serial.print(F(";Y="));
  Serial.print(y, 1);
  Serial.print(F(";Z="));
  Serial.println(z, 1);

  moveRectangular(x, y, z, 100, 1000, 1000, 1000);

  isInitialized = true;
  screenState = terminalModeActive ? SCREEN_STATUS : SCREEN_READY;
  updateLEDs();

  if (terminalModeActive) drawTerminalModeScreen();
  else refreshScreen();

  Serial.print(F("OK;GOTO_XYZ_DONE;POSITION_NAME=XYZ manuel;X="));
  Serial.print(currentX, 1);
  Serial.print(F(";Y="));
  Serial.print(currentY, 1);
  Serial.print(F(";Z="));
  Serial.println(currentZ, 1);

  // Extra simple response for older terminal compatibility.
  Serial.print(F("XYZ_OK "));
  Serial.print(currentX, 1);
  Serial.print(F(" "));
  Serial.print(currentY, 1);
  Serial.print(F(" "));
  Serial.println(currentZ, 1);

  sendStatus();
}



// =====================================================
// TAB 5 - GRIPPER TEST API
// =====================================================
void apiGotoCycle1EntreeTestHeight() {
  if (isRunning || isPaused) {
    Serial.println(F("ERR;GRIPPER_TEST_NOT_ALLOWED_WHILE_RUNNING"));
    sendStatus();
    return;
  }

  currentCycleType = 0;
  applyFullConfiguration();
  stopGripper();

  Point3D target = basePositions[0][POS_convoyeurEntree];
  target.z = 20.0;

  Serial.println(F("OK;GRIPPER_GOTO_C1_ENTREE_TEST_HEIGHT_START"));
  moveRectangular(target.x, target.y, target.z, 100, 1000, 1000, 1000);

  isInitialized = true;
  screenState = terminalModeActive ? SCREEN_STATUS : SCREEN_READY;
  updateLEDs();

  if (terminalModeActive) drawTerminalModeScreen();
  else refreshScreen();

  Serial.print(F("OK;GRIPPER_GOTO_C1_ENTREE_TEST_HEIGHT_DONE;POSITION_NAME=C1 Entree Test;X="));
  Serial.print(currentX, 1);
  Serial.print(F(";Y="));
  Serial.print(currentY, 1);
  Serial.print(F(";Z="));
  Serial.println(currentZ, 1);
  sendStatus();
}

void apiGripperAutoTest(String cmd) {
  if (isRunning || isPaused) {
    Serial.println(F("ERR;GRIPPER_TEST_NOT_ALLOWED_WHILE_RUNNING"));
    sendStatus();
    return;
  }

  int objectIndex = constrain(getArg(cmd).toInt(), 0, NB_OBJECTS - 1);
  currentCycleType = 0;
  currentObject = objectIndex;
  prepareObjectCycle(objectIndex);

  unsigned long tUp;
  unsigned long tXY;
  unsigned long tDown;
  getCycleTimes(tUp, tXY, tDown);

  isRunning = false;
  isPaused = false;
  isInitializing = false;
  currentPiece = 0;

  Serial.print(F("OK;GRIPPER_AUTO_TEST_START;OBJECT="));
  Serial.print(objectIndex);
  Serial.print(F(";OBJECT_NAME="));
  Serial.println(getObjectNameByIndex(objectIndex));
  sendGripConfig();

  // Same pickup idea as the real cycle: open first, approach the real
  // object-specific entrance position, then let the PID close long enough
  // before any lift happens. Do NOT stop after closing; the gripper must
  // keep holding while the object is lifted.
  openGripper();
  smartWait(1000);

  moveToPrehensionPositionForObject(0, objectIndex, POS_convoyeurEntree, 100, tUp, tXY, tDown);

  closeGripper();
  smartWait(3500);

  moveRectangular(currentX, currentY, 80.0, 100, tUp, tXY, tDown);
  smartWait(1000);

  moveToPrehensionPositionForObject(0, objectIndex, POS_convoyeurEntree, 100, tUp, tXY, tDown);
  smartWait(800);

  // Same release idea as the real cycle: open fully, then stop the motor.
  openGripper();
  smartWait(3500);
  stopGripper();

  isInitialized = true;
  screenState = terminalModeActive ? SCREEN_STATUS : SCREEN_READY;
  updateLEDs();

  if (terminalModeActive) drawTerminalModeScreen();
  else refreshScreen();

  Serial.print(F("OK;GRIPPER_AUTO_TEST_DONE;OBJECT="));
  Serial.print(objectIndex);
  Serial.print(F(";OBJECT_NAME="));
  Serial.print(getObjectNameByIndex(objectIndex));
  Serial.print(F(";X="));
  Serial.print(currentX, 1);
  Serial.print(F(";Y="));
  Serial.print(currentY, 1);
  Serial.print(F(";Z="));
  Serial.println(currentZ, 1);
  sendStatus();
}

void handleApiCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  Serial.print(F("DBG;CMD_RECEIVED="));
  Serial.println(cmd);

  enterTerminalMode();

  String upper = cmd;
  upper.toUpperCase();

  if (upper == "TERMINAL_ON") {
    Serial.println(F("OK;TERMINAL_ON"));
    sendStatus();
  } else if (upper == "TERMINAL_OFF") {
    terminalButtonTestMode = false;
    terminalLedTestMode = false;
    Serial.println(F("OK;TERMINAL_OFF"));
    exitTerminalMode();
    sendStatus();
  } else if (upper == "PING") {
    Serial.println(F("PONG"));
    sendStatus();
  } else if (upper == "GET_STATUS") {
    sendStatus();
  } else if (upper == "GET_CONFIG") {
    sendConfig();
  } else if (upper == "GET_GRIP_CONFIG") {
    sendGripConfig();
  } else if (upper.startsWith("SET_GRIP_CONFIG")) {
    apiSetGripConfig(cmd);
  } else if (upper == "GRIP_OPEN" || upper == "GRIPPER_OPEN") {
    apiGripOpen();
  } else if (upper == "GRIP_CLOSE" || upper == "GRIPPER_CLOSE") {
    apiGripClose();
  } else if (upper == "GRIP_STOP" || upper == "GRIPPER_STOP") {
    apiGripStop();
  } else if (upper == "GRIPPER_GOTO_C1_ENTREE_TEST_HEIGHT") {
    apiGotoCycle1EntreeTestHeight();
  } else if (upper.startsWith("GRIPPER_AUTO_TEST")) {
    apiGripperAutoTest(cmd);
  } else if (upper.startsWith("SET_CYCLE")) {
    int value = constrain(getArg(cmd).toInt(), 0, NB_CYCLES - 1);
    if (value != currentCycleType) {
      currentCycleType = value;
      terminalConfigChanged();
    }
    Serial.println(F("OK;SET_CYCLE"));
    sendStatus();
  } else if (upper.startsWith("SET_PREHENSEUR")) {
    int value = constrain(getArg(cmd).toInt(), 0, NB_PREHENSEURS - 1);
    if (value != currentPrehenseur) {
      currentPrehenseur = value;
      terminalConfigChanged();
    }
    applyGripperSettingsForObject(currentObject);
    Serial.println(F("OK;SET_PREHENSEUR"));
    sendGripConfig();
    sendStatus();
  } else if (upper.startsWith("SET_OBJECT")) {
    int value = constrain(getArg(cmd).toInt(), 0, NB_OBJECTS - 1);
    if (value != currentObject) {
      currentObject = value;
      terminalConfigChanged();
    }
    applyGripperSettingsForObject(currentObject);
    Serial.println(F("OK;SET_OBJECT"));
    sendGripConfig();
    sendStatus();
  } else if (upper.startsWith("SET_MODE")) {
    int value = constrain(getArg(cmd).toInt(), 0, 2);
    if (value != currentMode) {
      currentMode = value;
      terminalConfigChanged();
    }
    Serial.println(F("OK;SET_MODE"));
    sendStatus();
  } else if (upper.startsWith("SET_PIECES")) {
    int pieces = constrain(getArg(cmd).toInt(), 1, 5);
    int newCombine = pieces - 1;
    if (newCombine != currentCombine) {
      currentCombine = newCombine;
      terminalConfigChanged();
    }
    Serial.println(F("OK;SET_PIECES"));
    sendStatus();
  } else if (upper.startsWith("SET_SPEED")) {
    float value = constrain(getArg(cmd).toFloat(), 0.50, 3.00);
    if (abs(value - executionSpeedFactor) > 0.001) {
      executionSpeedFactor = value;
          terminalConfigChanged();
    }
    Serial.println(F("OK;SET_SPEED"));
    sendStatus();
  }

  else if (upper == "GO_REPOS") {
    Serial.println(F("OK;GO_REPOS_START"));

    isRunning = false;
    isPaused = false;
    isInitializing = false;

    stopGripper();

    if (manualServoTestDirty) {
      returnToReposFromCurrentServoAngles();
      manualServoTestDirty = false;
    } else {
      moveToReposPosition();
    }

    isInitialized = true;
    screenState = terminalModeActive ? SCREEN_STATUS : SCREEN_READY;
    currentPiece = 0;
    updateLEDs();

    if (terminalModeActive) drawTerminalModeScreen();
    else refreshScreen();

    Serial.println(F("OK;GO_REPOS_DONE"));
    sendStatus();
  } else if (upper.startsWith("SERVO_SET")) {
    if (isRunning || isPaused) {
      Serial.println(F("ERR;SERVO_TEST_NOT_ALLOWED_WHILE_RUNNING"));
      sendStatus();
      return;
    }

    String args = getArg(cmd);
    args.trim();

    int sp = args.indexOf(' ');
    if (sp < 0) {
      Serial.println(F("ERR;SERVO_SET_FORMAT_USE_SERVO_SET_A_90"));
      sendStatus();
      return;
    }

    String servoName = args.substring(0, sp);
    servoName.trim();
    servoName.toUpperCase();

    float value = args.substring(sp + 1).toFloat();
    value = constrain(value, 0.0, 180.0);

    if (servoName == "A") currentCmdA = value;
    else if (servoName == "B") currentCmdB = value;
    else if (servoName == "Z") currentCmdZ = value;
    else {
      Serial.println(F("ERR;UNKNOWN_SERVO_USE_A_B_Z"));
      sendStatus();
      return;
    }

    writeServosMicroseconds(currentCmdA, currentCmdB, currentCmdZ);
    manualServoTestDirty = true;
    isRunning = false;
    isPaused = false;
    updateLEDs();

    Serial.print(F("OK;SERVO_SET;SERVO="));
    Serial.print(servoName);
    Serial.print(F(";VALUE="));
    Serial.println(value, 1);
    sendStatus();
  } else if (upper.startsWith("SERVO_STEP")) {
    if (isRunning || isPaused) {
      Serial.println(F("ERR;SERVO_TEST_NOT_ALLOWED_WHILE_RUNNING"));
      sendStatus();
      return;
    }

    String args = getArg(cmd);
    args.trim();

    int sp = args.indexOf(' ');
    if (sp < 0) {
      Serial.println(F("ERR;SERVO_STEP_FORMAT_USE_SERVO_STEP_A_2"));
      sendStatus();
      return;
    }

    String servoName = args.substring(0, sp);
    servoName.trim();
    servoName.toUpperCase();

    float delta = args.substring(sp + 1).toFloat();

    if (servoName == "A") currentCmdA = constrain(currentCmdA + delta, 0.0, 180.0);
    else if (servoName == "B") currentCmdB = constrain(currentCmdB + delta, 0.0, 180.0);
    else if (servoName == "Z") currentCmdZ = constrain(currentCmdZ + delta, 0.0, 180.0);
    else {
      Serial.println(F("ERR;UNKNOWN_SERVO_USE_A_B_Z"));
      sendStatus();
      return;
    }

    writeServosMicroseconds(currentCmdA, currentCmdB, currentCmdZ);
    manualServoTestDirty = true;
    isRunning = false;
    isPaused = false;
    updateLEDs();

    Serial.print(F("OK;SERVO_STEP;SERVO="));
    Serial.print(servoName);
    Serial.print(F(";A="));
    Serial.print(currentCmdA, 1);
    Serial.print(F(";B="));
    Serial.print(currentCmdB, 1);
    Serial.print(F(";Z="));
    Serial.println(currentCmdZ, 1);
    sendStatus();
  }


  else if (upper.startsWith("GOTO_XYZ")) {
    apiGotoXYZ(cmd);
  }

  else if (upper.startsWith("GOTO_POSITION")) {
    apiGotoPosition(cmd);
  }

  else if (upper == "BUTTON_TEST_ON") {
    terminalButtonTestMode = true;
    isRunning = false;
    isPaused = false;
    pauseInterruptRequest = false;
    Serial.println(F("OK;BUTTON_TEST_ON"));
    sendStatus();
  } else if (upper == "BUTTON_TEST_OFF") {
    terminalButtonTestMode = false;
    Serial.println(F("OK;BUTTON_TEST_OFF"));
    sendStatus();
  } else if (upper.startsWith("LED_SET")) {
    String args = getArg(cmd);
    args.trim();
    int sp = args.indexOf(' ');
    if (sp < 0) {
      Serial.println(F("ERR;LED_SET_FORMAT_USE_LED_SET_INIT_1"));
      sendStatus();
      return;
    }
    String ledName = args.substring(0, sp);
    int value = args.substring(sp + 1).toInt();
    setLedByName(ledName, value != 0);
  } else if (upper == "LED_AUTO") {
    terminalLedTestMode = false;
    updateLEDs();
    Serial.println(F("OK;LED_AUTO"));
    sendStatus();
  } else if (upper.startsWith("LED_PULSE")) {
    pulseLedByName(getArg(cmd));
  } else if (upper == "TEST_INIT_LED") {
    Serial.println(F("OK;TEST_INIT_LED_START"));
    digitalWrite(LED_INIT, HIGH);
    delay(500);
    digitalWrite(LED_INIT, LOW);
    delay(250);
    digitalWrite(LED_INIT, HIGH);
    delay(500);
    updateLEDs();
    Serial.println(F("OK;TEST_INIT_LED_DONE"));
    sendStatus();
  } else if (upper == "FORCE_INIT_TEST") {
    Serial.println(F("OK;FORCE_INIT_TEST"));
    isInitialized = true;
    isInitializing = false;
    isRunning = false;
    isPaused = false;
    currentPiece = 0;
    screenState = SCREEN_READY;
    manualServoTestDirty = false;
    updateLEDs();
    if (terminalModeActive) drawTerminalModeScreen();
    sendStatus();
  } else if (upper == "INITIALIZE") {
    Serial.println(F("OK;INITIALIZE_START"));
    sendStatus();
    ihmInitializeAction();
    Serial.println(F("OK;INITIALIZE_DONE"));
    sendStatus();
  } else if (upper == "RUN_CYCLE") {
    ihmStartAction();
  } else if (upper == "PAUSE_TOGGLE") {
    ihmPauseToggleAction();
  } else {
    Serial.print(F("ERR;UNKNOWN_COMMAND;CMD="));
    Serial.println(cmd);
    sendStatus();
  }
}
void handleApiSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (apiBuffer.length() > 0) {
        handleApiCommand(apiBuffer);
        apiBuffer = "";
      }
    } else {
      apiBuffer += c;
      if (apiBuffer.length() > 80) {
        apiBuffer = "";
        Serial.println(F("ERR;COMMAND_TOO_LONG"));
      }
    }
  }
}

