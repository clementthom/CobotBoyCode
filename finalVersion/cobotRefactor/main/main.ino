// =====================================================
// main.ino — Top-level orchestration
// Includes all modules, owns the runtime state globals,
// and contains setup() / loop().
// All subsystem logic lives in the .cpp modules.
// =====================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
#include <AutoPID.h>
#include <MsTimer2.h>

#include "common.h"
#include "cinematics.h"
#include "trajectories.h"
#include "prehension.h"
#include "maintenance.h"
#include "userInterfaceHardware.h"
#include "userInterfaceScreen.h"
#include "eeprom.h"

// =====================================================
// [→ main.ino]  RUNTIME STATE GLOBALS
// Flags and config selectors owned by the top level.
// All modules read these via extern (declared in common.h).
// =====================================================
bool isInitialized = false;
bool isInitializing = false;
bool isRunning = false;
bool isPaused = false;

volatile bool pauseInterruptRequest = false;

bool terminalModeActive = false;
bool manualServoTestDirty = false;
bool terminalButtonTestMode = false;
bool terminalLedTestMode = false;
bool ledTestInit = false;
bool ledTestRun = false;
bool ledTestPause = false;
bool ledTestPower = true;
unsigned long lastTerminalContact = 0;

int menuIndex = 0;
int currentCycleType = 0;
int currentPrehenseur = 0;
int currentObject = 0;
int currentMode = 1;
int currentCombine = 0;
int currentPiece = 0;

unsigned long bootTime = 0;

// =====================================================
// [→ main.ino]  INITIALIZATION ACTION
// =====================================================
void performInitialization() {
  isRunning = false;
  isPaused = false;
  pauseMoveInProgress = false;

  applyFullConfiguration();

  drawInfoMessage("Init...", "Retour au repos");

  moveToReposPosition();

  closeGripper();
  smartWait(700);
  stopGripper();

  isInitialized = true;
  currentPiece = 0;
  screenState = SCREEN_READY;
  menuIndex = 0;
  refreshScreen();
}

// =====================================================
// PYTHON TERMINAL ONLY
// The old blocking Arduino Serial Monitor maintenance console was removed.
// Maintenance is now handled by the Python terminal through handleApiSerial()
// and handleApiCommand(). The Arduino remains the robot controller/API receiver.
// =====================================================

// =====================================================
// SHARED IHM ACTIONS
// These functions are used by BOTH:
// 1) the physical Arduino buttons
// 2) the Python terminal buttons over Serial
// This keeps INIT / START / PAUSE exactly the same.
// =====================================================

// =====================================================
// [→ main.ino]  SHARED IHM ACTIONS (physical buttons + Python terminal)
// =====================================================
void ihmInitializeAction() {
  Serial.println(F("DBG;IHM_INITIALIZE_ACTION_ENTER"));

  // Shared INIT action for BOTH physical INIT and Python terminal INIT.
  // It immediately puts the robot in INITIALIZING state so PLAY cannot
  // accidentally become the first real arming step.
  isInitializing = true;
  isInitialized = false;
  isRunning = false;
  isPaused = false;
  pauseInterruptRequest = false;
  pauseMoveInProgress = false;
  currentPiece = 0;

  updateLEDs();
  if (terminalModeActive) drawTerminalModeScreen();
  else drawInfoMessage("Init...", "Retour au repos");
  sendStatus();

  stopGripper();

  if (!servoA.attached()) servoA.attach(PIN_SERVO_A);
  if (!servoB.attached()) servoB.attach(PIN_SERVO_B);
  if (!servoZ.attached()) servoZ.attach(PIN_SERVO_Z);

  applyFullConfiguration();

  bool initOK = true;

  if (manualServoTestDirty) {
    initOK = returnToReposFromCurrentServoAngles();
    manualServoTestDirty = false;
  } else {
    initOK = initializeFromCartesianPosition(initX, initY, initZ);
    if (initOK) {
      moveToReposPosition();
    }
  }

  if (!initOK) {
    Serial.println(F("ERR;INIT_REPOS_NOT_REACHABLE"));
    isInitializing = false;
    isInitialized = false;
    isRunning = false;
    isPaused = false;
    updateLEDs();
    sendStatus();
    return;
  }

  closeGripper();
  smartWait(3500);
  stopGripper();

  isInitializing = false;
  isInitialized = true;
  isRunning = false;
  isPaused = false;
  currentPiece = 0;

  screenState = terminalModeActive ? SCREEN_STATUS : SCREEN_READY;
  menuIndex = 0;

  updateLEDs();
  if (terminalModeActive) drawTerminalModeScreen();
  else refreshScreen();

  Serial.println(F("DBG;IHM_INITIALIZE_ACTION_EXIT"));
  sendStatus();
}

void ihmStartAction() {
  Serial.println(F("DBG;IHM_START_ACTION_ENTER"));
  if (isInitializing) {
    Serial.println(F("ERR;INITIALIZATION_IN_PROGRESS"));
    sendStatus();
    return;
  }
  if (!isInitialized) {
    drawInfoMessage("ERREUR", "Initialiser avant");
    smartWait(1200);
    refreshScreen();
    Serial.println(F("ERR;NOT_INITIALIZED"));
    sendStatus();
    return;
  }

  if (isPaused) {
    Serial.println(F("ERR;PAUSED_USE_PAUSE_TOGGLE"));
    sendStatus();
    return;
  }

  if (isRunning) {
    Serial.println(F("ERR;ALREADY_RUNNING"));
    sendStatus();
    return;
  }

  if (manualServoTestDirty) {
    Serial.println(F("DBG;RETURNING_TO_REPOS_AFTER_SERVO_TEST"));
    if (!returnToReposFromCurrentServoAngles()) {
      isInitialized = false;
      updateLEDs();
      Serial.println(F("ERR;CANNOT_START_REPOS_NOT_REACHABLE"));
      sendStatus();
      return;
    }
    manualServoTestDirty = false;
    isInitialized = true;
  }

  cycleStartTime = millis();

  applyFullConfiguration();

  currentPiece = 0;
  isRunning = true;
  isPaused = false;
  screenState = SCREEN_STATUS;
  if (terminalModeActive) drawTerminalModeScreen();
  else drawStatusScreen();
  updateLEDs();

  Serial.println(F("OK;RUN_CYCLE_START"));
  sendStatus();
}

void ihmPauseToggleAction() {
  Serial.println(F("DBG;IHM_PAUSE_TOGGLE_ACTION_ENTER"));
  if (isInitializing) {
    Serial.println(F("ERR;INITIALIZATION_IN_PROGRESS"));
    sendStatus();
    return;
  }
  pauseInterruptRequest = true;
  handlePauseInterrupt();
  updateLEDs();
  if (terminalModeActive) drawTerminalModeScreen();
  sendStatus();
}



// =====================================================
// [→ main.ino]  SYSTEM YIELD & SMART WAIT
// =====================================================
void handlePauseInterrupt() {
  if (!pauseInterruptRequest) return;

  pauseInterruptRequest = false;

  if (terminalButtonTestMode) {
    sendButtonTestEvent("PAUSE");
    return;
  }

  if (!isInitialized) return;

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

    if (terminalModeActive) drawTerminalModeScreen();
    else drawStatusScreen();
  } else if (isPaused) {
    isPaused = false;
    isRunning = true;
    screenState = SCREEN_STATUS;
    if (terminalModeActive) drawTerminalModeScreen();
    else drawStatusScreen();

    pauseMoveInProgress = true;
    moveRectangular(pauseReturnX, pauseReturnY, pauseReturnZ, 80, 500, 700, 500);
    pauseMoveInProgress = false;
  }
}

void systemYield() {
  if (tick10ms) {
    tick10ms = false;
    updateGripperPID();
  }

  handleApiSerial();
  handlePauseInterrupt();
  handleButtons();
  handleEncoder();
  updateLEDs();

  if (!isRunning && screenState == SCREEN_STATUS && (millis() - lastStatusUpdate > statusRefreshDelay)) {
    lastStatusUpdate = millis();
    if (terminalModeActive) drawTerminalModeScreen();
    else drawStatusScreen();
  }

  while (isPaused && !pauseMoveInProgress) {
    if (tick10ms) {
      tick10ms = false;
      updateGripperPID();
    }

    handleApiSerial();
    handlePauseInterrupt();
    handleButtons();
    handleEncoder();
    updateLEDs();
    delay(5);
  }
}

void smartWait(unsigned long ms) {
  unsigned long start = millis();

  while (millis() - start < ms) {
    systemYield();
    delay(5);
  }
}


// =====================================================
// [→ main.ino]  SETUP & LOOP
// =====================================================
void setup() {
  delay(200);

  Serial.begin(9600);
  Serial.setTimeout(50);


  pinMode(BTN_PAUSE, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_INIT, INPUT_PULLUP);

  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  lastEncCLK = digitalRead(ENC_CLK);

  pinMode(LED_RUN, OUTPUT);
  pinMode(LED_PAUSE, OUTPUT);
  pinMode(LED_POWER, OUTPUT);
  pinMode(LED_INIT, OUTPUT);

  pinMode(PIN_GRIP_INA, OUTPUT);
  pinMode(PIN_GRIP_INB, OUTPUT);
  pinMode(PIN_GRIP_PWM, OUTPUT);

  stopGripper();

  servoA.attach(PIN_SERVO_A);
  servoB.attach(PIN_SERVO_B);
  servoZ.attach(PIN_SERVO_Z);

  pidGripper.setTimeStep(10);

  MsTimer2::set(10, timerISR);
  MsTimer2::start();

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    while (true)
      ;
  }

  setDefaultRuntimeConfig();
  applyFullConfiguration();

  initializeFromCartesianPosition(initX, initY, initZ);


  attachInterrupt(digitalPinToInterrupt(BTN_PAUSE), pauseISR, FALLING);

  bootTime = millis();

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 20);
  display.println(F("DEMARRAGE"));
  display.display();

  delay(800);

  refreshScreen();
  updateLEDs();
}

void loop() {
  systemYield();
  updateRobotCycle();
  updateLEDs();
}
