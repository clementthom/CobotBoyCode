#include <Arduino.h>
#include "userInterfaceHardware.h"
#include "userInterfaceScreen.h"
#include "cinematics.h"
#include "maintenance.h"

// ====================================================
// [userInterfaceHardware.cpp]  GLOBALS — debounce, encoder, timing state
// ====================================================

volatile bool tick10ms = false;
volatile bool pauseInterruptRequest = false;

bool previousButtonState[3] = { HIGH, HIGH, HIGH };
unsigned long lastDebounceTime[3] = { 0, 0, 0 };
const unsigned long debounceDelay = 50;

int lastEncCLK = HIGH;
unsigned long lastEncoderMoveTime = 0;
unsigned long lastEncoderButtonTime = 0;
const unsigned long encoderMoveDelay = 80;
const unsigned long encoderButtonDelay = 300;


// ====================================================
// [userInterfaceHardware.cpp]  INTERRUPT SERVICE ROUTINES
// ====================================================

void pauseISR() {
  pauseInterruptRequest = true;
}
void timerISR() {
  tick10ms = true;
}

// ====================================================
// [userInterfaceHardware.cpp]  BUTTON LOGIC
// ====================================================

int getButtonIndex(int pin) {
  if (pin == BTN_PAUSE) return 0;
  if (pin == BTN_START) return 1;
  if (pin == BTN_INIT) return 2;
  return -1;
}
bool buttonPressed(int pin) {
  bool currentState = digitalRead(pin);
  bool pressed = false;
  int idx = getButtonIndex(pin);

  if (idx == -1) return false;

  if (currentState == LOW && previousButtonState[idx] == HIGH) {
    if (millis() - lastDebounceTime[idx] > debounceDelay) {
      pressed = true;
      lastDebounceTime[idx] = millis();
    }
  }

  previousButtonState[idx] = currentState;
  return pressed;
}

// ====================================================
// [userInterfaceHardware.cpp]  LEDs
// ====================================================

void updateLEDs() {
  if (terminalLedTestMode) {
    digitalWrite(LED_INIT, ledTestInit ? HIGH : LOW);
    digitalWrite(LED_RUN, ledTestRun ? HIGH : LOW);
    digitalWrite(LED_PAUSE, ledTestPause ? HIGH : LOW);
    digitalWrite(LED_POWER, ledTestPower ? HIGH : LOW);
    return;
  }

  digitalWrite(LED_RUN, (isRunning && !isPaused) ? HIGH : LOW);
  digitalWrite(LED_PAUSE, isPaused ? HIGH : LOW);
  digitalWrite(LED_POWER, HIGH);

  bool initStateActive = (isInitializing || isInitialized) && !isRunning && !isPaused;
  digitalWrite(LED_INIT, initStateActive ? HIGH : LOW);
}

// ====================================================
// [userInterfaceHardware.cpp]  LED & BUTTON TEST HELPERS
// ====================================================

void sendButtonTestEvent(const char* buttonName) {
  Serial.print(F("EVENT;TYPE=BUTTON;NAME="));
  Serial.print(buttonName);
  Serial.println(F(";STATUS=RECEIVED"));
  sendStatus();
}
void setLedByName(String ledName, bool on) {
  ledName.trim();
  ledName.toUpperCase();

  int pin = -1;
  if (ledName == "INIT") pin = LED_INIT;
  else if (ledName == "RUN") pin = LED_RUN;
  else if (ledName == "PAUSE") pin = LED_PAUSE;
  else if (ledName == "POWER") pin = LED_POWER;

  if (pin < 0) {
    Serial.println(F("ERR;UNKNOWN_LED_USE_INIT_RUN_PAUSE_POWER"));
    sendStatus();
    return;
  }

  terminalLedTestMode = true;
  if (ledName == "INIT") ledTestInit = on;
  else if (ledName == "RUN") ledTestRun = on;
  else if (ledName == "PAUSE") ledTestPause = on;
  else if (ledName == "POWER") ledTestPower = on;
  updateLEDs();
  Serial.print(F("OK;LED_SET;LED="));
  Serial.print(ledName);
  Serial.print(F(";VALUE="));
  Serial.println(on ? 1 : 0);
  sendStatus();
}
void pulseLedByName(String ledName) {
  ledName.trim();
  ledName.toUpperCase();

  int pin = -1;
  if (ledName == "INIT") pin = LED_INIT;
  else if (ledName == "RUN") pin = LED_RUN;
  else if (ledName == "PAUSE") pin = LED_PAUSE;
  else if (ledName == "POWER") pin = LED_POWER;

  if (pin < 0) {
    Serial.println(F("ERR;UNKNOWN_LED_USE_INIT_RUN_PAUSE_POWER"));
    sendStatus();
    return;
  }

  bool previousLedTestMode = terminalLedTestMode;
  terminalLedTestMode = false;
  Serial.print(F("OK;LED_PULSE_START;LED="));
  Serial.println(ledName);
  digitalWrite(pin, HIGH);
  delay(400);
  digitalWrite(pin, LOW);
  delay(150);
  digitalWrite(pin, HIGH);
  delay(400);
  terminalLedTestMode = previousLedTestMode;
  updateLEDs();
  Serial.print(F("OK;LED_PULSE_DONE;LED="));
  Serial.println(ledName);
  sendStatus();
}

// ====================================================
// [userInterfaceHardware.cpp]  ENCODER & MENU NAVIGATION
// ====================================================

int getMenuCount(ScreenState s) {
  if (s == SCREEN_HOME) return 5;
  if (s == SCREEN_INIT_MENU) return 2;
  if (s == SCREEN_CONFIG) return 6;
  if (s == SCREEN_CYCLE_MENU) return 3;
  if (s == SCREEN_PREHENSEUR_MENU) return 3;
  if (s == SCREEN_OBJECT_MENU) return 4;
  if (s == SCREEN_MODE_MENU) return 4;
  if (s == SCREEN_COMBINE_MENU) return 6;
  return 0;
}
void handleEncoder() {
  int currentCLK = digitalRead(ENC_CLK);

  if (currentCLK != lastEncCLK && currentCLK == LOW) {
    if (millis() - lastEncoderMoveTime > encoderMoveDelay) {
      int count = getMenuCount(screenState);

      if (count > 0 && !isRunning) {
        if (digitalRead(ENC_DT) != currentCLK) {
          menuIndex = (menuIndex + 1) % count;
        } else {
          menuIndex--;
          if (menuIndex < 0) menuIndex = count - 1;
        }

        refreshScreen();
      }

      lastEncoderMoveTime = millis();
    }
  }

  lastEncCLK = currentCLK;

  if (digitalRead(ENC_SW) == LOW) {
    if (millis() - lastEncoderButtonTime > encoderButtonDelay) {
      if (!isRunning) executeCurrentScreen();
      lastEncoderButtonTime = millis();
    }
  }
}
void executeCurrentScreen() {
  switch (screenState) {
    case SCREEN_HOME:
      if (menuIndex == 0) screenState = SCREEN_INIT_MENU;
      else if (menuIndex == 1) screenState = SCREEN_CONFIG;
      else if (menuIndex == 4) screenState = SCREEN_STATUS;
      else if (menuIndex == 2) {
        drawInfoMessage("REGLAGES", "Utiliser Config", "pour le cycle");
        smartWait(1000);
        screenState = SCREEN_HOME;
      } else if (menuIndex == 3) {
        drawInfoMessage("INFO", "Encodeur: nav/OK", "Init avant Start");
        smartWait(1200);
        screenState = SCREEN_HOME;
      }
      menuIndex = 0;
      break;

    case SCREEN_INIT_MENU:
      if (menuIndex == 0) {
        performInitialization();
      } else {
        screenState = SCREEN_HOME;
        menuIndex = 0;
      }
      break;

    case SCREEN_CONFIG:
      if (menuIndex == 0) screenState = SCREEN_CYCLE_MENU;
      else if (menuIndex == 1) screenState = SCREEN_PREHENSEUR_MENU;
      else if (menuIndex == 2) screenState = SCREEN_OBJECT_MENU;
      else if (menuIndex == 3) screenState = SCREEN_MODE_MENU;
      else if (menuIndex == 4) screenState = SCREEN_COMBINE_MENU;
      else screenState = SCREEN_HOME;
      menuIndex = 0;
      break;

    case SCREEN_CYCLE_MENU:
      if (menuIndex < 2) {
        currentCycleType = menuIndex;
        configChanged();
      }
      screenState = SCREEN_CONFIG;
      menuIndex = 0;
      break;

    case SCREEN_PREHENSEUR_MENU:
      if (menuIndex < 2) {
        currentPrehenseur = menuIndex;
        configChanged();
      }
      screenState = SCREEN_CONFIG;
      menuIndex = 1;
      break;

    case SCREEN_OBJECT_MENU:
      if (menuIndex < 3) {
        currentObject = menuIndex;
        configChanged();
      }
      screenState = SCREEN_CONFIG;
      menuIndex = 2;
      break;

    case SCREEN_MODE_MENU:
      if (menuIndex < 3) {
        currentMode = menuIndex;
        configChanged();
      }
      screenState = SCREEN_CONFIG;
      menuIndex = 3;
      break;

    case SCREEN_COMBINE_MENU:
      if (menuIndex < 5) {
        currentCombine = menuIndex;
        configChanged();
      }
      screenState = SCREEN_CONFIG;
      menuIndex = 4;
      break;

    case SCREEN_STATUS:
    case SCREEN_READY:
      screenState = SCREEN_HOME;
      menuIndex = 0;
      break;

    default:
      break;
  }

  refreshScreen();
}

// ====================================================
// [userInterfaceHardware.cpp]  HANDLE BUTTONS (main poll loop)
// ====================================================

void handleButtons() {
  if (millis() - bootTime < bootIgnoreButtonsMs) return;

  if (buttonPressed(BTN_INIT)) {
    if (terminalButtonTestMode) sendButtonTestEvent("INIT");
    else ihmInitializeAction();
  }

  if (buttonPressed(BTN_START)) {
    if (terminalButtonTestMode) sendButtonTestEvent("START");
    else ihmStartAction();
  }

  // BTN_PAUSE normally uses an interrupt, but polling here lets the
  // maintenance terminal confirm the physical pause button during tests.
  if (terminalButtonTestMode && buttonPressed(BTN_PAUSE)) {
    sendButtonTestEvent("PAUSE");
  }
}

// ====================================================
// [userInterfaceHardware.cpp]  PAUSE INTERRUPT HANDLER
// ====================================================

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

