#include <Arduino.h>

#ifndef USERINTERFACEHARDWARE_H
#define USERINTERFACEHARDWARE_H

#include "common.h"

// =====================================================
// PIN DEFINITIONS (verbatim from original)
// =====================================================
const int PIN_SERVO_A = 5;
const int PIN_SERVO_B = 9;
const int PIN_SERVO_Z = 11;

const int PIN_GRIP_INA = 6;
const int PIN_GRIP_INB = 7;
const int PIN_GRIP_PWM = 3;

const int CURRENT_CLOSE_PIN = A4;
const int CURRENT_OPEN_PIN = A5;


const int BTN_PAUSE = 18;
const int BTN_START = 22;
const int BTN_INIT = 24;

const int ENC_CLK = 40;
const int ENC_DT = 42;
const int ENC_SW = 46;

const int LED_RUN = 30;
const int LED_PAUSE = 32;
const int LED_POWER = 34;
const int LED_INIT = 36;



// =====================================================
// EXTERN GLOBALS — defined in userInterfaceHardware.cpp
// =====================================================
extern volatile bool tick10ms;
extern volatile bool pauseInterruptRequest;

extern bool previousButtonState[3];
extern unsigned long lastDebounceTime[3];
extern int  lastEncCLK;
extern unsigned long lastEncoderMoveTime;
extern unsigned long lastEncoderButtonTime;
extern unsigned long lastStatusUpdate;

// =====================================================
// FUNCTION PROTOTYPES
// =====================================================

// --- ISRs ---
void pauseISR();
void timerISR();

// --- Button logic ---
int  getButtonIndex(int pin);
bool buttonPressed(int pin);
void handleButtons();
void handlePauseInterrupt();

// --- Encoder ---
void handleEncoder();
void executeCurrentScreen();

// --- LEDs ---
void updateLEDs();
void setLedByName(String ledName, bool on);
void pulseLedByName(String ledName);

// --- Test helpers (used by maintenance terminal) ---
void sendButtonTestEvent(const char* buttonName);

#endif // USERINTERFACEHARDWARE_H
