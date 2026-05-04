#ifndef USERINTERFACEHARDWARE_H
#define USERINTERFACEHARDWARE_H

#include "common.h"

// =====================================================
// EXTERN GLOBALS — defined in userInterfaceHardware.cpp
// =====================================================
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
