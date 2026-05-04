#include <Arduino.h>

#ifndef MAINTENANCE_H
#define MAINTENANCE_H

#include "common.h"

// =====================================================
// FUNCTION PROTOTYPES
// =====================================================

// --- Status & config senders (Arduino → Python) ---
void sendStatus();
void sendConfig();
void sendGripConfig();

// --- Command argument parsers ---
String getArg(String cmd);
int    parseSpaceSeparatedFloats(String args, float values[], int maxValues);

// --- API command handlers (Python → Arduino) ---
void apiSetGripConfig(String cmd);
void apiGripOpen();
void apiGripClose();
void apiGripStop();
void apiPauseToggle();
void apiGotoPosition(String cmd);
void apiGotoXYZ(String cmd);
void apiGotoCycle1EntreeTestHeight();
void apiGripperAutoTest(String cmd);

// --- Serial dispatch ---
void handleApiCommand(String cmd);
void handleApiSerial();

#endif // MAINTENANCE_H
