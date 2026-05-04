#ifndef USERINTERFACESCREEN_H
#define USERINTERFACESCREEN_H

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "common.h"

// =====================================================
// OLED HARDWARE CONFIG
// =====================================================
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT    64
#define OLED_RESET       -1
#define SCREEN_ADDRESS 0x3C

// =====================================================
// EXTERN GLOBALS — defined in userInterfaceScreen.cpp
// =====================================================
extern Adafruit_SSD1306 display;
extern ScreenState screenState;
extern unsigned long lastStatusUpdate;

extern const char* homeMenu[];
extern const char* initMenu[];
extern const char* configMenu[];
extern const char* cycleMenu[];
extern const char* prehenseurMenu[];
extern const char* objectMenu[];
extern const char* modeMenu[];
extern const char* combineMenu[];

// =====================================================
// FUNCTION PROTOTYPES
// =====================================================

// --- Text helpers ---
const char* getCycleText();
const char* getObjectText();
const char* getPrehenseurText();
const char* getModeText();
int         getCombineValue();
const char* getRunStateText();

// --- Draw functions ---
void drawMenu(const char* title, const char* items[], int itemCount, int selected);
void drawStatusScreen();
void drawInfoMessage(const char* title, const char* line1, const char* line2 = "");
void drawReadyScreen();
void drawTerminalModeScreen();

// --- Refresh ---
void refreshScreen();
void refreshStatusBetweenMoves();

// --- Terminal mode display control ---
void enterTerminalMode();
void exitTerminalMode();

// --- Menu helpers (used by handleEncoder) ---
int  getMenuCount(ScreenState s);
void configChanged();
void terminalConfigChanged();

#endif // USERINTERFACESCREEN_H
