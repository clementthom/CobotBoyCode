#include <Arduino.h>
#include "userInterfaceScreen.h"
#include "trajectories.h"

// ====================================================
// [userInterfaceScreen.cpp]  GLOBALS — display object, screen state, menus
// ====================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

ScreenState screenState = SCREEN_HOME;

const char* homeMenu[] = { "Init", "Config", "Reglages", "Info", "Statut" };
const char* initMenu[] = { "Repos + Init", "Retour" };
const char* configMenu[] = { "Choix Cycle", "Choix Preh.", "Choix Objet", "Choix Mode", "Nb Pieces", "Retour" };
const char* cycleMenu[] = { "Cycle 1", "Cycle 2", "Retour" };
const char* prehenseurMenu[] = { "Preh. 1 offset", "Preh. 2 centre", "Retour" };
const char* objectMenu[] = { "Gomme", "Gobelet", "Cylindre", "Retour" };
const char* modeMenu[] = { "ECO", "Normal", "RAPIDE", "Retour" };
const char* combineMenu[] = { "1", "2", "3", "4", "5", "Retour" };

unsigned long lastStatusUpdate = 0;
const unsigned long statusRefreshDelay = 500;

// ====================================================
// [userInterfaceScreen.cpp]  TEXT HELPERS
// ====================================================

const char* getCycleText() {
  return (currentCycleType == 0) ? "Cycle 1" : "Cycle 2";
}
const char* getObjectText() {
  return getObjectNameByIndex(currentObject);
}
const char* getPrehenseurText() {
  return getPrehenseurNameByIndex(currentPrehenseur);
}
const char* getModeText() {
  if (currentMode == 0) return "ECO";
  if (currentMode == 1) return "Normal";
  return "RAPIDE";
}
int getCombineValue() {
  return currentCombine + 1;
}
const char* getRunStateText() {
  if (isInitializing) return "INITIALISATION";
  if (isPaused) return "PAUSE";
  if (isRunning) return "EN MARCHE";
  if (isInitialized) return "PRET";
  return "NON INIT";
}

// ====================================================
// [userInterfaceScreen.cpp]  OLED DRAW FUNCTIONS
// ====================================================

void drawMenu(const char* title, const char* items[], int itemCount, int selected) {
  if (terminalModeActive) {
    drawTerminalModeScreen();
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(title);
  display.println(F("----------------"));

  int startIdx = 0;
  if (selected > 3) startIdx = selected - 3;

  for (int i = startIdx; i < min(itemCount, startIdx + 4); i++) {
    if (i == selected) display.print(F("> "));
    else display.print(F("  "));
    display.println(items[i]);
  }

  display.display();
}
void drawStatusScreen() {
  if (terminalModeActive) {
    drawTerminalModeScreen();
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.println(F("STATUT"));
  display.println(F("----------------"));
  display.print(F("Cycle : "));
  display.println(getCycleText());
  display.print(F("Preh. : "));
  display.println(getPrehenseurText());
  display.print(F("Objet : "));
  display.println(getObjectText());
  display.print(F("Mode  : "));
  display.println(getModeText());
  display.print(F("Piece : "));
  display.print(currentPiece);
  display.print(F("/"));
  display.println(getCombineValue());
  display.print(F("Etat  : "));
  display.println(getRunStateText());

  display.display();
}
void drawInfoMessage(const char* title, const char* line1, const char* line2 = "") {
  if (terminalModeActive) {
    drawTerminalModeScreen();
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.println(title);
  display.println(F("----------------"));
  display.println(line1);
  if (strlen(line2) > 0) display.println(line2);

  display.display();
}
void drawReadyScreen() {
  if (terminalModeActive) {
    drawTerminalModeScreen();
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.println(F("PRET"));
  display.println(F("----------------"));
  display.print(F("Cy: "));
  display.println(getCycleText());
  display.print(F("Pr:"));
  display.println(getPrehenseurText());
  display.print(F("Obj:"));
  display.println(getObjectText());
  display.print(F("Md: "));
  display.println(getModeText());
  display.print(F("Nb: "));
  display.println(getCombineValue());
  display.println(F("Start = marche"));
  display.println(F("Enc = menu"));

  display.display();
}

// ====================================================
// [userInterfaceScreen.cpp]  SCREEN REFRESH
// ====================================================

void refreshScreen() {
  if (terminalModeActive) {
    drawTerminalModeScreen();
    return;
  }

  if (isRunning && !isPaused) return;

  switch (screenState) {
    case SCREEN_HOME: drawMenu("MENU", homeMenu, 5, menuIndex); break;
    case SCREEN_INIT_MENU: drawMenu("INIT", initMenu, 2, menuIndex); break;
    case SCREEN_CONFIG: drawMenu("CONFIG", configMenu, 6, menuIndex); break;
    case SCREEN_CYCLE_MENU: drawMenu("CHOIX CYCLE", cycleMenu, 3, menuIndex); break;
    case SCREEN_PREHENSEUR_MENU: drawMenu("CHOIX PREH.", prehenseurMenu, 3, menuIndex); break;
    case SCREEN_OBJECT_MENU: drawMenu("CHOIX OBJET", objectMenu, 4, menuIndex); break;
    case SCREEN_MODE_MENU: drawMenu("CHOIX MODE", modeMenu, 4, menuIndex); break;
    case SCREEN_COMBINE_MENU: drawMenu("COMBINE", combineMenu, 6, menuIndex); break;
    case SCREEN_STATUS: drawStatusScreen(); break;
    case SCREEN_READY: drawReadyScreen(); break;
    default: break;
  }
}
void refreshStatusBetweenMoves() {
  if (screenState == SCREEN_STATUS) {
    if (terminalModeActive) drawTerminalModeScreen();
    else drawStatusScreen();
  }
}

// ====================================================
// [userInterfaceScreen.cpp]  TERMINAL MODE DISPLAY
// ====================================================

void drawTerminalModeScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.println(F("MAINTENANCE"));
  display.println(F("MODE TERMINAL"));
  display.println(F("----------------"));
  display.println(F("Python connecte"));
  display.println(F("Utiliser PC"));
  display.println(F("OLED verrouille"));
  display.println(F("Cavalier inutile"));

  display.display();
}
void enterTerminalMode() {
  terminalModeActive = true;
  lastTerminalContact = millis();
  screenState = SCREEN_STATUS;
  drawTerminalModeScreen();
}
void exitTerminalMode() {
  terminalModeActive = false;
  screenState = isInitialized ? SCREEN_READY : SCREEN_HOME;
  refreshScreen();
}

