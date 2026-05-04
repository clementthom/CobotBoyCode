#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
#include <AutoPID.h>
#include <MsTimer2.h>

// ====================================================
// [→ common.h]  SHARED TYPES — structs & enums used across modules
// ====================================================
// (originally from: ROBOT KINEMATICS, CYCLE POSITIONS,
//  HMI GLOBAL STATES, RUNTIME CONFIG, GRIPPER PID sections)

struct Point3D {
  float x;
  float y;
  float z;
};

struct JointCmd {
  float a;
  float b;
  float z;
  bool reachable;
};

const int NB_CYCLES = 2;
const int NB_OBJECTS = 3;
const int NB_POSITIONS = 4;
const int NB_PREHENSEURS = 2;

const int PREHENSEUR_OFFSET = 0;
const int PREHENSEUR_CENTERED = 1;

const int OBJ_GOMME = 0;
const int OBJ_GOBELET = 1;
const int OBJ_CYLINDRE = 2;

const int POS_convoyeurEntree = 0;
const int POS_machineA = 1;
const int POS_machineB = 2;
const int POS_convoyeurSortie = 3;

enum ScreenState {
  SCREEN_HOME,
  SCREEN_INIT_MENU,
  SCREEN_CONFIG,
  SCREEN_SETTINGS,
  SCREEN_INFO,
  SCREEN_STATUS,
  SCREEN_CYCLE_MENU,
  SCREEN_PREHENSEUR_MENU,
  SCREEN_OBJECT_MENU,
  SCREEN_MODE_MENU,
  SCREEN_COMBINE_MENU,
  SCREEN_READY
};

struct PositionOffset {
  float dx;
  float dy;
  float dz;
};

struct GripSettings {
  float openSetpoint;
  float closeSetpoint;
};

enum GripperState {
  GRIPPER_STOPPED,
  GRIPPER_OPENING,
  GRIPPER_CLOSING
};

GripperState gripperState = GRIPPER_STOPPED;

double gripperPWM_PID = 0.0;
double measuredTension = 0.0;
double gripperSetpoint = 0.0;


// ====================================================
// [→ userInterfaceHardware.h]  PIN MAPPING
// ====================================================
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


// ====================================================
// [→ userInterfaceScreen.h]  OLED CONFIG
// ====================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ====================================================
// [→ cinematics.h / cinematics.cpp]  KINEMATICS PARAMETERS & STATE
// ====================================================
// (struct Point3D and struct JointCmd moved to [common.h] above)
Servo servoA;
Servo servoB;
Servo servoZ;

float L1 = 150.0;
float L2 = 143.0;
float offsetX = 20.0;
float offsetZ = 75.0;

float degOffsetA = -20.0;
float degOffsetB = -10.0;
float degOffsetZ = -2.0;

float toolRadius = 40.0;
float objectRadius = 32.0;
float toolOffset = 65.0;
float prehensionHeight = 50.0;
float prehensionHeightBloc = 50.0;

float toolLength = toolOffset - (toolRadius - objectRadius);
float toolHeight = -45;

float maxStepA = 0.2;
float maxStepB = 0.2;
float maxStepZ = 0.5;

float executionSpeedFactor = 1.0;

// Cycle runtime measurement for the maintenance terminal.
// This only measures time; it does not change movement logic.
unsigned long cycleStartTime = 0;
unsigned long cycleEndTime = 0;

float initX = 0.0;
float initY = 130.0;
float initZ = 120.0;

float pauseReturnX = 0.0;
float pauseReturnY = 130.0;
float pauseReturnZ = 120.0;

bool pauseMoveInProgress = false;

float currentX = 0.0;
float currentY = 130.0;
float currentZ = 120.0;

float currentCmdA = 90.0;
float currentCmdB = 90.0;
float currentCmdZ = 90.0;


// ====================================================
// [→ trajectories.h / trajectories.cpp]  CYCLE POSITION ARRAYS
// ====================================================
// (NB_* dimension constants and OBJ_*/POS_* index constants moved to [common.h] above)
Point3D basePositions[NB_CYCLES][NB_POSITIONS] = {
  //position cycle 1
  {
    { 155, 20, prehensionHeight },
    { -155, 115, prehensionHeight },
    { 160, 150, prehensionHeight },
    { -155, 20, prehensionHeight } },
  //position cycle 2
  {
    { -155, 25, prehensionHeight },         //convoyeur entrée
    { -145, 155, (prehensionHeightBloc) },  //machine A
    { 160, 100, prehensionHeight },         //machine B
    { 160, 20, prehensionHeight }           //convoyeur sortie
  }
  /*OLD absolue
  //position cycle 1
  {
    {155, 20, prehensionHeight},
    {-155, 115, prehensionHeight},
    {160, 150, prehensionHeight},
    {-155, 20, prehensionHeight}
  },
  //position cycle 2
  {
    {-155, 20, prehensionHeight}, //convoyeur entrée
    {-155, 152, (prehensionHeightBloc)},  //machine A
    {165, 116, prehensionHeight}, //machine B
    {155, 20, prehensionHeight} //convoyeur esortie
  }
  */
};

const char* positionNames[NB_CYCLES][NB_POSITIONS] = {
  { "Entree", "A", "B", "Sortie" },
  { "Entree", "Machine A", "Machine B", "Sortie" }
};


// ====================================================
// [→ userInterfaceHardware.h + userInterfaceScreen.h]  HMI STATE GLOBALS
// ====================================================
// (enum ScreenState moved to [common.h] above)
ScreenState screenState = SCREEN_HOME;

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
const unsigned long bootIgnoreButtonsMs = 1500;

bool previousButtonState[3] = { HIGH, HIGH, HIGH };
unsigned long lastDebounceTime[3] = { 0, 0, 0 };
const unsigned long debounceDelay = 50;

int lastEncCLK = HIGH;
unsigned long lastEncoderMoveTime = 0;
unsigned long lastEncoderButtonTime = 0;
const unsigned long encoderMoveDelay = 80;
const unsigned long encoderButtonDelay = 300;

unsigned long lastStatusUpdate = 0;
const unsigned long statusRefreshDelay = 500;

const char* homeMenu[] = { "Init", "Config", "Reglages", "Info", "Statut" };
const char* initMenu[] = { "Repos + Init", "Retour" };
const char* configMenu[] = { "Choix Cycle", "Choix Preh.", "Choix Objet", "Choix Mode", "Nb Pieces", "Retour" };
const char* cycleMenu[] = { "Cycle 1", "Cycle 2", "Retour" };
const char* prehenseurMenu[] = { "Preh. 1 offset", "Preh. 2 centre", "Retour" };
const char* objectMenu[] = { "Gomme", "Gobelet", "Cylindre", "Retour" };
const char* modeMenu[] = { "ECO", "Normal", "RAPIDE", "Retour" };
const char* combineMenu[] = { "1", "2", "3", "4", "5", "Retour" };

// ====================================================
// [→ prehension.h globals]  GRIPPER SETPOINTS
// ====================================================
double consigneOuverture = 1.80;
double consigneFermeture = 1.40;

double seuilOuvertureMax = 2.30;
double seuilFermetureMax = 2.60;


// ====================================================
// [→ trajectories.cpp globals]  RUNTIME POSITION & GRIP OFFSET TABLES
// ====================================================
// (struct PositionOffset and struct GripSettings moved to [common.h] above)
PositionOffset positionOffsets[NB_PREHENSEURS][NB_CYCLES][NB_OBJECTS][NB_POSITIONS];
GripSettings gripSettings[NB_PREHENSEURS][NB_OBJECTS];

// ====================================================
// [→ prehension.h / prehension.cpp globals]  GRIPPER PID STATE
// ====================================================
// (enum GripperState moved to [common.h] above)
#define KP_GRIP 90.0
#define KI_GRIP 0.0
#define KD_GRIP 2.5

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
// [main.ino]  FORWARD DECLARATIONS
// ====================================================
void smartWait(unsigned long ms);
void handleButtons();
void moveToReposPosition();
void moveRectangular(float xf, float yf, float zf, float liftHeight, unsigned long tUp, unsigned long tXY, unsigned long tDown);
void performInitialization();
void executeCurrentScreen();
void handleApiSerial();
void sendStatus();
void sendConfig();
void apiGotoCycle1EntreeTestHeight();
void apiGripperAutoTest(String cmd);
void ihmInitializeAction();
void ihmStartAction();
void ihmPauseToggleAction();
void enterTerminalMode();
void exitTerminalMode();
void drawTerminalModeScreen();

// ====================================================
// [→ userInterfaceHardware.cpp]  INTERRUPT SERVICE ROUTINES
// ====================================================
void pauseISR() {
  pauseInterruptRequest = true;
}

volatile bool tick10ms = false;

void timerISR() {
  tick10ms = true;
}


// ====================================================
// [→ userInterfaceHardware.cpp]  BUTTON LOGIC
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
// [→ userInterfaceScreen.cpp]  TEXT HELPERS
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
// [→ userInterfaceScreen.cpp]  OLED DRAW FUNCTIONS
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
// [→ userInterfaceHardware.cpp]  LEDs
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
// [→ trajectories.cpp + main.ino]  CONFIGURATION HELPERS
// ====================================================
const char* getObjectNameByIndex(int obj) {
  if (obj == 0) return "Gomme";
  if (obj == 1) return "Gobelet";
  return "Cylindre";
}

const char* getPrehenseurNameByIndex(int prehenseur) {
  if (prehenseur == PREHENSEUR_CENTERED) return "Prehenseur 2";
  return "Prehenseur 1";
}

bool selectedPrehenseurUsesToolOffset() {
  return currentPrehenseur == PREHENSEUR_OFFSET;
}

void applyObjectGeometry(int objectIndex) {
  if (objectIndex == OBJ_GOBELET) {
    objectRadius = 25;
    prehensionHeight = 30;
    Serial.print("Rayon objet = ");
    Serial.print(objectRadius);
    Serial.print("Hauteur prehension = ");
    Serial.println(prehensionHeight);
  } else if (objectIndex == OBJ_GOMME) {
    objectRadius = 20;
    prehensionHeight = 20;
    Serial.print("Rayon objet = ");
    Serial.print(objectRadius);
    Serial.print("Hauteur prehension = ");
    Serial.println(prehensionHeight);
  } else if (objectIndex == OBJ_CYLINDRE) {
    objectRadius = 20;
    prehensionHeight = 22;
    Serial.print("Rayon objet = ");
    Serial.print(objectRadius);
    Serial.print("Hauteur prehension = ");
    Serial.println(prehensionHeight);
  }

  toolLength = toolOffset - (toolRadius - objectRadius);
  Serial.println(toolLength);
}

void setDefaultRuntimeConfig() {
  for (int pr = 0; pr < NB_PREHENSEURS; pr++) {
    for (int c = 0; c < NB_CYCLES; c++) {
      for (int o = 0; o < NB_OBJECTS; o++) {
        for (int pos = 0; pos < NB_POSITIONS; pos++) {
          positionOffsets[pr][c][o][pos].dx = 0.0;
          positionOffsets[pr][c][o][pos].dy = 0.0;
          positionOffsets[pr][c][o][pos].dz = 0.0;
        }
      }
    }

    gripSettings[pr][OBJ_GOMME].openSetpoint = 1.80;
    gripSettings[pr][OBJ_GOMME].closeSetpoint = 1.30;

    gripSettings[pr][OBJ_GOBELET].openSetpoint = 1.80;
    gripSettings[pr][OBJ_GOBELET].closeSetpoint = 1.15;

    gripSettings[pr][OBJ_CYLINDRE].openSetpoint = 1.80;
    gripSettings[pr][OBJ_CYLINDRE].closeSetpoint = 1.40;
  }

  executionSpeedFactor = 1.0;
  degOffsetA = -20.0;
  degOffsetB = -10.0;
  degOffsetZ = -2.0;
}

void applyGripperSettingsForObject(int objectIndex) {
  consigneOuverture = gripSettings[currentPrehenseur][objectIndex].openSetpoint;
  consigneFermeture = gripSettings[currentPrehenseur][objectIndex].closeSetpoint;
}

void applyFullConfiguration() {
  if (executionSpeedFactor < 0.5 || executionSpeedFactor > 3.0) {
    executionSpeedFactor = 1.0;
  }

  applyObjectGeometry(currentObject);
  applyGripperSettingsForObject(currentObject);
}

PositionOffset getOffset(int prehenseur, int cycle, int object, int position) {
  return positionOffsets[prehenseur][cycle][object][position];
}

PositionOffset getOffset(int cycle, int object, int position) {
  return getOffset(currentPrehenseur, cycle, object, position);
}

// Converts a taught Machine A/B point from prehenseur-circle center
// to real piece center using the same tool geometry as the IK.
//
// toolLength = toolOffset - (toolRadius - objectRadius)
// therefore the centering correction is: toolOffset - toolLength
// which equals the radial difference between the prehenseur center
// and the real piece center.
void applyMachinePieceCentering(Point3D& target, int position) {
  if (!selectedPrehenseurUsesToolOffset()) {
    return;
  }

  if (position != POS_machineA && position != POS_machineB) {
    return;
  }

  float r = sqrt(target.x * target.x + target.y * target.y);
  if (r < 0.001) {
    return;
  }

  float ux = target.x / r;
  float uy = target.y / r;

  float pieceCenterCorrection = toolOffset - toolLength;

  target.x += ux * pieceCenterCorrection;
  target.y += uy * pieceCenterCorrection;
}

Point3D getCorrectedPosition(int cycle, int object, int position) {
  Point3D base = basePositions[cycle][position];
  PositionOffset off = getOffset(cycle, object, position);

  Point3D corrected;
  corrected.x = base.x + off.dx;
  corrected.y = base.y + off.dy;
  corrected.z = base.z + off.dz;

  return corrected;
}

Point3D getCorrectedPrehensionPosition(int cycle, int object, int position) {
  applyObjectGeometry(object);

  Point3D base = basePositions[cycle][position];
  PositionOffset off = getOffset(cycle, object, position);

  Point3D corrected;
  corrected.x = base.x + off.dx;
  corrected.y = base.y + off.dy;

  // Machine A/B taught positions are treated as piece-center targets.
  // This shifts the commanded XY point so the actual object/piece center,
  // not only the middle of the circular prehenseur, is centered on the machine.
  applyMachinePieceCentering(corrected, position);

  // Default object pickup/drop height
  corrected.z = prehensionHeight + off.dz;

  // Cycle 2 Machine A must be 50 mm higher
  if (cycle == 1 && position == POS_machineA) {
    corrected.z = prehensionHeight + 50.0 + off.dz;
  }

  return corrected;
}

void configChanged() {
  isInitialized = false;
  isInitializing = false;
  isRunning = false;
  isPaused = false;
  currentPiece = 0;

  applyObjectGeometry(currentObject);
  applyFullConfiguration();

  Serial.print(F("Rayon objet = "));
  Serial.println(objectRadius);

  Serial.print(F("Hauteur prehension = "));
  Serial.println(prehensionHeight);
}

void terminalConfigChanged() {
  if (isInitializing) return;
  // In Python terminal mode, changing the dropdown values must NOT cancel INIT.
  // The Python UI sends SET_CYCLE / SET_OBJECT / SET_MODE / SET_PIECES / SET_SPEED
  // before INIT and also before PLAY. If we reset isInitialized here, PLAY will fail.
  if (isRunning) return;
  currentPiece = 0;
  applyFullConfiguration();
  if (terminalModeActive) drawTerminalModeScreen();
}


// ====================================================
// [→ userInterfaceHardware.cpp]  HMI — ENCODER & MENU NAVIGATION
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
// [→ prehension.cpp]  GRIPPER PID FUNCTIONS
// ====================================================
double readVoltage(int pin) {
  return ((double)analogRead(pin) * 5.0) / 1023.0;
}

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


// ====================================================
// [→ main.ino]  SYSTEM YIELD & SMART WAIT
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


// ====================================================
// [→ cinematics.cpp]  KINEMATICS
// ====================================================
int degToUs(float deg) {
  deg = constrain(deg, 0.0, 180.0);
  return (int)(500.0 + (deg / 180.0) * 2000.0);
}

void writeServosMicroseconds(float cmdA, float cmdB, float cmdZ) {
  servoA.writeMicroseconds(degToUs(cmdA));
  servoB.writeMicroseconds(degToUs(cmdB));
  servoZ.writeMicroseconds(degToUs(cmdZ));
}

float limitStep(float currentValue, float targetValue, float maxStep) {
  float delta = targetValue - currentValue;

  if (delta > maxStep) return currentValue + maxStep;
  if (delta < -maxStep) return currentValue - maxStep;

  return targetValue;
}

JointCmd computeIK(float x, float y, float z) {
  JointCmd out;
  out.reachable = false;

  float rTool = sqrt(x * x + y * y);
  float theta3 = atan2(y, x);

  float rWrist = rTool - toolLength;
  float zWrist = z - toolHeight;

  float rp = rWrist - offsetX;
  float zp = zWrist - offsetZ;

  float D = (rp * rp + zp * zp - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);

  if (D < -1.0 || D > 1.0) return out;

  float theta1 = atan2(sqrt(1.0 - D * D), D);
  float theta2 = atan2(zp, rp) - atan2(L2 * sin(theta1), L1 + L2 * cos(theta1));

  float degA = degrees(theta2) + 180.0;
  float degB = 180.0 - degrees(theta2 + theta1);
  float degZ = degrees(theta3);

  float cmdA = degA + degOffsetA;
  float cmdB = degB + degOffsetB;
  float cmdZ = degZ + degOffsetZ;

  if (cmdA >= 0.0 && cmdA <= 180.0 && cmdB >= 0.0 && cmdB <= 180.0 && cmdZ >= 0.0 && cmdZ <= 180.0) {
    out.a = cmdA;
    out.b = cmdB;
    out.z = cmdZ;
    out.reachable = true;
  }

  return out;
}




// ====================================================
// [→ cinematics.cpp]  MOVEMENT
// ====================================================
void moveJointsSmoothTo(JointCmd target, unsigned long dtMs = 5) {
  if (!target.reachable) return;

  bool done = false;

  while (!done) {
    systemYield();

    currentCmdA = limitStep(currentCmdA, target.a, maxStepA);
    currentCmdB = limitStep(currentCmdB, target.b, maxStepB);
    currentCmdZ = limitStep(currentCmdZ, target.z, maxStepZ);

    writeServosMicroseconds(currentCmdA, currentCmdB, currentCmdZ);

    done = abs(currentCmdA - target.a) < 0.05 && abs(currentCmdB - target.b) < 0.05 && abs(currentCmdZ - target.z) < 0.05;

    delay(dtMs);
  }
}

float trapezoidalProfile(float u, float alpha = 0.2) {
  if (u <= 0.0) return 0.0;
  if (u >= 1.0) return 1.0;
  if (alpha <= 0.0) return u;
  if (alpha >= 0.5) alpha = 0.49;

  float vmax = 1.0 / (1.0 - alpha);
  float s = 0.0;

  if (u < alpha) {
    s = vmax * (u * u) / (2.0 * alpha);
  } else if (u < (1.0 - alpha)) {
    s = vmax * (alpha / 2.0 + (u - alpha));
  } else {
    float du = 1.0 - u;
    s = 1.0 - vmax * (du * du) / (2.0 * alpha);
  }

  return s;
}

void moveLinearSegment(Point3D p0, Point3D p1, unsigned long dureeMs, unsigned long dtMs = 20) {
  if (dureeMs < dtMs) dureeMs = dtMs;

  int steps = max((int)(dureeMs / dtMs), 20);

  for (int i = 0; i <= steps; i++) {
    systemYield();

    float u = (float)i / steps;
    float s = trapezoidalProfile(u, 0.5);

    float x = p0.x + s * (p1.x - p0.x);
    float y = p0.y + s * (p1.y - p0.y);
    float z = p0.z + s * (p1.z - p0.z);

    JointCmd target = computeIK(x, y, z);

    if (target.reachable) {
      currentCmdA = target.a;
      currentCmdB = target.b;
      currentCmdZ = target.z;

      currentX = x;
      currentY = y;
      currentZ = z;

      writeServosMicroseconds(currentCmdA, currentCmdB, currentCmdZ);
    }

    delay(dtMs);
  }

  JointCmd target = computeIK(p1.x, p1.y, p1.z);

  if (target.reachable) {
    moveJointsSmoothTo(target, dtMs);
    currentX = p1.x;
    currentY = p1.y;
    currentZ = p1.z;
  }
}

void moveRectangular(float xf, float yf, float zf, float liftHeight, unsigned long tUp, unsigned long tXY, unsigned long tDown) {
  Point3D pStart = { currentX, currentY, currentZ };

  float zLift = max(currentZ, zf) + liftHeight;

  Point3D pUp = { currentX, currentY, zLift };
  Point3D pMid = { xf, yf, zLift };
  Point3D pEnd = { xf, yf, zf };

  moveLinearSegment(pStart, pUp, tUp, 20);
  moveLinearSegment(pUp, pMid, tXY, 20);
  moveLinearSegment(pMid, pEnd, tDown, 20);
}

// V13: quick Servo B retract used immediately after releasing a piece.
// This replaces the slower V12 Cartesian retract.
void servoBBackwardBeforeLift(float degreesBack, unsigned long durationMs) {
  float startB = currentCmdB;
  float targetB = constrain(currentCmdB - degreesBack, 0.0, 180.0);

  int steps = max((int)(durationMs / 10), 1);

  for (int i = 1; i <= steps; i++) {
    systemYield();
    float u = (float)i / steps;
    currentCmdB = startB + u * (targetB - startB);
    writeServosMicroseconds(currentCmdA, currentCmdB, currentCmdZ);
    delay(10);
  }

  currentCmdB = targetB;
  writeServosMicroseconds(currentCmdA, currentCmdB, currentCmdZ);
}

// V13: after Servo B has been nudged backward, do not restart the lift from
// the old Cartesian drop pose. Go upward first, then continue normally.

// V16: synchronized Servo A + Servo B retract used after releasing at Machine A.
// This creates a small diagonal motion away from the object before the Z lift.
void servoABBackwardBeforeLift(float degreesA, float degreesB, unsigned long durationMs) {
  float startA = currentCmdA;
  float startB = currentCmdB;

  float targetA = constrain(currentCmdA + degreesA, 0.0, 180.0);
  float targetB = constrain(currentCmdB - degreesB, 0.0, 180.0);

  int steps = max((int)(durationMs / 10), 1);

  for (int i = 1; i <= steps; i++) {
    systemYield();

    float u = (float)i / steps;

    currentCmdA = startA + u * (targetA - startA);
    currentCmdB = startB + u * (targetB - startB);

    writeServosMicroseconds(currentCmdA, currentCmdB, currentCmdZ);
    delay(10);
  }

  currentCmdA = targetA;
  currentCmdB = targetB;
  writeServosMicroseconds(currentCmdA, currentCmdB, currentCmdZ);
}

void moveRectangularAfterServoBNudge(float xf, float yf, float zf, float liftHeight, unsigned long tUp, unsigned long tXY, unsigned long tDown) {
  float zLift = max(currentZ, zf) + liftHeight;

  JointCmd upCmd = computeIK(currentX, currentY, zLift);
  if (upCmd.reachable) {
    moveJointsSmoothTo(upCmd, 5);
    currentZ = zLift;
    currentCmdA = upCmd.a;
    currentCmdB = upCmd.b;
    currentCmdZ = upCmd.z;
  }

  Point3D pUp = { currentX, currentY, currentZ };
  Point3D pMid = { xf, yf, currentZ };
  Point3D pEnd = { xf, yf, zf };

  moveLinearSegment(pUp, pMid, tXY, 20);
  moveLinearSegment(pMid, pEnd, tDown, 20);
}

// V17: Sortie needs a safe vertical lift before moving back to repos.
// If the requested lift height is unreachable, this tries smaller safe lift heights.
// It will NOT move in X/Y unless the Z lift succeeded, avoiding dragging the piece.
void leaveSortieAfterDrop(float xf, float yf, float zf, float liftHeight, unsigned long tUp, unsigned long tXY, unsigned long tDown) {
  servoBBackwardBeforeLift(5.0, 120);

  float startX = currentX;
  float startY = currentY;
  float startZ = currentZ;

  float liftOptions[4] = { liftHeight, 80.0, 60.0, 40.0 };
  bool lifted = false;

  for (int i = 0; i < 4; i++) {
    float zLift = max(startZ, zf) + liftOptions[i];
    JointCmd upCmd = computeIK(startX, startY, zLift);

    if (upCmd.reachable) {
      moveJointsSmoothTo(upCmd, 5);
      currentX = startX;
      currentY = startY;
      currentZ = zLift;
      currentCmdA = upCmd.a;
      currentCmdB = upCmd.b;
      currentCmdZ = upCmd.z;
      lifted = true;
      break;
    }
  }

  if (!lifted) {
    Serial.println(F("WARNING: Sortie leave aborted because no safe Z lift was reachable."));
    return;
  }

  Point3D pUp = { currentX, currentY, currentZ };
  Point3D pMid = { xf, yf, currentZ };
  Point3D pEnd = { xf, yf, zf };

  moveLinearSegment(pUp, pMid, tXY, 20);
  moveLinearSegment(pMid, pEnd, tDown, 20);
}

void moveToReposPosition() {
  moveRectangular(initX, initY, initZ, 100, 200, 200, 200);
}

bool returnToReposFromCurrentServoAngles() {
  JointCmd reposCmd = computeIK(initX, initY, initZ);

  if (!reposCmd.reachable) {
    Serial.println(F("ERR;REPOS_NOT_REACHABLE"));
    return false;
  }

  moveJointsSmoothTo(reposCmd, 5);

  currentX = initX;
  currentY = initY;
  currentZ = initZ;
  currentCmdA = reposCmd.a;
  currentCmdB = reposCmd.b;
  currentCmdZ = reposCmd.z;

  writeServosMicroseconds(currentCmdA, currentCmdB, currentCmdZ);
  return true;
}

bool initializeFromCartesianPosition(float x, float y, float z) {
  JointCmd initCmd = computeIK(x, y, z);

  if (!initCmd.reachable) return false;

  currentX = x;
  currentY = y;
  currentZ = z;

  currentCmdA = initCmd.a;
  currentCmdB = initCmd.b;
  currentCmdZ = initCmd.z;

  writeServosMicroseconds(currentCmdA, currentCmdB, currentCmdZ);

  return true;
}



// ====================================================
// [→ main.ino]  INITIALIZATION ACTION
// ====================================================
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

// ====================================================
// [→ main.ino]  SHARED IHM ACTIONS (physical buttons + Python terminal)
// ====================================================
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



// ====================================================
// [→ userInterfaceHardware.cpp]  LED & BUTTON TEST HELPERS
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
// [→ userInterfaceHardware.cpp]  HANDLE BUTTONS
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
// [→ trajectories.cpp]  SPEED PROFILE & POSITION MOVEMENT HELPERS
// ====================================================
void getCycleTimes(unsigned long& tUp, unsigned long& tXY, unsigned long& tDown) {
  float speedMult = 1.0;

  if (currentMode == 0) speedMult = 2.5;
  if (currentMode == 1) speedMult = 1.5;
  if (currentMode == 2) speedMult = 1.0;

  speedMult = speedMult * executionSpeedFactor;

  unsigned long tUpBase = 500;
  unsigned long tXYBase = 500;
  unsigned long tDownBase = 500;

  tUp = tUpBase * speedMult;
  tXY = tXYBase * speedMult;
  tDown = tDownBase * speedMult;
}

void moveToBasePositionForObject(int cycle, int objectIndex, int position, float liftHeight, unsigned long tUp, unsigned long tXY, unsigned long tDown) {
  Point3D target = getCorrectedPosition(cycle, objectIndex, position);
  moveRectangular(target.x, target.y, target.z, liftHeight, tUp, tXY, tDown);
}

void moveToPrehensionPositionForObject(int cycle, int objectIndex, int position, float liftHeight, unsigned long tUp, unsigned long tXY, unsigned long tDown) {
  applyObjectGeometry(objectIndex);

  Point3D base = basePositions[cycle][position];
  Point3D target = getCorrectedPrehensionPosition(cycle, objectIndex, position);

  Serial.println("=== DEBUG POSITION ===");
  Serial.print("Cycle: "); Serial.println(cycle);
  Serial.print("Object: "); Serial.println(objectIndex);
  Serial.print("Position: "); Serial.println(position);

  Serial.print("BASE X: "); Serial.println(base.x);
  Serial.print("BASE Y: "); Serial.println(base.y);
  Serial.print("BASE Z: "); Serial.println(base.z);

  Serial.print("TARGET X: "); Serial.println(target.x);
  Serial.print("TARGET Y: "); Serial.println(target.y);
  Serial.print("TARGET Z: "); Serial.println(target.z);
  Serial.println("======================");

  moveRectangular(target.x, target.y, target.z, liftHeight, tUp, tXY, tDown);
}

// Used when the robot comes back to re-grab an object after a drop.
// This keeps the drop position unchanged, but lets the grab-back position
// be shifted slightly so the prehenseur does not land directly on top of the object.
void moveToPrehensionPositionExtraForObject(
  int cycle,
  int objectIndex,
  int position,
  float extraX,
  float extraY,
  float extraZ,
  float liftHeight,
  unsigned long tUp,
  unsigned long tXY,
  unsigned long tDown) {
  applyObjectGeometry(objectIndex);

  Point3D target = getCorrectedPrehensionPosition(cycle, objectIndex, position);

  target.x += extraX;
  target.y += extraY;
  target.z += extraZ;

  moveRectangular(target.x, target.y, target.z, liftHeight, tUp, tXY, tDown);
}

void moveToBasePosition(int cycle, int position, float liftHeight, unsigned long tUp, unsigned long tXY, unsigned long tDown) {
  moveToBasePositionForObject(cycle, currentObject, position, liftHeight, tUp, tXY, tDown);
}

void prepareObjectCycle(int objectIndex) {
  currentObject = objectIndex;
  applyObjectGeometry(objectIndex);
  applyGripperSettingsForObject(objectIndex);
}


// ====================================================
// [→ trajectories.cpp]  CYCLE SEQUENCES
// ====================================================
void SequenceCycle1ForObject(int objectIndex) {
  prepareObjectCycle(objectIndex);

  unsigned long tUp;
  unsigned long tXY;
  unsigned long tDown;

  getCycleTimes(tUp, tXY, tDown);

  openGripper();
  smartWait(500);
  // convoyeur entrée
  moveToPrehensionPositionForObject(0, objectIndex, POS_convoyeurEntree, 100, tUp, tXY, tDown);

  closeGripper();
  smartWait(2000);
  // machine A
  moveToPrehensionPositionForObject(0, objectIndex, POS_machineA, 100, tUp, tXY, tDown);
  smartWait(3000);

  moveToPrehensionPositionForObject(0, objectIndex, POS_machineB, 100, tUp, tXY, tDown);
  smartWait(3000);

  moveToPrehensionPositionForObject(0, objectIndex, POS_convoyeurSortie, 100, tUp, tXY, tDown);

  openGripper();
  smartWait(2000);

  moveRectangular(initX, initY, initZ, 100, tUp, tXY, tDown);

  closeGripper();
  smartWait(1000);
}

// =====================================================
// GENERIC SEQUENCE CYCLE 2
// The object is passed in so offsets + gripper setpoints are object-specific.
// =====================================================
void SequenceCycle2ForObject(int objectIndex) {
  prepareObjectCycle(objectIndex);

  unsigned long tUp;
  unsigned long tXY;
  unsigned long tDown;

  getCycleTimes(tUp, tXY, tDown);

  openGripper();

  //moveRectangular(initX, initY, initZ, 100, tUp, tXY, tDown);

  moveToPrehensionPositionForObject(1, objectIndex, POS_convoyeurEntree, 100, tUp, tXY, tDown);

  closeGripper();
  smartWait(3500);

  moveToPrehensionPositionForObject(1, objectIndex, POS_machineA, 100, tUp, tXY, tDown);

  openGripper();
  smartWait(3500);
  stopGripper();

  // V16: Machine A needs Servo A and Servo B together for a clean diagonal retract.
  // Start small to avoid sweeping into the object.
  servoABBackwardBeforeLift(4.0, 5.0, 180);

  moveRectangularAfterServoBNudge(initX, initY, initZ, 100, tUp, tXY, tDown);

  smartWait(5000);

  // Re-grab Machine A with diagonal correction from the drop position.
  // Drop remains unchanged; only this grab-back move is shifted.
  // Machine A re-grab correction: X +4 mm, Y -4 mm.
  moveToPrehensionPositionExtraForObject(1, objectIndex, POS_machineA, +4, -4, 0, 100, tUp, tXY, tDown);

  closeGripper();
  smartWait(3500);

  moveToPrehensionPositionForObject(1, objectIndex, POS_machineB, 100, tUp, tXY, tDown);

  openGripper();
  smartWait(3500);
  stopGripper();

  // V13: quick Servo B retract before lifting away from the released object.
  servoBBackwardBeforeLift(5.0, 120);

  moveRectangularAfterServoBNudge(initX, initY, initZ, 100, tUp, tXY, tDown);

  smartWait(5000);

  // Re-grab Machine B with correction from the drop position.
  // Drop remains unchanged; only this grab-back move is shifted.
  moveToPrehensionPositionExtraForObject(1, objectIndex, POS_machineB, -2, -3, 0, 100, tUp, tXY, tDown);

  closeGripper();
  smartWait(3500);

  moveToPrehensionPositionForObject(1, objectIndex, POS_convoyeurSortie, 100, tUp, tXY, tDown);

  openGripper();
  smartWait(3500);
  stopGripper();

  // V17: Sortie must lift in Z before travelling back to repos.
  // This prevents the open gripper from taking the piece with it.
  leaveSortieAfterDrop(initX, initY, initZ, 100, tUp, tXY, tDown);

  //closeGripper();
  //smartWait(1000);
}


// ====================================================
// [→ trajectories.cpp]  OBJECT CYCLE DISPATCHERS
// ====================================================
void cycleGomme() {
  if (currentCycleType == 0) {
    SequenceCycle1ForObject(OBJ_GOMME);
  } else {
    SequenceCycle2ForObject(OBJ_GOMME);
  }
}

void cycleGobelet() {
  if (currentCycleType == 0) {
    SequenceCycle1ForObject(OBJ_GOBELET);
  } else {
    SequenceCycle2ForObject(OBJ_GOBELET);
  }
}

void cycleCylindre() {
  if (currentCycleType == 0) {
    SequenceCycle1ForObject(OBJ_CYLINDRE);
  } else {
    SequenceCycle2ForObject(OBJ_CYLINDRE);
  }
}

// Old names kept as wrappers so any older serial/maintenance command still works.
void SequenceCycle1() {
  SequenceCycle1ForObject(currentObject);
}

void SequenceCycle2() {
  SequenceCycle2ForObject(currentObject);
}

void runSelectedObjectCycle() {
  if (currentObject == OBJ_GOMME) {
    cycleGomme();
  } else if (currentObject == OBJ_GOBELET) {
    cycleGobelet();
  } else {
    cycleCylindre();
  }
}

void updateRobotCycle() {
  if (isRunning && !isPaused) {
    int totalPieces = getCombineValue();

    while (currentPiece < totalPieces && isRunning) {
      if (terminalModeActive) drawTerminalModeScreen();
      else if (screenState == SCREEN_STATUS) drawStatusScreen();

      runSelectedObjectCycle();

      currentPiece++;

      if (terminalModeActive) drawTerminalModeScreen();
      else if (screenState == SCREEN_STATUS) drawStatusScreen();

      sendStatus();

      if (!isRunning || isPaused) break;
    }

    isRunning = false;
    isPaused = false;

    openGripper();
    smartWait(1000);
    stopGripper();

    isInitialized = true;
    currentPiece = 0;
    screenState = terminalModeActive ? SCREEN_STATUS : SCREEN_READY;
    menuIndex = 0;
    refreshScreen();

    cycleEndTime = millis();
    unsigned long cycleDuration = cycleEndTime - cycleStartTime;
    Serial.print(F("OK;RUN_CYCLE_DONE;CYCLE_TIME_MS="));
    Serial.println(cycleDuration);

    sendStatus();
  }
}




// ====================================================
// [→ userInterfaceScreen.cpp]  TERMINAL MODE DISPLAY
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


// ====================================================
// [→ maintenance.cpp]  PYTHON TERMINAL SERIAL API
// ====================================================
// (Command documentation originally at lines 1992-2027 preserved inside section)
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


// ====================================================
// [→ main.ino]  SETUP & LOOP
// ====================================================
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
