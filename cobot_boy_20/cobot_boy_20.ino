#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
#include <AutoPID.h>
#include <MsTimer2.h>
#include <EEPROM.h>

// =====================================================
// PIN MAPPING
// =====================================================
const int PIN_SERVO_A = 5;
const int PIN_SERVO_B = 9;
const int PIN_SERVO_Z = 11;

const int PIN_GRIP_INA = 6;
const int PIN_GRIP_INB = 7;
const int PIN_GRIP_PWM = 3;

const int CURRENT_CLOSE_PIN = A4;
const int CURRENT_OPEN_PIN  = A5;

const int PIN_MAINTENANCE = 44;

const int BTN_PAUSE = 18;
const int BTN_START = 22;
const int BTN_INIT  = 24;

const int ENC_CLK = 40;
const int ENC_DT  = 42;
const int ENC_SW  = 46;

const int LED_RUN   = 30;
const int LED_PAUSE = 32;
const int LED_POWER = 34;
const int LED_INIT  = 36;

// =====================================================
// OLED CONFIG
// =====================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// =====================================================
// ROBOT KINEMATICS
// =====================================================
Servo servoA;
Servo servoB;
Servo servoZ;

float L1 = 150.0;
float L2 = 143.0;
float offsetX = 20.0;
float offsetZ = 75.0;

float degOffsetA = -20.0;
float degOffsetB = -10.0;
float degOffsetZ = 0.0;

float toolRadius = 40;
float objectRadius = 32;
float toolLength = 65.0 - (toolRadius - objectRadius);
float toolHeight = -45;

float maxStepA = 0.2;
float maxStepB = 0.2;
float maxStepZ = 0.5;

float executionSpeedFactor = 1.0;

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

// =====================================================
// CYCLE POSITIONS
// =====================================================
const int NB_CYCLES = 2;
const int NB_OBJECTS = 3;
const int NB_POSITIONS = 4;

const int OBJ_GOMME = 0;
const int OBJ_GOBELET = 1;
const int OBJ_CYLINDRE = 2;

const int POS_1 = 0;
const int POS_2 = 1;
const int POS_3 = 2;
const int POS_4 = 3;

Point3D basePositions[NB_CYCLES][NB_POSITIONS] = {
  {
    {155, 20, 37},
    {-155, 115, 37},
    {160, 150, 37},
    {-155, 20, 37}
  },
  {
    {155, 7, 37},
    {-155, 150, 87},
    {165, 116, 37},
    {-155, 20, 0}
  }
};

const char* positionNames[NB_CYCLES][NB_POSITIONS] = {
  {"Entree", "A", "B", "Sortie"},
  {"Input", "Machine A", "Machine B", "Output"}
};

// =====================================================
// HMI GLOBAL STATES
// =====================================================
enum ScreenState {
  SCREEN_HOME,
  SCREEN_INIT_MENU,
  SCREEN_CONFIG,
  SCREEN_SETTINGS,
  SCREEN_INFO,
  SCREEN_STATUS,
  SCREEN_CYCLE_MENU,
  SCREEN_OBJECT_MENU,
  SCREEN_MODE_MENU,
  SCREEN_COMBINE_MENU,
  SCREEN_READY
};

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
int currentObject    = 0;
int currentMode      = 1;
int currentCombine   = 0;
int currentPiece     = 0;

unsigned long bootTime = 0;
const unsigned long bootIgnoreButtonsMs = 1500;

bool previousButtonState[3] = {HIGH, HIGH, HIGH};
unsigned long lastDebounceTime[3] = {0, 0, 0};
const unsigned long debounceDelay = 50;

int lastEncCLK = HIGH;
unsigned long lastEncoderMoveTime = 0;
unsigned long lastEncoderButtonTime = 0;
const unsigned long encoderMoveDelay = 80;
const unsigned long encoderButtonDelay = 300;

unsigned long lastStatusUpdate = 0;
const unsigned long statusRefreshDelay = 500;

const char* homeMenu[] = {"Init", "Config", "Settings", "Info", "Status"};
const char* initMenu[] = {"Go Repos + Init", "Return"};
const char* configMenu[] = {"Choix Cycle", "Choix Objet", "Choix Mode", "Combine Cycle", "Retour"};
const char* cycleMenu[] = {"Cycle 1", "Cycle 2", "Retour"};
const char* objectMenu[] = {"Gomme", "Gobelet", "Cylindre", "Retour"};
const char* modeMenu[] = {"ECO", "Normal", "SPEED", "Retour"};
const char* combineMenu[] = {"1", "2", "3", "4", "5", "Retour"};

// =====================================================
// GRIPPER SETPOINTS
// IMPORTANT: declared here before EEPROM functions use them
// =====================================================
double consigneOuverture = 1.80;
double consigneFermeture = 1.40;

double seuilOuvertureMax = 2.30;
double seuilFermetureMax = 2.60;

// =====================================================
// EEPROM SETTINGS
// =====================================================
struct PositionOffset {
  float dx;
  float dy;
  float dz;
};

struct GripSettings {
  float openSetpoint;
  float closeSetpoint;
};

struct EEPROMData {
  unsigned long magic;
  PositionOffset offsets[NB_CYCLES][NB_OBJECTS][NB_POSITIONS];
  GripSettings grip[NB_OBJECTS];
  float savedSpeedFactor;
  float savedServoOffsetA;
  float savedServoOffsetB;
  float savedServoOffsetZ;
};

const unsigned long EEPROM_MAGIC = 20260427;
const int EEPROM_ADDRESS = 0;

EEPROMData eepromData;

int maintCycle = 0;
int maintObject = 0;
int maintPosition = 0;

float offsetStep = 1.0;
float gripStep = 0.05;
float speedStep = 0.1;
float servoOffsetStep = 1.0;

float constrainOffset(float value) {
  return constrain(value, -30.0, 30.0);
}

const char* getObjectNameByIndex(int obj) {
  if (obj == 0) return "Gomme";
  if (obj == 1) return "Gobelet";
  return "Cylindre";
}

const char* getCycleNameByIndex(int cyc) {
  if (cyc == 0) return "Cycle 1";
  return "Cycle 2";
}

void setDefaultEEPROMData() {
  eepromData.magic = EEPROM_MAGIC;

  for (int c = 0; c < NB_CYCLES; c++) {
    for (int o = 0; o < NB_OBJECTS; o++) {
      for (int p = 0; p < NB_POSITIONS; p++) {
        eepromData.offsets[c][o][p].dx = 0.0;
        eepromData.offsets[c][o][p].dy = 0.0;
        eepromData.offsets[c][o][p].dz = 0.0;
      }
    }
  }

  eepromData.grip[OBJ_GOMME].openSetpoint = 1.80;
  eepromData.grip[OBJ_GOMME].closeSetpoint = 1.30;

  eepromData.grip[OBJ_GOBELET].openSetpoint = 1.80;
  eepromData.grip[OBJ_GOBELET].closeSetpoint = 1.15;

  eepromData.grip[OBJ_CYLINDRE].openSetpoint = 1.80;
  eepromData.grip[OBJ_CYLINDRE].closeSetpoint = 1.40;

  eepromData.savedSpeedFactor = 1.0;
  eepromData.savedServoOffsetA = -20.0;
  eepromData.savedServoOffsetB = -10.0;
  eepromData.savedServoOffsetZ = 0.0;
}

void applySavedSettingsToRobot() {
  executionSpeedFactor = eepromData.savedSpeedFactor;

  if (executionSpeedFactor < 0.5 || executionSpeedFactor > 3.0) {
    executionSpeedFactor = 1.0;
  }

  degOffsetA = eepromData.savedServoOffsetA;
  degOffsetB = eepromData.savedServoOffsetB;
  degOffsetZ = eepromData.savedServoOffsetZ;
}

void applyGripperSettingsForObject(int objectIndex) {
  consigneOuverture = eepromData.grip[objectIndex].openSetpoint;
  consigneFermeture = eepromData.grip[objectIndex].closeSetpoint;
}

void applyFullConfiguration() {
  applySavedSettingsToRobot();
  applyGripperSettingsForObject(currentObject);
}

void saveSettingsToEEPROM() {
  eepromData.magic = EEPROM_MAGIC;
  eepromData.savedSpeedFactor = executionSpeedFactor;
  eepromData.savedServoOffsetA = degOffsetA;
  eepromData.savedServoOffsetB = degOffsetB;
  eepromData.savedServoOffsetZ = degOffsetZ;

  EEPROM.put(EEPROM_ADDRESS, eepromData);
  Serial.println(F("EEPROM saved successfully."));
}

void loadSettingsFromEEPROM() {
  EEPROM.get(EEPROM_ADDRESS, eepromData);

  if (eepromData.magic != EEPROM_MAGIC) {
    setDefaultEEPROMData();
    EEPROM.put(EEPROM_ADDRESS, eepromData);
  }

  applyFullConfiguration();
}

void resetEEPROMToDefaults() {
  setDefaultEEPROMData();
  EEPROM.put(EEPROM_ADDRESS, eepromData);
  applyFullConfiguration();

  Serial.println(F("EEPROM reset to default values."));
}

PositionOffset getOffset(int cycle, int object, int position) {
  return eepromData.offsets[cycle][object][position];
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

// =====================================================
// GRIPPER PID
// =====================================================
#define KP_GRIP 90.0
#define KI_GRIP 0.0
#define KD_GRIP 2.5

enum GripperState {
  GRIPPER_STOPPED,
  GRIPPER_OPENING,
  GRIPPER_CLOSING
};

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
  KD_GRIP
);

// =====================================================
// FORWARD DECLARATIONS
// =====================================================
void smartWait(unsigned long ms);
void handleButtons();
void moveToReposPosition();
void moveRectangular(float xf, float yf, float zf, float liftHeight, unsigned long tUp, unsigned long tXY, unsigned long tDown);
void performInitialization();
void executeCurrentScreen();
void handleApiSerial();
void sendStatus();
void sendConfig();
void ihmInitializeAction();
void ihmStartAction();
void ihmPauseToggleAction();
void enterTerminalMode();
void exitTerminalMode();
void drawTerminalModeScreen();

// =====================================================
// INTERRUPTS
// =====================================================
void pauseISR() {
  pauseInterruptRequest = true;
}

volatile bool tick10ms = false;

void timerISR() {
  tick10ms = true;
}

// =====================================================
// BUTTON LOGIC
// =====================================================
int getButtonIndex(int pin) {
  if (pin == BTN_PAUSE) return 0;
  if (pin == BTN_START) return 1;
  if (pin == BTN_INIT)  return 2;
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

// =====================================================
// TEXT HELPERS
// =====================================================
const char* getCycleText() {
  return (currentCycleType == 0) ? "Cycle 1" : "Cycle 2";
}

const char* getObjectText() {
  return getObjectNameByIndex(currentObject);
}

const char* getModeText() {
  if (currentMode == 0) return "ECO";
  if (currentMode == 1) return "Normal";
  return "SPEED";
}

int getCombineValue() {
  return currentCombine + 1;
}

const char* getRunStateText() {
  if (isInitializing) return "INITIALIZING";
  if (isPaused) return "PAUSED";
  if (isRunning) return "RUNNING";
  if (isInitialized) return "READY";
  return "NOT INIT";
}

// =====================================================
// OLED
// =====================================================
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

  display.println(F("STATUS"));
  display.println(F("----------------"));
  display.print(F("Cycle : ")); display.println(getCycleText());
  display.print(F("Objet : ")); display.println(getObjectText());
  display.print(F("Mode  : ")); display.println(getModeText());
  display.print(F("Piece : ")); display.print(currentPiece); display.print(F("/")); display.println(getCombineValue());
  display.print(F("State : ")); display.println(getRunStateText());

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

  display.println(F("READY"));
  display.println(F("----------------"));
  display.print(F("Cy: ")); display.println(getCycleText());
  display.print(F("Obj:")); display.println(getObjectText());
  display.print(F("Md: ")); display.println(getModeText());
  display.print(F("Nb: ")); display.println(getCombineValue());
  display.println(F("Start = run"));
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
    case SCREEN_HOME:         drawMenu("MENU", homeMenu, 5, menuIndex); break;
    case SCREEN_INIT_MENU:    drawMenu("INIT", initMenu, 2, menuIndex); break;
    case SCREEN_CONFIG:       drawMenu("CONFIG", configMenu, 5, menuIndex); break;
    case SCREEN_CYCLE_MENU:   drawMenu("CHOIX CYCLE", cycleMenu, 3, menuIndex); break;
    case SCREEN_OBJECT_MENU:  drawMenu("CHOIX OBJET", objectMenu, 4, menuIndex); break;
    case SCREEN_MODE_MENU:    drawMenu("CHOIX MODE", modeMenu, 4, menuIndex); break;
    case SCREEN_COMBINE_MENU: drawMenu("COMBINE", combineMenu, 6, menuIndex); break;
    case SCREEN_STATUS:       drawStatusScreen(); break;
    case SCREEN_READY:        drawReadyScreen(); break;
    default: break;
  }
}

void refreshStatusBetweenMoves() {
  if (screenState == SCREEN_STATUS) {
    if (terminalModeActive) drawTerminalModeScreen();
    else drawStatusScreen();
  }
}

// =====================================================
// LEDS
// =====================================================
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

void configChanged() {
  isInitialized = false;
  isInitializing = false;
  isRunning = false;
  isPaused = false;
  currentPiece = 0;

  applyFullConfiguration();
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

// =====================================================
// HMI
// =====================================================
int getMenuCount(ScreenState s) {
  if (s == SCREEN_HOME) return 5;
  if (s == SCREEN_INIT_MENU) return 2;
  if (s == SCREEN_CONFIG) return 5;
  if (s == SCREEN_CYCLE_MENU) return 3;
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
        drawInfoMessage("SETTINGS", "Use Config menu", "for cycle setup");
        smartWait(1000);
        screenState = SCREEN_HOME;
      }
      else if (menuIndex == 3) {
        drawInfoMessage("INFO", "Encoder: nav/select", "Init before Start");
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
      else if (menuIndex == 1) screenState = SCREEN_OBJECT_MENU;
      else if (menuIndex == 2) screenState = SCREEN_MODE_MENU;
      else if (menuIndex == 3) screenState = SCREEN_COMBINE_MENU;
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

    case SCREEN_OBJECT_MENU:
      if (menuIndex < 3) {
        currentObject = menuIndex;
        configChanged();
      }
      screenState = SCREEN_CONFIG;
      menuIndex = 1;
      break;

    case SCREEN_MODE_MENU:
      if (menuIndex < 3) {
        currentMode = menuIndex;
        configChanged();
      }
      screenState = SCREEN_CONFIG;
      menuIndex = 2;
      break;

    case SCREEN_COMBINE_MENU:
      if (menuIndex < 5) {
        currentCombine = menuIndex;
        configChanged();
      }
      screenState = SCREEN_CONFIG;
      menuIndex = 3;
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

// =====================================================
// GRIPPER PID FUNCTIONS
// =====================================================
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

  double voltageOpen  = readVoltage(CURRENT_OPEN_PIN);
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

// =====================================================
// SYSTEM YIELD
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
  }
  else if (isPaused) {
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
// KINEMATICS
// =====================================================
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

  if (cmdA >= 0.0 && cmdA <= 180.0 &&
      cmdB >= 0.0 && cmdB <= 180.0 &&
      cmdZ >= 0.0 && cmdZ <= 180.0) {
    out.a = cmdA;
    out.b = cmdB;
    out.z = cmdZ;
    out.reachable = true;
  }

  return out;
}

// =====================================================
// MOVEMENT
// =====================================================
void moveJointsSmoothTo(JointCmd target, unsigned long dtMs = 5) {
  if (!target.reachable) return;

  bool done = false;

  while (!done) {
    systemYield();

    currentCmdA = limitStep(currentCmdA, target.a, maxStepA);
    currentCmdB = limitStep(currentCmdB, target.b, maxStepB);
    currentCmdZ = limitStep(currentCmdZ, target.z, maxStepZ);

    writeServosMicroseconds(currentCmdA, currentCmdB, currentCmdZ);

    done = abs(currentCmdA - target.a) < 0.05 &&
           abs(currentCmdB - target.b) < 0.05 &&
           abs(currentCmdZ - target.z) < 0.05;

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
  }
  else if (u < (1.0 - alpha)) {
    s = vmax * (alpha / 2.0 + (u - alpha));
  }
  else {
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

  refreshStatusBetweenMoves();
}

void moveRectangular(float xf, float yf, float zf, float liftHeight, unsigned long tUp, unsigned long tXY, unsigned long tDown) {
  Point3D pStart = {currentX, currentY, currentZ};

  float zLift = max(currentZ, zf) + liftHeight;

  Point3D pUp  = {currentX, currentY, zLift};
  Point3D pMid = {xf, yf, zLift};
  Point3D pEnd = {xf, yf, zf};

  moveLinearSegment(pStart, pUp,  tUp, 20);
  moveLinearSegment(pUp,    pMid, tXY, 20);
  moveLinearSegment(pMid,   pEnd, tDown, 20);
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


// =====================================================
// INITIALIZATION ACTION
// =====================================================
void performInitialization() {
  isRunning = false;
  isPaused = false;
  pauseMoveInProgress = false;

  loadSettingsFromEEPROM();
  applyFullConfiguration();

  drawInfoMessage("Init...", "Going to repos");

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
// MAINTENANCE PRINT FUNCTIONS
// =====================================================
void printCurrentConfiguration() {
  Serial.println();
  Serial.println(F("========== CURRENT CONFIGURATION =========="));
  Serial.print(F("Cycle selected: ")); Serial.println(getCycleText());
  Serial.print(F("Object selected: ")); Serial.println(getObjectText());
  Serial.print(F("Mode: ")); Serial.println(getModeText());
  Serial.print(F("Number of pieces: ")); Serial.println(getCombineValue());

  Serial.print(F("Current X: ")); Serial.println(currentX);
  Serial.print(F("Current Y: ")); Serial.println(currentY);
  Serial.print(F("Current Z: ")); Serial.println(currentZ);

  Serial.print(F("Execution speed factor: ")); Serial.println(executionSpeedFactor);

  Serial.print(F("Open setpoint: ")); Serial.println(eepromData.grip[currentObject].openSetpoint);
  Serial.print(F("Close setpoint: ")); Serial.println(eepromData.grip[currentObject].closeSetpoint);

  Serial.print(F("Servo offset A: ")); Serial.println(degOffsetA);
  Serial.print(F("Servo offset B: ")); Serial.println(degOffsetB);
  Serial.print(F("Servo offset Z: ")); Serial.println(degOffsetZ);
  Serial.println(F("==========================================="));
}

void printMaintenanceNotice() {
  Serial.println();
  Serial.println(F("========== NOTICE D'UTILISATION =========="));
  Serial.println(F("Mode normal: ordinateur debranche."));
  Serial.println(F("Mode maintenance: cavalier entre PIN 44 et GND puis reset."));
  Serial.println(F("Utiliser le moniteur serie a 9600 bauds."));
  Serial.println(F("La console permet de modifier la configuration comme l'IHM."));
  Serial.println(F("Elle permet aussi de tester les servos, LEDs, pince et position repos."));
  Serial.println(F("Les offsets sont stockes dans l'EEPROM."));
  Serial.println(F("Pour quitter: retirer le cavalier puis reset Arduino."));
  Serial.println(F("=========================================="));
}

void printMainMaintenanceMenu() {
  Serial.println();
  Serial.println(F("========== MAINTENANCE MAIN MENU =========="));

  Serial.println(F("[Configuration]"));
  Serial.println(F("c  - Change cycle"));
  Serial.println(F("j  - Change object"));
  Serial.println(F("e  - Change mode ECO / Normal / SPEED"));
  Serial.println(F("n  - Change number of pieces"));
  Serial.println(F("f/s - Faster / slower execution factor"));

  Serial.println();
  Serial.println(F("[Manual Robot Control]"));
  Serial.println(F("1 - Manual Servo A with encoder"));
  Serial.println(F("2 - Manual Servo B with encoder"));
  Serial.println(F("3 - Manual Servo Z with encoder"));
  Serial.println(F("8 - Go to REPOS position"));

  Serial.println();
  Serial.println(F("[Gripper]"));
  Serial.println(F("4 - Open gripper PID"));
  Serial.println(F("5 - Close gripper PID"));
  Serial.println(F("6 - Stop gripper"));

  Serial.println();
  Serial.println(F("[Offset Tables]"));
  Serial.println(F("T1 - Open full offset table for Cycle 1"));
  Serial.println(F("T2 - Open full offset table for Cycle 2"));

  Serial.println();
  Serial.println(F("[System]"));
  Serial.println(F("7 - Test LEDs"));
  Serial.println(F("9 - Print current configuration"));
  Serial.println(F("w - Save EEPROM"));
  Serial.println(F("l - Load EEPROM"));
  Serial.println(F("d - Reset EEPROM"));
  Serial.println(F("h - Notice"));
  Serial.println(F("m - Show menu"));

  Serial.println(F("==========================================="));
  Serial.println(F("Enter command:"));
}

void printSelectedMaintenanceTarget() {
  Serial.println();
  Serial.println(F("----- SELECTED OFFSET TARGET -----"));
  Serial.print(F("Cycle: ")); Serial.println(getCycleNameByIndex(maintCycle));
  Serial.print(F("Object: ")); Serial.println(getObjectNameByIndex(maintObject));
  Serial.print(F("Position: ")); Serial.println(positionNames[maintCycle][maintPosition]);

  PositionOffset off = eepromData.offsets[maintCycle][maintObject][maintPosition];
  Point3D base = basePositions[maintCycle][maintPosition];
  Point3D finalPos = getCorrectedPosition(maintCycle, maintObject, maintPosition);

  Serial.print(F("Base:  "));
  Serial.print(base.x); Serial.print(F(", "));
  Serial.print(base.y); Serial.print(F(", "));
  Serial.println(base.z);

  Serial.print(F("Offset: "));
  Serial.print(off.dx); Serial.print(F(", "));
  Serial.print(off.dy); Serial.print(F(", "));
  Serial.println(off.dz);

  Serial.print(F("Final: "));
  Serial.print(finalPos.x); Serial.print(F(", "));
  Serial.print(finalPos.y); Serial.print(F(", "));
  Serial.println(finalPos.z);

  Serial.println(F("----------------------------------"));
}

void printPositionTable(int cycle, int object) {
  Serial.println();
  Serial.print(F("========== "));
  Serial.print(getCycleNameByIndex(cycle));
  Serial.print(F(" OFFSET TABLE / Object: "));
  Serial.print(getObjectNameByIndex(object));
  Serial.println(F(" =========="));

  Serial.println(F("Position       BaseX   BaseY   BaseZ   OffX   OffY   OffZ   FinalX   FinalY   FinalZ"));

  for (int p = 0; p < NB_POSITIONS; p++) {
    Point3D base = basePositions[cycle][p];
    PositionOffset off = eepromData.offsets[cycle][object][p];
    Point3D finalPos = getCorrectedPosition(cycle, object, p);

    Serial.print(positionNames[cycle][p]);

    int nameLen = strlen(positionNames[cycle][p]);
    for (int s = nameLen; s < 15; s++) Serial.print(F(" "));

    Serial.print(base.x); Serial.print(F("   "));
    Serial.print(base.y); Serial.print(F("   "));
    Serial.print(base.z); Serial.print(F("   "));

    Serial.print(off.dx); Serial.print(F("   "));
    Serial.print(off.dy); Serial.print(F("   "));
    Serial.print(off.dz); Serial.print(F("   "));

    Serial.print(finalPos.x); Serial.print(F("   "));
    Serial.print(finalPos.y); Serial.print(F("   "));
    Serial.println(finalPos.z);
  }

  Serial.println();
  Serial.print(F("Open setpoint: "));
  Serial.println(eepromData.grip[object].openSetpoint);

  Serial.print(F("Close setpoint: "));
  Serial.println(eepromData.grip[object].closeSetpoint);

  Serial.print(F("Servo offset A: "));
  Serial.println(eepromData.savedServoOffsetA);

  Serial.print(F("Servo offset B: "));
  Serial.println(eepromData.savedServoOffsetB);

  Serial.print(F("Servo offset Z: "));
  Serial.println(eepromData.savedServoOffsetZ);

  Serial.println(F("=========================================================="));
}

void printCycleTableMenu(int cycle) {
  maintCycle = cycle;

  Serial.println();
  Serial.print(F("========== "));
  Serial.print(getCycleNameByIndex(cycle));
  Serial.println(F(" OFFSET TABLE MENU =========="));

  Serial.println(F("[Navigation]"));
  Serial.println(F("j - Change object"));
  Serial.println(F("k - Change position"));

  Serial.println();
  Serial.println(F("[Prehenseur position offsets]"));
  Serial.println(F("x/X - Increase/decrease X offset"));
  Serial.println(F("y/Y - Increase/decrease Y offset"));
  Serial.println(F("r/R - Increase/decrease Z offset"));

  Serial.println();
  Serial.println(F("[Servo angle offsets]"));
  Serial.println(F("a/A - Increase/decrease Servo A offset"));
  Serial.println(F("b/B - Increase/decrease Servo B offset"));
  Serial.println(F("z/Z - Increase/decrease Servo Z offset"));

  Serial.println();
  Serial.println(F("[Gripper / motor settings]"));
  Serial.println(F("p/o - Increase/decrease close setpoint"));
  Serial.println(F("i/u - Increase/decrease open setpoint"));

  Serial.println();
  Serial.println(F("[Actions]"));
  Serial.println(F("t - Print this cycle table"));
  Serial.println(F("w - Save EEPROM"));
  Serial.println(F("q - Return to main menu"));

  Serial.println(F("=========================================="));
  printSelectedMaintenanceTarget();
  printPositionTable(maintCycle, maintObject);
  Serial.println(F("Enter command:"));
}

// =====================================================
// MAINTENANCE EDIT FUNCTIONS
// =====================================================
void updateSelectedOffset(char axis, float delta) {
  PositionOffset* off = &eepromData.offsets[maintCycle][maintObject][maintPosition];

  if (axis == 'x') off->dx = constrainOffset(off->dx + delta);
  if (axis == 'y') off->dy = constrainOffset(off->dy + delta);
  if (axis == 'z') off->dz = constrainOffset(off->dz + delta);

  printSelectedMaintenanceTarget();
}

void updateServoOffset(char servo, float delta) {
  if (servo == 'a') {
    degOffsetA += delta;
    eepromData.savedServoOffsetA = degOffsetA;
    Serial.print(F("Servo offset A = "));
    Serial.println(degOffsetA);
  }

  if (servo == 'b') {
    degOffsetB += delta;
    eepromData.savedServoOffsetB = degOffsetB;
    Serial.print(F("Servo offset B = "));
    Serial.println(degOffsetB);
  }

  if (servo == 'z') {
    degOffsetZ += delta;
    eepromData.savedServoOffsetZ = degOffsetZ;
    Serial.print(F("Servo offset Z = "));
    Serial.println(degOffsetZ);
  }
}

void updateGripSetpoint(bool closeSetpoint, float delta) {
  if (closeSetpoint) {
    eepromData.grip[maintObject].closeSetpoint += delta;

    if (eepromData.grip[maintObject].closeSetpoint > seuilFermetureMax) {
      eepromData.grip[maintObject].closeSetpoint = seuilFermetureMax;
    }

    if (eepromData.grip[maintObject].closeSetpoint < 0.5) {
      eepromData.grip[maintObject].closeSetpoint = 0.5;
    }

    consigneFermeture = eepromData.grip[maintObject].closeSetpoint;

    Serial.print(F("Close setpoint for "));
    Serial.print(getObjectNameByIndex(maintObject));
    Serial.print(F(" = "));
    Serial.println(eepromData.grip[maintObject].closeSetpoint);
  } else {
    eepromData.grip[maintObject].openSetpoint += delta;

    if (eepromData.grip[maintObject].openSetpoint > seuilOuvertureMax) {
      eepromData.grip[maintObject].openSetpoint = seuilOuvertureMax;
    }

    if (eepromData.grip[maintObject].openSetpoint < 0.5) {
      eepromData.grip[maintObject].openSetpoint = 0.5;
    }

    consigneOuverture = eepromData.grip[maintObject].openSetpoint;

    Serial.print(F("Open setpoint for "));
    Serial.print(getObjectNameByIndex(maintObject));
    Serial.print(F(" = "));
    Serial.println(eepromData.grip[maintObject].openSetpoint);
  }
}

void testLEDsMaintenance() {
  Serial.println(F("Testing LEDs..."));

  digitalWrite(LED_RUN, HIGH);
  delay(300);
  digitalWrite(LED_PAUSE, HIGH);
  delay(300);
  digitalWrite(LED_POWER, HIGH);
  delay(300);
  digitalWrite(LED_INIT, HIGH);
  delay(700);

  digitalWrite(LED_RUN, LOW);
  digitalWrite(LED_PAUSE, LOW);
  digitalWrite(LED_POWER, HIGH);
  digitalWrite(LED_INIT, HIGH);

  Serial.println(F("LED test finished."));
}

// =====================================================
// MAINTENANCE MODE
// =====================================================
enum MaintenanceConsoleState {
  MAINT_MAIN,
  MAINT_TABLE_CYCLE1,
  MAINT_TABLE_CYCLE2,
  MAINT_SERVO_A,
  MAINT_SERVO_B,
  MAINT_SERVO_Z
};

MaintenanceConsoleState maintConsoleState = MAINT_MAIN;

void updateManualServoFromEncoder() {
  int currentCLK = digitalRead(ENC_CLK);

  if (currentCLK != lastEncCLK && currentCLK == LOW) {
    if (millis() - lastEncoderMoveTime > encoderMoveDelay) {
      int dir = (digitalRead(ENC_DT) != currentCLK) ? 1 : -1;

      if (maintConsoleState == MAINT_SERVO_A) {
        currentCmdA = constrain(currentCmdA + dir * 2.0, 0.0, 180.0);
        writeServosMicroseconds(currentCmdA, currentCmdB, currentCmdZ);
        Serial.print(F("Servo A = "));
        Serial.println(currentCmdA);
      }
      else if (maintConsoleState == MAINT_SERVO_B) {
        currentCmdB = constrain(currentCmdB + dir * 2.0, 0.0, 180.0);
        writeServosMicroseconds(currentCmdA, currentCmdB, currentCmdZ);
        Serial.print(F("Servo B = "));
        Serial.println(currentCmdB);
      }
      else if (maintConsoleState == MAINT_SERVO_Z) {
        currentCmdZ = constrain(currentCmdZ + dir * 2.0, 0.0, 180.0);
        writeServosMicroseconds(currentCmdA, currentCmdB, currentCmdZ);
        Serial.print(F("Servo Z = "));
        Serial.println(currentCmdZ);
      }

      lastEncoderMoveTime = millis();
    }
  }

  lastEncCLK = currentCLK;

  if (digitalRead(ENC_SW) == LOW) {
    if (millis() - lastEncoderButtonTime > encoderButtonDelay) {
      maintConsoleState = MAINT_MAIN;
      Serial.println(F("Manual servo control stopped."));
      printMainMaintenanceMenu();
      lastEncoderButtonTime = millis();
    }
  }
}

String readSerialCommand() {
  String cmd = "";

  if (Serial.available()) {
    cmd = Serial.readStringUntil('\n');
    cmd.trim();
  }

  return cmd;
}

void processMainMaintenanceCommand(String cmd) {
  if (cmd == "1") {
    maintConsoleState = MAINT_SERVO_A;
    Serial.println(F("Manual Servo A selected. Turn encoder. Press encoder or type q to exit."));
  }
  else if (cmd == "2") {
    maintConsoleState = MAINT_SERVO_B;
    Serial.println(F("Manual Servo B selected. Turn encoder. Press encoder or type q to exit."));
  }
  else if (cmd == "3") {
    maintConsoleState = MAINT_SERVO_Z;
    Serial.println(F("Manual Servo Z selected. Turn encoder. Press encoder or type q to exit."));
  }
  else if (cmd == "4") {
    applyGripperSettingsForObject(maintObject);
    Serial.println(F("Opening gripper with PID..."));
    openGripper();
  }
  else if (cmd == "5") {
    applyGripperSettingsForObject(maintObject);
    Serial.println(F("Closing gripper with PID..."));
    closeGripper();
  }
  else if (cmd == "6") {
    Serial.println(F("Stopping gripper..."));
    stopGripper();
  }
  else if (cmd == "7") {
    testLEDsMaintenance();
  }
  else if (cmd == "8") {
    Serial.println(F("Going to REPOS position..."));
    moveToReposPosition();
    Serial.println(F("REPOS movement finished."));
  }
  else if (cmd == "9") {
    printCurrentConfiguration();
  }
  else if (cmd == "T1" || cmd == "t1") {
    maintConsoleState = MAINT_TABLE_CYCLE1;
    printCycleTableMenu(0);
  }
  else if (cmd == "T2" || cmd == "t2") {
    maintConsoleState = MAINT_TABLE_CYCLE2;
    printCycleTableMenu(1);
  }
  else if (cmd == "c") {
    currentCycleType = (currentCycleType + 1) % NB_CYCLES;
    maintCycle = currentCycleType;
    configChanged();
    Serial.print(F("Cycle selected: "));
    Serial.println(getCycleText());
  }
  else if (cmd == "j") {
    currentObject = (currentObject + 1) % NB_OBJECTS;
    maintObject = currentObject;
    configChanged();
    Serial.print(F("Object selected: "));
    Serial.println(getObjectText());
  }
  else if (cmd == "e") {
    currentMode = (currentMode + 1) % 3;
    configChanged();
    Serial.print(F("Mode selected: "));
    Serial.println(getModeText());
  }
  else if (cmd == "n") {
    currentCombine = (currentCombine + 1) % 5;
    configChanged();
    Serial.print(F("Number of pieces: "));
    Serial.println(getCombineValue());
  }
  else if (cmd == "f") {
    executionSpeedFactor -= speedStep;
    if (executionSpeedFactor < 0.5) executionSpeedFactor = 0.5;
    eepromData.savedSpeedFactor = executionSpeedFactor;
    Serial.print(F("Execution speed factor: "));
    Serial.println(executionSpeedFactor);
  }
  else if (cmd == "s") {
    executionSpeedFactor += speedStep;
    if (executionSpeedFactor > 3.0) executionSpeedFactor = 3.0;
    eepromData.savedSpeedFactor = executionSpeedFactor;
    Serial.print(F("Execution speed factor: "));
    Serial.println(executionSpeedFactor);
  }
  else if (cmd == "w" || cmd == "W") {
    saveSettingsToEEPROM();
  }
  else if (cmd == "l" || cmd == "L") {
    loadSettingsFromEEPROM();
    applyFullConfiguration();
    Serial.println(F("EEPROM loaded."));
    printCurrentConfiguration();
  }
  else if (cmd == "d" || cmd == "D") {
    resetEEPROMToDefaults();
    applyFullConfiguration();
    printCurrentConfiguration();
  }
  else if (cmd == "h" || cmd == "H") {
    printMaintenanceNotice();
  }
  else if (cmd == "m" || cmd == "M") {
    printMainMaintenanceMenu();
  }
  else {
    Serial.println(F("Unknown command. Type m to show menu."));
  }
}

void processTableMaintenanceCommand(String cmd) {
  if (cmd == "q" || cmd == "Q") {
    maintConsoleState = MAINT_MAIN;
    printMainMaintenanceMenu();
  }
  else if (cmd == "j") {
    maintObject = (maintObject + 1) % NB_OBJECTS;
    currentObject = maintObject;
    applyGripperSettingsForObject(currentObject);
    printSelectedMaintenanceTarget();
  }
  else if (cmd == "k") {
    maintPosition = (maintPosition + 1) % NB_POSITIONS;
    printSelectedMaintenanceTarget();
  }
  else if (cmd == "x") updateSelectedOffset('x', offsetStep);
  else if (cmd == "X") updateSelectedOffset('x', -offsetStep);
  else if (cmd == "y") updateSelectedOffset('y', offsetStep);
  else if (cmd == "Y") updateSelectedOffset('y', -offsetStep);
  else if (cmd == "r") updateSelectedOffset('z', offsetStep);
  else if (cmd == "R") updateSelectedOffset('z', -offsetStep);
  else if (cmd == "a") updateServoOffset('a', servoOffsetStep);
  else if (cmd == "A") updateServoOffset('a', -servoOffsetStep);
  else if (cmd == "b") updateServoOffset('b', servoOffsetStep);
  else if (cmd == "B") updateServoOffset('b', -servoOffsetStep);
  else if (cmd == "z") updateServoOffset('z', servoOffsetStep);
  else if (cmd == "Z") updateServoOffset('z', -servoOffsetStep);
  else if (cmd == "p") updateGripSetpoint(true, gripStep);
  else if (cmd == "o") updateGripSetpoint(true, -gripStep);
  else if (cmd == "i") updateGripSetpoint(false, gripStep);
  else if (cmd == "u") updateGripSetpoint(false, -gripStep);
  else if (cmd == "t" || cmd == "T") printPositionTable(maintCycle, maintObject);
  else if (cmd == "w" || cmd == "W") saveSettingsToEEPROM();
  else {
    Serial.println(F("Unknown table command. Type q to return to main menu."));
  }
}

void maintenanceMode() {
  drawInfoMessage("MAINTENANCE", "Moving to REPOS", "Use Serial");

  loadSettingsFromEEPROM();
  applyFullConfiguration();

  stopGripper();

  digitalWrite(LED_POWER, HIGH);
  digitalWrite(LED_INIT, HIGH);
  digitalWrite(LED_RUN, LOW);
  digitalWrite(LED_PAUSE, LOW);

  Serial.println();
  Serial.println(F("Entering maintenance mode..."));
  Serial.println(F("Loading EEPROM settings..."));

  initializeFromCartesianPosition(initX, initY, initZ);

  Serial.println(F("Robot is now at REPOS position."));

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("MAINTENANCE"));
  display.println(F("----------------"));
  display.println(F("Robot at REPOS"));
  display.println(F("Use Serial"));
  display.println(F("9600 bauds"));
  display.display();

  maintCycle = currentCycleType;
  maintObject = currentObject;
  maintPosition = 0;
  maintConsoleState = MAINT_MAIN;

  printCurrentConfiguration();
  printMaintenanceNotice();
  printMainMaintenanceMenu();

  while (true) {
    if (tick10ms) {
      tick10ms = false;
      updateGripperPID();
    }

    if (maintConsoleState == MAINT_SERVO_A ||
        maintConsoleState == MAINT_SERVO_B ||
        maintConsoleState == MAINT_SERVO_Z) {
      updateManualServoFromEncoder();
    }

    String cmd = readSerialCommand();

    if (cmd.length() > 0) {
      if (maintConsoleState == MAINT_SERVO_A ||
          maintConsoleState == MAINT_SERVO_B ||
          maintConsoleState == MAINT_SERVO_Z) {
        if (cmd == "q" || cmd == "Q") {
          maintConsoleState = MAINT_MAIN;
          Serial.println(F("Manual servo control stopped."));
          printMainMaintenanceMenu();
        } else {
          Serial.println(F("Turn encoder to move servo. Press encoder or type q to exit."));
        }
      }
      else if (maintConsoleState == MAINT_TABLE_CYCLE1 ||
               maintConsoleState == MAINT_TABLE_CYCLE2) {
        processTableMaintenanceCommand(cmd);
      }
      else {
        processMainMaintenanceCommand(cmd);
      }

      Serial.println(F("Enter command:"));
    }

    delay(5);
  }
}

// =====================================================
// SHARED IHM ACTIONS
// These functions are used by BOTH:
// 1) the physical Arduino buttons
// 2) the Python terminal buttons over Serial
// This keeps INIT / START / PAUSE exactly the same.
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
  else drawInfoMessage("Init...", "Going to repos");
  sendStatus();

  stopGripper();

  if (!servoA.attached()) servoA.attach(PIN_SERVO_A);
  if (!servoB.attached()) servoB.attach(PIN_SERVO_B);
  if (!servoZ.attached()) servoZ.attach(PIN_SERVO_Z);

  loadSettingsFromEEPROM();
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
  smartWait(700);
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
    drawInfoMessage("ERROR", "Initialize first");
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

  loadSettingsFromEEPROM();
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

// =====================================================
// HANDLE BUTTONS
// =====================================================
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

// =====================================================
// SPEED HELPER
// =====================================================
void getCycleTimes(unsigned long &tUp, unsigned long &tXY, unsigned long &tDown) {
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

void moveToEEPROMPositionForObject(int cycle, int objectIndex, int position, float liftHeight, unsigned long tUp, unsigned long tXY, unsigned long tDown) {
  Point3D target = getCorrectedPosition(cycle, objectIndex, position);
  moveRectangular(target.x, target.y, target.z, liftHeight, tUp, tXY, tDown);
}

// Kept for compatibility with old maintenance/test code.
void moveToEEPROMPosition(int cycle, int position, float liftHeight, unsigned long tUp, unsigned long tXY, unsigned long tDown) {
  moveToEEPROMPositionForObject(cycle, currentObject, position, liftHeight, tUp, tXY, tDown);
}

void prepareObjectCycle(int objectIndex) {
  currentObject = objectIndex;
  loadSettingsFromEEPROM();
  applySavedSettingsToRobot();
  applyGripperSettingsForObject(objectIndex);
}

// =====================================================
// GENERIC SEQUENCE CYCLE 1
// The object is passed in so offsets + gripper setpoints are object-specific.
// =====================================================
void SequenceCycle1ForObject(int objectIndex) {
  prepareObjectCycle(objectIndex);

  unsigned long tUp;
  unsigned long tXY;
  unsigned long tDown;

  getCycleTimes(tUp, tXY, tDown);

  openGripper();
  smartWait(500);

  moveToEEPROMPositionForObject(0, objectIndex, POS_1, 100, tUp, tXY, tDown);

  closeGripper();
  smartWait(2000);

  moveToEEPROMPositionForObject(0, objectIndex, POS_2, 100, tUp, tXY, tDown);
  smartWait(3000);

  moveToEEPROMPositionForObject(0, objectIndex, POS_3, 100, tUp, tXY, tDown);
  smartWait(3000);

  moveToEEPROMPositionForObject(0, objectIndex, POS_4, 100, tUp, tXY, tDown);

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

  moveRectangular(initX, initY, initZ, 100, tUp, tXY, tDown);

  openGripper();
  smartWait(500);

  moveToEEPROMPositionForObject(1, objectIndex, POS_1, 100, tUp, tXY, tDown);

  closeGripper();
  smartWait(2000);

  moveToEEPROMPositionForObject(1, objectIndex, POS_2, 100, tUp, tXY, tDown);

  openGripper();
  smartWait(2000);

  moveRectangular(initX, initY, initZ, 100, tUp, tXY, tDown);

  smartWait(5000);

  moveToEEPROMPositionForObject(1, objectIndex, POS_2, 100, tUp, tXY, tDown);

  closeGripper();
  smartWait(2000);

  moveToEEPROMPositionForObject(1, objectIndex, POS_3, 100, tUp, tXY, tDown);

  openGripper();
  smartWait(2000);

  moveRectangular(initX, initY, initZ, 100, tUp, tXY, tDown);

  smartWait(5000);

  moveToEEPROMPositionForObject(1, objectIndex, POS_3, 100, tUp, tXY, tDown);

  closeGripper();
  smartWait(2000);

  moveToEEPROMPositionForObject(1, objectIndex, POS_4, 100, tUp, tXY, tDown);

  openGripper();
  smartWait(2000);

  moveRectangular(initX, initY, initZ, 100, tUp, tXY, tDown);

  closeGripper();
  smartWait(1000);
}

// =====================================================
// OBJECT METHODS
// These are the three methods requested: one for each object.
// Each method uses the selected cycle type but applies its own object offsets
// and gripper EEPROM settings.
// =====================================================
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
      runSelectedObjectCycle();

      currentPiece++;

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
    sendStatus();
  }
}



// =====================================================
// PYTHON TERMINAL MODE DISPLAY
// This mode is activated automatically when the Python
// terminal sends PING or any API command. No jumper needed.
// =====================================================
void drawTerminalModeScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.println(F("MAINTENANCE"));
  display.println(F("TERMINAL MODE"));
  display.println(F("----------------"));
  display.println(F("Python connected"));
  display.println(F("Use PC console"));
  display.println(F("OLED locked"));
  display.println(F("No jumper needed"));

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

// =====================================================
// PYTHON TERMINAL SERIAL API
// This is what makes the Python INIT / PLAY / PAUSE buttons work.
// Commands expected from Python:
//   TERMINAL_ON
//   TERMINAL_OFF
//   PING
//   GET_STATUS
//   GET_CONFIG
//   SET_CYCLE 0/1
//   SET_OBJECT 0/1/2
//   SET_MODE 0/1/2
//   SET_PIECES 1..5
//   SET_SPEED 0.50..3.00
//   INITIALIZE
//   RUN_CYCLE
//   PAUSE_TOGGLE
//   GO_REPOS
//   SERVO_SET A/B/Z angle
//   SERVO_STEP A/B/Z delta
//   BUTTON_TEST_ON / BUTTON_TEST_OFF
//   LED_SET INIT/RUN/PAUSE/POWER 0/1
//   LED_PULSE INIT/RUN/PAUSE/POWER
// =====================================================
String apiBuffer = "";

void sendStatus() {
  Serial.print(F("STATUS;"));
  Serial.print(F("STATE=")); Serial.print(getRunStateText()); Serial.print(F(";"));
  Serial.print(F("INIT=")); Serial.print(isInitialized ? 1 : 0); Serial.print(F(";"));
  Serial.print(F("INITIALIZING=")); Serial.print(isInitializing ? 1 : 0); Serial.print(F(";"));
  Serial.print(F("RUN=")); Serial.print(isRunning ? 1 : 0); Serial.print(F(";"));
  Serial.print(F("PAUSE=")); Serial.print(isPaused ? 1 : 0); Serial.print(F(";"));
  Serial.print(F("CYCLE=")); Serial.print(currentCycleType); Serial.print(F(";"));
  Serial.print(F("CYCLE_NAME=")); Serial.print(getCycleText()); Serial.print(F(";"));
  Serial.print(F("OBJECT=")); Serial.print(currentObject); Serial.print(F(";"));
  Serial.print(F("OBJECT_NAME=")); Serial.print(getObjectText()); Serial.print(F(";"));
  Serial.print(F("MODE=")); Serial.print(currentMode); Serial.print(F(";"));
  Serial.print(F("MODE_NAME=")); Serial.print(getModeText()); Serial.print(F(";"));
  Serial.print(F("PIECES=")); Serial.print(getCombineValue()); Serial.print(F(";"));
  Serial.print(F("CURRENT_PIECE=")); Serial.print(currentPiece); Serial.print(F(";"));
  Serial.print(F("SPEED_FACTOR=")); Serial.print(executionSpeedFactor, 2); Serial.print(F(";"));
  Serial.print(F("SERVO_A=")); Serial.print(currentCmdA, 1); Serial.print(F(";"));
  Serial.print(F("SERVO_B=")); Serial.print(currentCmdB, 1); Serial.print(F(";"));
  Serial.print(F("SERVO_Z=")); Serial.println(currentCmdZ, 1);
}

void sendConfig() {
  Serial.print(F("CONFIG;"));
  Serial.print(F("CYCLE=")); Serial.print(currentCycleType); Serial.print(F(";"));
  Serial.print(F("OBJECT=")); Serial.print(currentObject); Serial.print(F(";"));
  Serial.print(F("MODE=")); Serial.print(currentMode); Serial.print(F(";"));
  Serial.print(F("PIECES=")); Serial.print(getCombineValue()); Serial.print(F(";"));
  Serial.print(F("SPEED_FACTOR=")); Serial.println(executionSpeedFactor, 2);
}

String getArg(String cmd) {
  int sp = cmd.indexOf(' ');
  if (sp < 0) return "";
  String arg = cmd.substring(sp + 1);
  arg.trim();
  return arg;
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
  }
  else if (isPaused) {
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
  }
  else {
    Serial.println(F("ERR;PAUSE_NOT_AVAILABLE"));
    sendStatus();
  }
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
  }
  else if (upper == "TERMINAL_OFF") {
    terminalButtonTestMode = false;
    terminalLedTestMode = false;
    Serial.println(F("OK;TERMINAL_OFF"));
    exitTerminalMode();
    sendStatus();
  }
  else if (upper == "PING") {
    Serial.println(F("PONG"));
    sendStatus();
  }
  else if (upper == "GET_STATUS") {
    sendStatus();
  }
  else if (upper == "GET_CONFIG") {
    sendConfig();
  }
  else if (upper.startsWith("SET_CYCLE")) {
    int value = constrain(getArg(cmd).toInt(), 0, NB_CYCLES - 1);
    if (value != currentCycleType) {
      currentCycleType = value;
      terminalConfigChanged();
    }
    Serial.println(F("OK;SET_CYCLE"));
    sendStatus();
  }
  else if (upper.startsWith("SET_OBJECT")) {
    int value = constrain(getArg(cmd).toInt(), 0, NB_OBJECTS - 1);
    if (value != currentObject) {
      currentObject = value;
      terminalConfigChanged();
    }
    applyGripperSettingsForObject(currentObject);
    Serial.println(F("OK;SET_OBJECT"));
    sendStatus();
  }
  else if (upper.startsWith("SET_MODE")) {
    int value = constrain(getArg(cmd).toInt(), 0, 2);
    if (value != currentMode) {
      currentMode = value;
      terminalConfigChanged();
    }
    Serial.println(F("OK;SET_MODE"));
    sendStatus();
  }
  else if (upper.startsWith("SET_PIECES")) {
    int pieces = constrain(getArg(cmd).toInt(), 1, 5);
    int newCombine = pieces - 1;
    if (newCombine != currentCombine) {
      currentCombine = newCombine;
      terminalConfigChanged();
    }
    Serial.println(F("OK;SET_PIECES"));
    sendStatus();
  }
  else if (upper.startsWith("SET_SPEED")) {
    float value = constrain(getArg(cmd).toFloat(), 0.50, 3.00);
    if (abs(value - executionSpeedFactor) > 0.001) {
      executionSpeedFactor = value;
      eepromData.savedSpeedFactor = executionSpeedFactor;
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
  }
  else if (upper.startsWith("SERVO_SET")) {
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
  }
  else if (upper.startsWith("SERVO_STEP")) {
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
    Serial.print(F(";A=")); Serial.print(currentCmdA, 1);
    Serial.print(F(";B=")); Serial.print(currentCmdB, 1);
    Serial.print(F(";Z=")); Serial.println(currentCmdZ, 1);
    sendStatus();
  }


  else if (upper == "BUTTON_TEST_ON") {
    terminalButtonTestMode = true;
    isRunning = false;
    isPaused = false;
    pauseInterruptRequest = false;
    Serial.println(F("OK;BUTTON_TEST_ON"));
    sendStatus();
  }
  else if (upper == "BUTTON_TEST_OFF") {
    terminalButtonTestMode = false;
    Serial.println(F("OK;BUTTON_TEST_OFF"));
    sendStatus();
  }
  else if (upper.startsWith("LED_SET")) {
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
  }
  else if (upper == "LED_AUTO") {
    terminalLedTestMode = false;
    updateLEDs();
    Serial.println(F("OK;LED_AUTO"));
    sendStatus();
  }
  else if (upper.startsWith("LED_PULSE")) {
    pulseLedByName(getArg(cmd));
  }
  else if (upper == "TEST_INIT_LED") {
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
  }
  else if (upper == "FORCE_INIT_TEST") {
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
  }
  else if (upper == "INITIALIZE") {
    Serial.println(F("OK;INITIALIZE_START"));
    sendStatus();
    ihmInitializeAction();
    Serial.println(F("OK;INITIALIZE_DONE"));
    sendStatus();
  }
  else if (upper == "RUN_CYCLE") {
    ihmStartAction();
  }
  else if (upper == "PAUSE_TOGGLE") {
    ihmPauseToggleAction();
  }
  else {
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

// =====================================================
// SETUP & LOOP
// =====================================================
void setup() {
  delay(200);

  Serial.begin(9600);
  Serial.setTimeout(50);

  pinMode(PIN_MAINTENANCE, INPUT_PULLUP);

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
    while (true);
  }

  loadSettingsFromEEPROM();
  applyFullConfiguration();

  initializeFromCartesianPosition(initX, initY, initZ);

  if (digitalRead(PIN_MAINTENANCE) == LOW) {
    maintenanceMode();
  }

  attachInterrupt(digitalPinToInterrupt(BTN_PAUSE), pauseISR, FALLING);

  bootTime = millis();

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 20);
  display.println(F("BOOTING"));
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


