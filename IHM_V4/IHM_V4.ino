#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

// =====================================================
// PIN MAPPING
// =====================================================
const int PIN_SERVO_A = 5;
const int PIN_SERVO_B = 9;
const int PIN_SERVO_Z = 11;

const int PIN_GRIP_INA = 6;
const int PIN_GRIP_INB = 7;
const int PIN_GRIP_PWM = 3;

const int PIN_MAINTENANCE = 44; // pin 44 to GND = maintenance mode

const int BTN_PAUSE  = 18;
const int BTN_START  = 22;
const int BTN_INIT   = 24;
const int BTN_CYCLE  = 26;
const int BTN_SELECT = 28;

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
// ROBOT KINEMATICS & PARAMETERS
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
float degOffsetZ = -2.0;

float toolRadius = 40;
float objectRadius = 13;
float toolLength = 70.0 - (toolRadius - objectRadius);
float toolHeight = -50;

float maxStepA = 0.2;
float maxStepB = 0.2;
float maxStepZ = 0.5;

// Maintenance-adjustable parameters
float executionSpeedFactor = 1.0;
int gripPWM = 255;

// Repos / initialization position
float initX = 0.0;
float initY = 130.0;
float initZ = 120.0;

// Pause return position
float pauseReturnX = 0.0;
float pauseReturnY = 130.0;
float pauseReturnZ = 120.0;
bool pauseMoveInProgress = false;

float currentX = initX;
float currentY = initY;
float currentZ = initZ;

float currentCmdA = 0.0;
float currentCmdB = 0.0;
float currentCmdZ = 0.0;

struct Point3D { float x; float y; float z; };
struct JointCmd { float a; float b; float z; bool reachable; };

// =====================================================
// SCREEN & HMI STATES
// =====================================================
enum ScreenState {
  SCREEN_HOME, SCREEN_CONFIG, SCREEN_SETTINGS, SCREEN_INFO,
  SCREEN_STATUS, SCREEN_CYCLE_MENU, SCREEN_OBJECT_MENU,
  SCREEN_MODE_MENU, SCREEN_COMBINE_MENU, SCREEN_READY
};

ScreenState screenState = SCREEN_HOME;

bool isInitialized = false;
bool isRunning = false;
bool isPaused = false;
volatile bool pauseInterruptRequest = false;

int menuIndex = 0;
int currentCycleType = 0;
int currentObject    = 0;
int currentMode      = 0;
int currentCombine   = 0;
int currentPiece     = 0;

unsigned long bootTime = 0;
const unsigned long bootIgnoreButtonsMs = 1500;

bool previousButtonState[5] = {HIGH, HIGH, HIGH, HIGH, HIGH};
unsigned long lastDebounceTime[5] = {0, 0, 0, 0, 0};
const unsigned long debounceDelay = 50;

unsigned long lastStatusUpdate = 0;
const unsigned long statusRefreshDelay = 300;

const char* homeMenu[] = {"Config", "Settings", "Info", "Status"};
const char* configMenu[] = {"Choix Cycle", "Choix Objet", "Choix Mode", "Combine Cycle", "Retour"};
const char* cycleMenu[] = {"Cycle 1", "Cycle 2", "Retour"};
const char* objectMenu[] = {"Gomme", "Gobelet", "Cylindre", "Retour"};
const char* modeMenu[] = {"ECO", "Normal", "SPEED", "Retour"};
const char* combineMenu[] = {"1", "2", "3", "4", "5", "Retour"};

// =====================================================
// INTERRUPT & BUTTON LOGIC
// =====================================================
void pauseISR() {
  pauseInterruptRequest = true;
}

int getButtonIndex(int pin) {
  if (pin == BTN_PAUSE) return 0;
  if (pin == BTN_START) return 1;
  if (pin == BTN_INIT)  return 2;
  if (pin == BTN_CYCLE) return 3;
  if (pin == BTN_SELECT)return 4;
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

const char* getCycleText() {
  return (currentCycleType == 0) ? "Cycle 1" : "Cycle 2";
}

const char* getObjectText() {
  if (currentObject == 0) return "Gomme";
  if (currentObject == 1) return "Gobelet";
  return "Cylindre";
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
  if (isPaused) return "PAUSED";
  if (isRunning) return "RUNNING";
  if (isInitialized) return "READY";
  return "NOT INIT";
}

// =====================================================
// OLED DRAWING FUNCTIONS
// =====================================================
void drawMenu(const char* title, const char* items[], int itemCount, int selected) {
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
  display.println(F("Press Start"));

  display.display();
}

void refreshScreen() {
  switch (screenState) {
    case SCREEN_HOME:         drawMenu("MENU", homeMenu, 4, menuIndex); break;
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

// =====================================================
// LED & HMI EXECUTION LOGIC
// =====================================================
void updateLEDs() {
  digitalWrite(LED_RUN, (isRunning && !isPaused) ? HIGH : LOW);
  digitalWrite(LED_PAUSE, isPaused ? HIGH : LOW);
  digitalWrite(LED_POWER, HIGH);

  bool initStateActive = (screenState == SCREEN_READY) && isInitialized && !isRunning && !isPaused;
  digitalWrite(LED_INIT, initStateActive ? HIGH : LOW);
}

void configChanged() {
  isInitialized = false;
  isRunning = false;
  isPaused = false;
  currentPiece = 0;
}

void executeCurrentScreen() {
  switch (screenState) {
    case SCREEN_HOME:
      if (menuIndex == 0) screenState = SCREEN_CONFIG;
      if (menuIndex == 3) screenState = SCREEN_STATUS;
      menuIndex = 0;
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

int getMenuCount(ScreenState s) {
  if (s == SCREEN_HOME) return 4;
  if (s == SCREEN_CONFIG) return 5;
  if (s == SCREEN_CYCLE_MENU) return 3;
  if (s == SCREEN_OBJECT_MENU) return 4;
  if (s == SCREEN_MODE_MENU) return 4;
  if (s == SCREEN_COMBINE_MENU) return 6;
  return 0;
}

// =====================================================
// SYSTEM YIELD & NON-BLOCKING DELAY
// =====================================================
void handleButtons() {
  if (millis() - bootTime < bootIgnoreButtonsMs) return;

  if (buttonPressed(BTN_CYCLE)) {
    int count = getMenuCount(screenState);
    if (count > 0) {
      menuIndex = (menuIndex + 1) % count;
      refreshScreen();
    }
  }

  if (buttonPressed(BTN_SELECT)) {
    executeCurrentScreen();
  }

  if (buttonPressed(BTN_INIT) && !isRunning) {
    drawInfoMessage("Init...", "Going to repos");
    moveToReposPosition();
    //if (initializeFromCartesianPosition(initX, initY, initZ)) {
      smartWait(1000);
      isInitialized = true;
      currentPiece = 0;
      screenState = SCREEN_READY;
      refreshScreen();
    //} else {
      //drawInfoMessage("ERROR", "Repos unreachable");
      //delay(1200);
      //refreshScreen();
    //}
  }

  if (buttonPressed(BTN_START)) {
    if (!isInitialized) {
      drawInfoMessage("ERROR", "Initialize first");
      delay(1200);
      refreshScreen();
      return;
    }

    if (isPaused) return;

    isRunning = true;
    screenState = SCREEN_STATUS;
    refreshScreen();
  }
}

void handlePauseInterrupt() {
  if (!pauseInterruptRequest) return;
  pauseInterruptRequest = false;

  if (!isInitialized) return;

  if (isRunning && !isPaused) {
    pauseReturnX = currentX;
    pauseReturnY = currentY;
    pauseReturnZ = currentZ;

    isPaused = true;
    isRunning = false;
    screenState = SCREEN_STATUS;
    refreshScreen();

    pauseMoveInProgress = true;
    moveToReposPosition();
    pauseMoveInProgress = false;

    drawStatusScreen();
  }
  else if (isPaused) {
    isPaused = false;
    isRunning = true;
    screenState = SCREEN_STATUS;
    refreshScreen();

    pauseMoveInProgress = true;
    moveRectangular(pauseReturnX, pauseReturnY, pauseReturnZ, 80, 500, 700, 500);
    pauseMoveInProgress = false;
  }
}

void systemYield() {
  handlePauseInterrupt();
  handleButtons();
  updateLEDs();

  if (screenState == SCREEN_STATUS && (isRunning || isPaused)) {
    if (millis() - lastStatusUpdate > statusRefreshDelay) {
      lastStatusUpdate = millis();
      drawStatusScreen();
    }
  }

  while (isPaused && !pauseMoveInProgress) {
    handlePauseInterrupt();
    handleButtons();
    updateLEDs();
    delay(10);
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
// MOVEMENT EXECUTION
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

float trapezoidalProfile(float u, float alpha = 0.1) {
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

void moveLinearSegment(Point3D p0, Point3D p1, unsigned long dureeMs, unsigned long dtMs = 5) {
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
  moveRectangular(initX, initY, initZ, 80, 1000, 2000, 1000);
}

// =====================================================
// GRIPPER FUNCTIONS
// =====================================================
void stopGripper() {
  analogWrite(PIN_GRIP_PWM, 0);
  digitalWrite(PIN_GRIP_INA, LOW);
  digitalWrite(PIN_GRIP_INB, LOW);
}

void fermer(int timeMs) {
  digitalWrite(PIN_GRIP_INB, HIGH);
  digitalWrite(PIN_GRIP_INA, LOW);
  analogWrite(PIN_GRIP_PWM, gripPWM);

  smartWait(timeMs);

  stopGripper();
}

void ouvrir(int timeMs) {
  digitalWrite(PIN_GRIP_INB, LOW);
  digitalWrite(PIN_GRIP_INA, HIGH);
  analogWrite(PIN_GRIP_PWM, gripPWM);

  smartWait(timeMs);

  stopGripper();
}

// =====================================================
// DYNAMIC ROBOT CYCLE LOGIC
// =====================================================
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

void Sequence() {
  float speedMult = 1.0;

  if (currentMode == 0) speedMult = 2.5;
  if (currentMode == 1) speedMult = 1.5;
  if (currentMode == 2) speedMult = 1.0;

  speedMult = speedMult * executionSpeedFactor;

  unsigned long tUpBase = 500;
  unsigned long tXYBase = 500;
  unsigned long tDownBase = 500;

  unsigned long tUp = tUpBase * speedMult;
  unsigned long tXY = tXYBase * speedMult;
  unsigned long tDown = tDownBase * speedMult;

  int gripCloseTime = 1200;
  float targetZ = 15;

  if (currentObject == 0) {
    gripCloseTime = 1200;
    targetZ = 15;
  }
  else if (currentObject == 1) {
    gripCloseTime = 800;
    targetZ = 30;
  }
  else if (currentObject == 2) {
    gripCloseTime = 2000;
    targetZ = 20;
  }
/*
  moveRectangular(initX, initY, initZ, 100, tUp, tXY, tDown);
  smartWait(300);

  ouvrir(2000);
  smartWait(300);

  moveRectangular(160, 50, targetZ, 100, tUp, tXY, tDown);
  smartWait(300);

  fermer(gripCloseTime);
  smartWait(300);

  moveRectangular(-155, 150, 60, 100, tUp, tXY, tDown);
  smartWait(300);

  ouvrir(2000);
  smartWait(300);
  */
  moveRectangular(initX, initY, initZ, 100, 1000, 1000, 1000);
  smartWait(20);

  ouvrir(3000);
  smartWait(20);
  //Convoyeur entree
  moveRectangular(155, 20, 10, 100, 1000, 1000, 1000);
  smartWait(2000);

  fermer(3000);
  smartWait(20);
  //Machine A
  moveRectangular(-155, 150, 60, 100, 1000, 1000, 1000);
  smartWait(20);

  ouvrir(2000);
  smartWait(20);

    ouvrir(3000);
  smartWait(20);
  //Machine B
  moveRectangular(165, 116, 7, 100, 1000, 1000, 1000);
  smartWait(2000);

  fermer(3000);
  smartWait(20);
  //Convoyeur sortie
  moveRectangular(-155, 20, 0, 100, 1000, 1000, 1000);
  smartWait(20);

  ouvrir(2000);
  smartWait(20);
}

void updateRobotCycle() {
  if (isRunning && !isPaused) {
    int totalPieces = getCombineValue();

    while (currentPiece < totalPieces && isRunning) {
      if (currentCycleType == 0) {
        Sequence();
      }
      else {
        Sequence();
      }

      currentPiece++;
      drawStatusScreen();
    }

    isRunning = false;
    isPaused = false;
    screenState = SCREEN_HOME;
    menuIndex = 0;
    refreshScreen();
  }
}

// =====================================================
// MAINTENANCE MODE
// =====================================================
void printCurrentConfiguration() {
  Serial.println();
  Serial.println(F("----- CURRENT SYSTEM INFORMATION -----"));
  Serial.print(F("Cycle: ")); Serial.println(getCycleText());
  Serial.print(F("Object: ")); Serial.println(getObjectText());
  Serial.print(F("Mode: ")); Serial.println(getModeText());
  Serial.print(F("Pieces: ")); Serial.println(getCombineValue());
  Serial.print(F("Current X: ")); Serial.println(currentX);
  Serial.print(F("Current Y: ")); Serial.println(currentY);
  Serial.print(F("Current Z: ")); Serial.println(currentZ);
  Serial.print(F("Execution speed factor: ")); Serial.println(executionSpeedFactor);
  Serial.print(F("Grip PWM: ")); Serial.println(gripPWM);
  Serial.print(F("Offset A: ")); Serial.println(degOffsetA);
  Serial.print(F("Offset B: ")); Serial.println(degOffsetB);
  Serial.print(F("Offset Z: ")); Serial.println(degOffsetZ);
  Serial.println(F("--------------------------------------"));
}

void printMaintenanceMenu() {
  Serial.println();
  Serial.println(F("======================================"));
  Serial.println(F("          MAINTENANCE CONSOLE"));
  Serial.println(F("======================================"));
  Serial.println(F("8 - Go to repos position (0,130,100)"));
  Serial.println(F("1 - Test Servo A"));
  Serial.println(F("2 - Test Servo B"));
  Serial.println(F("3 - Test Servo Z"));
  Serial.println(F("4 - Open gripper"));
  Serial.println(F("5 - Close gripper"));
  Serial.println(F("6 - Stop gripper"));
  Serial.println(F("7 - Test LEDs"));
  Serial.println(F("f/s - Faster / slower execution"));
  Serial.println(F("p/o - Increase / decrease gripper power"));
  Serial.println(F("a/A - Increase / decrease Servo A offset"));
  Serial.println(F("b/B - Increase / decrease Servo B offset"));
  Serial.println(F("z/Z - Increase / decrease Servo Z offset"));
  Serial.println(F("9 - Print full configuration"));
  Serial.println(F("m - Show menu again"));
  Serial.println(F("Remove jumper + reset to exit maintenance."));
  Serial.println(F("======================================"));
  Serial.println(F("Enter command:"));
}

void testServoMaintenance(Servo &servo, const char* name) {
  Serial.print(F("Testing "));
  Serial.println(name);

  servo.write(60);
  delay(700);
  servo.write(90);
  delay(700);
  servo.write(120);
  delay(700);
  servo.write(90);
  delay(500);

  Serial.println(F("Servo test finished."));
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
  digitalWrite(LED_POWER, LOW);
  digitalWrite(LED_INIT, LOW);

  Serial.println(F("LED test finished."));
}

void maintenanceMode() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("MAINTENANCE"));
  display.println(F("----------------"));
  display.println(F("IHM disabled"));
  display.println(F("Use Serial"));
  display.println(F("9600 baud"));
  display.display();

  stopGripper();

  digitalWrite(LED_POWER, HIGH);
  digitalWrite(LED_INIT, HIGH);
  digitalWrite(LED_RUN, LOW);
  digitalWrite(LED_PAUSE, LOW);

  printMaintenanceMenu();

  while (true) {
    if (Serial.available()) {
      char cmd = Serial.read();

      if (cmd == '\n' || cmd == '\r') continue;

      if (cmd == '1') testServoMaintenance(servoA, "Servo A");
      else if (cmd == '2') testServoMaintenance(servoB, "Servo B");
      else if (cmd == '3') testServoMaintenance(servoZ, "Servo Z");
      else if (cmd == '4') ouvrir(1000);
      else if (cmd == '5') fermer(1000);
      else if (cmd == '6') stopGripper();
      else if (cmd == '7') testLEDsMaintenance();
      else if (cmd == '8') {
        Serial.println(F("Going to repos position..."));
        if (initializeFromCartesianPosition(initX, initY, initZ)) {
          Serial.println(F("Repos position reached."));
        } else {
          Serial.println(F("ERROR: Repos position not reachable."));
        }
      }
      else if (cmd == 'f') {
        executionSpeedFactor -= 0.1;
        if (executionSpeedFactor < 0.5) executionSpeedFactor = 0.5;
        Serial.print(F("Execution speed factor: "));
        Serial.println(executionSpeedFactor);
      }
      else if (cmd == 's') {
        executionSpeedFactor += 0.1;
        if (executionSpeedFactor > 3.0) executionSpeedFactor = 3.0;
        Serial.print(F("Execution speed factor: "));
        Serial.println(executionSpeedFactor);
      }
      else if (cmd == 'p') {
        gripPWM += 10;
        if (gripPWM > 255) gripPWM = 255;
        Serial.print(F("Grip PWM: "));
        Serial.println(gripPWM);
      }
      else if (cmd == 'o') {
        gripPWM -= 10;
        if (gripPWM < 0) gripPWM = 0;
        Serial.print(F("Grip PWM: "));
        Serial.println(gripPWM);
      }
      else if (cmd == 'a') {
        degOffsetA += 1.0;
        Serial.print(F("Offset A: "));
        Serial.println(degOffsetA);
      }
      else if (cmd == 'A') {
        degOffsetA -= 1.0;
        Serial.print(F("Offset A: "));
        Serial.println(degOffsetA);
      }
      else if (cmd == 'b') {
        degOffsetB += 1.0;
        Serial.print(F("Offset B: "));
        Serial.println(degOffsetB);
      }
      else if (cmd == 'B') {
        degOffsetB -= 1.0;
        Serial.print(F("Offset B: "));
        Serial.println(degOffsetB);
      }
      else if (cmd == 'z') {
        degOffsetZ += 1.0;
        Serial.print(F("Offset Z: "));
        Serial.println(degOffsetZ);
      }
      else if (cmd == 'Z') {
        degOffsetZ -= 1.0;
        Serial.print(F("Offset Z: "));
        Serial.println(degOffsetZ);
      }
      else if (cmd == '9') printCurrentConfiguration();
      else if (cmd == 'm' || cmd == 'M') printMaintenanceMenu();
      else Serial.println(F("Unknown command. Type m to show menu."));

      Serial.println(F("Enter command:"));
    }
  }
}

// =====================================================
// SETUP & LOOP
// =====================================================
void setup() {
  delay(200);

  Serial.begin(9600);

  pinMode(PIN_MAINTENANCE, INPUT_PULLUP);

  pinMode(BTN_PAUSE, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_INIT, INPUT_PULLUP);
  pinMode(BTN_CYCLE, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);

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

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    while (true);
  }

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
}