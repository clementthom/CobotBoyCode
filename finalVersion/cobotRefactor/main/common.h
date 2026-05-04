#ifndef COMMON_H
#define COMMON_H

#include <stdlib.h>
#include <math.h>
#include <stdio.h>

// =====================================================
// YOUR CODE — shared enums and structs (unchanged)
// =====================================================

typedef enum { OBSTACLE, WORKING_ZONE } ZoneType;

typedef enum {
  CONVOYEUR_ENTREE, CONVOYEUR_SORTIE, MACHINE_A, MACHINE_B,
  INIT, GRAB_OBJECT, RELEASE_OBJECT
} CycleStep;

typedef enum {
  GRAB, RELEASE, KEEP_CURRENT_PREHENSION_POSITION, UNKNOWN_STATUS
} PrehensionStatus;

typedef enum { ECO, PERFORMANCE } CycleMode;
typedef enum { GOMME, GOBELET, CYLINDRE } ObjectName;
typedef enum { ROBOT1, ROBOT2 } RobotName;
typedef enum { LASSO, EYE } PrehensionSystemType;

typedef struct { float x; float y; float z; } Coordinates;

typedef struct {
  Coordinates origin;
  Coordinates farthestPointFromOrigin;
  ZoneType zoneType;
} Zone;

typedef struct { Coordinates distances; ZoneType zoneType; } ZoneDistances;

typedef struct {
  int radius; int height; ObjectName objectName; double consigne;
} Object;

typedef struct { Object gomme; Object gobelet; Object cyclindre; } ObjectList;
typedef struct { RobotName robotName; PrehensionSystemType prehensionSystemType; } Robot;

// =====================================================
// TEAM'S CODE — shared structs (added to common.h)
// =====================================================

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

// Dimension constants for offset and grip tables
const int NB_CYCLES = 2;
const int NB_OBJECTS = 3;
const int NB_POSITIONS = 4;
const int NB_PREHENSEURS = 2;

// Prehenseur index constants
const int PREHENSEUR_OFFSET = 0;
const int PREHENSEUR_CENTERED = 1;

// Object index constants
const int OBJ_GOMME = 0;
const int OBJ_GOBELET = 1;
const int OBJ_CYLINDRE = 2;

// Position index constants
const int POS_convoyeurEntree = 0;
const int POS_machineA = 1;
const int POS_machineB = 2;
const int POS_convoyeurSortie = 3;

struct PositionOffset {
  float dx;
  float dy;
  float dz;
};
struct GripSettings {
  float openSetpoint;
  float closeSetpoint;
};

// Screen states (used by userInterfaceScreen and main)
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

// Gripper FSM states (used by prehension and maintenance)
enum GripperState {
  GRIPPER_STOPPED,
  GRIPPER_OPENING,
  GRIPPER_CLOSING
};

// =====================================================
// MAIN.INO — extern declarations for globals owned by main
// (defined in main.ino, visible to all .cpp modules)
// =====================================================
extern bool isInitialized;
extern bool isInitializing;
extern bool isRunning;
extern bool isPaused;

extern bool terminalModeActive;
extern bool manualServoTestDirty;
extern bool terminalButtonTestMode;
extern bool terminalLedTestMode;
extern bool ledTestInit;
extern bool ledTestRun;
extern bool ledTestPause;
extern bool ledTestPower;
extern unsigned long lastTerminalContact;

extern int menuIndex;
extern int currentCycleType;
extern int currentPrehenseur;
extern int currentObject;
extern int currentMode;
extern int currentCombine;
extern int currentPiece;

extern unsigned long bootTime;

extern const unsigned long bootIgnoreButtonsMs;
extern const unsigned long statusRefreshDelay;
// =====================================================
// MAIN.INO — extern declarations for functions defined in main.ino
// Called by .cpp modules; declared here so all modules see them.
// =====================================================
extern void systemYield();
extern void smartWait(unsigned long ms);
extern void performInitialization();
extern void ihmInitializeAction();
extern void ihmStartAction();
extern void ihmPauseToggleAction();

#endif // COMMON_H
