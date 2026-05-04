#include <Arduino.h>
#include "trajectories.h"
#include "prehension.h"
#include "userInterfaceScreen.h"
#include "maintenance.h"

// ====================================================
// [trajectories.cpp]  GLOBALS — position tables, offset tables, grip tables
// ====================================================

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


// =====================================================
// HMI GLOBAL STATES
// =====================================================

PositionOffset positionOffsets[NB_PREHENSEURS][NB_CYCLES][NB_OBJECTS][NB_POSITIONS];
GripSettings gripSettings[NB_PREHENSEURS][NB_OBJECTS];

// ====================================================
// [trajectories.cpp]  NAME HELPERS
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


// ====================================================
// [trajectories.cpp]  CONFIGURATION HELPERS
// ====================================================

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

// ====================================================
// [trajectories.cpp]  OFFSET & POSITION LOOKUP
// ====================================================

PositionOffset getOffset(int prehenseur, int cycle, int object, int position) {
  return positionOffsets[prehenseur][cycle][object][position];
}
PositionOffset getOffset(int cycle, int object, int position) {
  return getOffset(currentPrehenseur, cycle, object, position);
}
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

// ====================================================
// [trajectories.cpp]  CONFIGURATION CHANGE CALLBACKS
// ====================================================

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
// [trajectories.cpp]  CYCLE TIMING
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

// ====================================================
// [trajectories.cpp]  MOVEMENT DISPATCHERS
// ====================================================

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
// [trajectories.cpp]  CYCLE SEQUENCES
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
// [trajectories.cpp]  OBJECT CYCLE DISPATCHERS
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
// [trajectories.cpp [YOUR CODE]]  YOUR TRAJECTORY LOGIC — zones, cycle steps, trajectory profiles
// ====================================================

// [YOUR trajectories.cpp not found in uploads — paste content here]
