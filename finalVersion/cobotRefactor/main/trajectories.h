#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include "common.h"
#include "cinematics.h"   // for prehensionHeight (used in basePositions initialiser)

// =====================================================
// YOUR CODE — trajectory enums (kept for your modules)
// =====================================================
typedef enum { DEPART_POINT, INTERM_POINT_1, INTERM_POINT_2, DESTINATION_POINT } IntermediatePoint;
typedef enum { CYCLE1, CYCLE2, CUSTOM } SelectedCycle;

extern Coordinates initPosition;
extern Coordinates convoyeurEntree;

// =====================================================
// EXTERN GLOBALS — defined in trajectories.cpp
// =====================================================
extern Point3D basePositions[NB_CYCLES][NB_POSITIONS];
extern const char* positionNames[NB_CYCLES][NB_POSITIONS];
extern PositionOffset positionOffsets[NB_PREHENSEURS][NB_CYCLES][NB_OBJECTS][NB_POSITIONS];
extern GripSettings   gripSettings[NB_PREHENSEURS][NB_OBJECTS];

// =====================================================
// FUNCTION PROTOTYPES
// =====================================================

// --- Name helpers ---
const char* getObjectNameByIndex(int obj);
const char* getPrehenseurNameByIndex(int prehenseur);

// --- Configuration helpers ---
bool selectedPrehenseurUsesToolOffset();
void applyObjectGeometry(int objectIndex);
void setDefaultRuntimeConfig();
void applyGripperSettingsForObject(int objectIndex);
void applyFullConfiguration();

// --- Offset & position lookup ---
PositionOffset getOffset(int prehenseur, int cycle, int object, int position);
PositionOffset getOffset(int cycle, int object, int position);
void           applyMachinePieceCentering(Point3D& target, int position);
Point3D        getCorrectedPosition(int cycle, int object, int position);
Point3D        getCorrectedPrehensionPosition(int cycle, int object, int position);

// --- Cycle timing ---
void getCycleTimes(unsigned long& tUp, unsigned long& tXY, unsigned long& tDown);

// --- Movement dispatchers ---
void moveToBasePositionForObject(int cycle, int objectIndex, int position,
                                 float liftHeight,
                                 unsigned long tUp, unsigned long tXY, unsigned long tDown);
void moveToPrehensionPositionForObject(int cycle, int objectIndex, int position,
                                       float liftHeight,
                                       unsigned long tUp, unsigned long tXY, unsigned long tDown);
void moveToPrehensionPositionExtraForObject(int cycle, int objectIndex, int position,
                                            float extraX, float extraY, float extraZ,
                                            float liftHeight,
                                            unsigned long tUp, unsigned long tXY, unsigned long tDown);
void moveToBasePosition(int cycle, int position, float liftHeight,
                        unsigned long tUp, unsigned long tXY, unsigned long tDown);
void prepareObjectCycle(int objectIndex);

// --- Cycle sequences ---
void SequenceCycle1ForObject(int objectIndex);
void SequenceCycle2ForObject(int objectIndex);
void cycleGomme();
void cycleGobelet();
void cycleCylindre();
void SequenceCycle1();
void SequenceCycle2();
void runSelectedObjectCycle();
void updateRobotCycle();

// =====================================================
// YOUR CODE — function prototypes (kept for your modules)
// =====================================================
void           initCoordinates(SelectedCycle* selectedCycle);
void           initZone();
int            checkIfInZone(Coordinates coordinates, Zone zone);
ZoneDistances  checkIfCloseToObstacle(Coordinates coordinates, Zone zone);
PrehensionStatus stepActions(CycleStep stepToken, Coordinates* coordinates);
void           cycleExecution(Coordinates* destinationCoordinates,
                              PrehensionStatus* actionPrehension,
                              int* nextStepIndex, SelectedCycle* selectedCycle);
Coordinates    trajectoryProfile(Coordinates currentCoordinates, Coordinates destination,
                                 int selectedMaxAlt, CycleMode cycleMode,
                                 IntermediatePoint* intermediatePoint);

#endif // TRAJECTORIES_H
