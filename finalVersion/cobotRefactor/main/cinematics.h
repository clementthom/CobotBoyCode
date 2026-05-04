#ifndef CINEMATICS_H
#define CINEMATICS_H

#include <Servo.h>
#include "common.h"

// =====================================================
// TEAM'S CODE — speed profile enum (replaces your SpeedProfileType
// which is superseded by the trapezoidalProfile() approach)
// =====================================================
// YOUR CODE — kept for compatibility with your modules
enum SpeedProfileType { CONSTANT, TRAPESOIDAL_LINEAR, TRAPESOIDAL_EXPONENTIAL };

// YOUR CODE — servo parameters struct
typedef struct {
  float servoMaxStep;
  float startAngle;
  float maxStep;
  float angleCommand;
  float currentAngle;
  float angleOffset;
} ServoParams;

typedef struct {
  ServoParams servoLeft;
  ServoParams servoRight;
  ServoParams servoZ;
  int reachable;
} ServoSet;

// =====================================================
// EXTERN GLOBALS — defined in cinematics.cpp
// =====================================================
extern Servo servoA;
extern Servo servoB;
extern Servo servoZ;

extern float L1;
extern float L2;
extern float offsetX;
extern float offsetZ;

extern float degOffsetA;
extern float degOffsetB;
extern float degOffsetZ;

extern float toolRadius;
extern float objectRadius;
extern float toolOffset;
extern float prehensionHeight;
extern float prehensionHeightBloc;
extern float toolLength;
extern float toolHeight;

extern float maxStepA;
extern float maxStepB;
extern float maxStepZ;

extern float executionSpeedFactor;

extern unsigned long cycleStartTime;
extern unsigned long cycleEndTime;

extern float initX;
extern float initY;
extern float initZ;

extern float pauseReturnX;
extern float pauseReturnY;
extern float pauseReturnZ;

extern bool pauseMoveInProgress;

extern float currentX;
extern float currentY;
extern float currentZ;

extern float currentCmdA;
extern float currentCmdB;
extern float currentCmdZ;

// =====================================================
// FUNCTION PROTOTYPES
// =====================================================

// --- Low-level servo helpers ---
int   degToUs(float deg);
void  writeServosMicroseconds(float cmdA, float cmdB, float cmdZ);
float limitStep(float currentValue, float targetValue, float maxStep);

// --- Inverse kinematics ---
JointCmd computeIK(float x, float y, float z);

// --- Motion primitives ---
void  moveJointsSmoothTo(JointCmd target, unsigned long dtMs = 5);
float trapezoidalProfile(float u, float alpha = 0.2);
void  moveLinearSegment(Point3D p0, Point3D p1, unsigned long dureeMs, unsigned long dtMs = 20);
void  moveRectangular(float xf, float yf, float zf, float liftHeight,
                      unsigned long tUp, unsigned long tXY, unsigned long tDown);
void  servoBBackwardBeforeLift(float degreesBack, unsigned long durationMs);
void  servoABBackwardBeforeLift(float degreesA, float degreesB, unsigned long durationMs);
void  moveRectangularAfterServoBNudge(float xf, float yf, float zf, float liftHeight,
                                      unsigned long tUp, unsigned long tXY, unsigned long tDown);
void  leaveSortieAfterDrop(float xf, float yf, float zf, float liftHeight,
                           unsigned long tUp, unsigned long tXY, unsigned long tDown);

// --- High-level position moves ---
void moveToReposPosition();
bool returnToReposFromCurrentServoAngles();
bool initializeFromCartesianPosition(float x, float y, float z);

// =====================================================
// YOUR CODE — function prototypes (kept for your modules)
// =====================================================
void  coordinatesChange(Coordinates* coordinates, float xPosition, float yPosition, float zPosition);
void  coordinatesToAngles(Coordinates* coordinates, ServoSet* servoSet, Object* currentObject);
void  applyServoCommand(ServoSet* servoSet, int delayStepCloserToCommand,
                        SpeedProfileType speedProfileType, int depthPercentage,
                        CycleMode cycleMode, int* elapsedTimeSinceServoCycleStart,
                        float* anglePerformedDuringAcceleration, int* remainingCycleTime);
void  speedProfileApplication(ServoSet* servoSet, enum SpeedProfileType speedProfileType,
                              int depthPercentage, CycleMode cycleMode,
                              int elapsedTimeSinceServoCycleStart, int delayCommandServo,
                              float* anglePerformedDuringAcceleration, int* remainingCycleTime);
void  anglesToCoordinates(ServoSet* servoSet, Coordinates* coordinates, Object* currentObject);
void  initServoSet(ServoSet* servoSet, int delayStepCloserToCommand);
void  affectInitialServoPosition(ServoSet* servoSet);
void  objectListInit();
void  initRobotOffsets(Robot* robot);

#endif // CINEMATICS_H
