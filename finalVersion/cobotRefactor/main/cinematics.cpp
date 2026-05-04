#include <Arduino.h>
#include "cinematics.h"

// ====================================================
// [cinematics.cpp]  GLOBALS — servo objects, kinematics params, motion state
// ====================================================

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
// [cinematics.cpp]  LOW-LEVEL SERVO HELPERS
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


// ====================================================
// [cinematics.cpp]  INVERSE KINEMATICS
// ====================================================

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
// [cinematics.cpp]  MOTION PRIMITIVES
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

// ====================================================
// [cinematics.cpp]  HIGH-LEVEL POSITION MOVES
// ====================================================

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
// [cinematics.cpp [YOUR CODE]]  YOUR KINEMATICS — coordinatesToAngles, speedProfiles, FK, etc.
// ====================================================

// NOTE: limitStep and delay() from your PC-side code are excluded here:
//   limitStep  → identical to team's version above (same signature & logic)
//   delay()    → Arduino provides this natively; your version was PC-debug only

// [YOUR cinematics.cpp not found in uploads — paste content here]
