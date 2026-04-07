#include <Servo.h>
#include "common.h" //lists structs and other elements shared by different modules
#include "cinematics.h" //cinematic-related functions and structs (position + servo controls)
#include "cinematics.c"
#include "debug.h" //debug functions, mainly console tests returns
#include "trajectories.h" //code related to registered trajectories and speed profiles
#include "userInterface.h" //manages the control panel components and screens
#include "maintenance.h" //serial communication with a computer using serial port (maintenance mode)

//Components pins
#define SERVO_LEFT_PIN 5
#define SERVO_RIGHT_PIN 9
#define SERVO_Z_PIN 11

//servo objects declarations
Servo servoLeft;
Servo servoRight;
Servo servoZ;




// ======================================================
// SETUP
// ======================================================
void setup() {
  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);
  servoZ.attach(SERVO_Z_PIN);

  Serial.begin(9600);
  delay(1000);
  Serial.println("Demarrage...");

  //initialisation position initiale de l'outil (pointe préhenseur, point théorique)
  currentPosition.x=160.0;
  currentPosition.y=150.0;
  currentPosition.z=30.0;
  
  /*
  initializeFromCartesianPosition(initX, initY, initZ); //takes obstacles into account
  */
  delay(1000);
}

// ======================================================
// LOOP
// ======================================================
void loop() {
  /*
  moveRectangular(160, 150, 30, 40.0, 2000, 2500, 2000);
  delay(1000);

  moveRectangular(-160, 115, 30, 40.0, 2500, 3500, 2000);
  delay(1000);
  */

  
}

/*


void printRectangularTrajectory(const Coordinates& pStart, const Coordinates& pUp, const Coordinates& pMid, const Coordinates& pEnd) {
  printLine();
  Serial.println("Trajectoire rectangulaire");
  printPoint("Depart ", pStart);
  printPoint("Lift   ", pUp);
  printPoint("XY     ", pMid);
  printPoint("Arrivee", pEnd);
}

// ======================================================
// CONVERSIONS / COMMANDE SERVO
// ======================================================
int degToUs(float deg) { //la consigne pour les servos n'est plus en position, mais en temps (microsecondes)
  deg = constrain(deg, 0.0, 180.0); //limite la valeur en degrés aux bornes définies (ici 0 et 180)
  return (int)(500.0 + (deg / 180.0) * 2000.0); // 500 à 2500 us
}

void writeServosMicroseconds(float cmdA, float cmdB, float cmdZ) {
  servoLeft.writeMicroseconds(degToUs(cmdA));
  servoRight.writeMicroseconds(degToUs(cmdB));
  servoZ.writeMicroseconds(degToUs(cmdZ));
}

float limitStep(float currentValue, float targetValue, float maxStep) {
  float delta = targetValue - currentValue;

  if (delta > maxStep) return currentValue + maxStep;
  if (delta < -maxStep) return currentValue - maxStep;
  return targetValue;
}


// ======================================================
// CINEMATIQUE INVERSE
// Entrée : position de la POINTE DU PREHENSEUR
// Sortie : commandes servo
// ======================================================
JointCmd computeIK(float x, float y, float z) {
  JointCmd out;
  out.reachable = false;

  // ------------------------------------------
  // 1) Point outil -> point poignet
  // ------------------------------------------
  float rTool = sqrt(x * x + y * y);
  float theta3 = atan2(y, x);

  // Le segment de 50 mm reste parallèle au sol
  float rWrist = rTool - toolLength;
  float zWrist = z - toolHeight;   // toolHeight = -10 => zWrist = z + 10

  // ------------------------------------------
  // 2) Correction des offsets mécaniques bras
  // ------------------------------------------
  float rp = rWrist - offsetX;
  float zp = zWrist - offsetZ;

  // ------------------------------------------
  // 3) IK bras 2 segments
  // ------------------------------------------
  float D = (rp * rp + zp * zp - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);

  if (D < -1.0 || D > 1.0) {
    if (debugIK) {
      DebugIKData dbg = {x, y, z, rTool, rWrist, zWrist, rp, zp, D, 0, 0, 0, 0, 0, 0, false};
      printIKDebug(dbg);
    }
    return out;
  }

  float theta1 = atan2(sqrt(1.0 - D * D), D);
  float theta2 = atan2(zp, rp) - atan2(L2 * sin(theta1), L1 + L2 * cos(theta1));

  // Angles géométriques -> selon ta calibration actuelle
  float degA = degrees(theta2) + 180.0;
  float degB = 180.0 - degrees(theta2 + theta1);
  float degZ = degrees(theta3);

  // Offsets servo
  float cmdA = degA + degOffsetA;
  float cmdB = degB + degOffsetB;
  float cmdZ = degZ + degOffsetZ;

  if (debugIK) {
    DebugIKData dbg = {x, y, z, rTool, rWrist, zWrist, rp, zp, D, degA, degB, degZ, cmdA, cmdB, cmdZ, true};
    printIKDebug(dbg);
  }

  if (cmdA < 0.0 || cmdA > 180.0) return out;
  if (cmdB < 0.0 || cmdB > 180.0) return out;
  if (cmdZ < 0.0 || cmdZ > 180.0) return out;

  out.a = cmdA;
  out.b = cmdB;
  out.z = cmdZ;
  out.reachable = true;
  return out;
}

// ======================================================
// DEPLACEMENT DIRECT VERS UNE COMMANDE CIBLE
// ======================================================
void moveJointsSmoothTo(JointCmd target, unsigned long dtMs = 20) {
  if (!target.reachable) return;

  bool done = false;

  while (!done) {
    currentCmdA = limitStep(currentCmdA, target.a, maxStepA);
    currentCmdB = limitStep(currentCmdB, target.b, maxStepB);
    currentCmdZ = limitStep(currentCmdZ, target.z, maxStepZ);

    writeServosMicroseconds(currentCmdA, currentCmdB, currentCmdZ);

    done =
      abs(currentCmdA - target.a) < 0.05 &&
      abs(currentCmdB - target.b) < 0.05 &&
      abs(currentCmdZ - target.z) < 0.05;

    delay(dtMs);
  }
}

// ======================================================
// PROFIL DE VITESSE TRAPEZOIDAL
// u entre 0 et 1
// retourne s entre 0 et 1
// ======================================================
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

// ======================================================
// DEPLACEMENT LINEAIRE CARTESIEN
// ======================================================
void moveLinearSegment(Coordinates p0, Coordinates p1, unsigned long dureeMs, unsigned long dtMs = 20) {
  if (dureeMs < dtMs) dureeMs = dtMs;

  unsigned long t0 = millis();

  while (true) {
    unsigned long t = millis() - t0;
    if (t >= dureeMs) break;

    float u = (float)t / (float)dureeMs;
    float s = trapezoidalProfile(u, 0.2);

    float x = p0.x + s * (p1.x - p0.x);
    float y = p0.y + s * (p1.y - p0.y);
    float z = p0.z + s * (p1.z - p0.z);

    JointCmd target = computeIK(x, y, z);

    if (target.reachable) {
      currentCmdA = limitStep(currentCmdA, target.a, maxStepA);
      currentCmdB = limitStep(currentCmdB, target.b, maxStepB);
      currentCmdZ = limitStep(currentCmdZ, target.z, maxStepZ);

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

// ======================================================
// TRAJECTOIRE RECTANGULAIRE
// 1) monter en Z
// 2) deplacer en XY
// 3) descendre en Z
// ======================================================
void moveRectangular(float xf, float yf, float zf,
                     float liftHeight,
                     unsigned long tUp,
                     unsigned long tXY,
                     unsigned long tDown) {
  Coordinates pStart = {currentX, currentY, currentZ};
  float zLift = max(currentZ, zf) + liftHeight;

  Coordinates pUp  = {currentX, currentY, zLift};
  Coordinates pMid = {xf, yf, zLift};
  Coordinates pEnd = {xf, yf, zf};

  if (debugTraj) {
    printRectangularTrajectory(pStart, pUp, pMid, pEnd);
  }

  moveLinearSegment(pStart, pUp,  tUp, 20);
  moveLinearSegment(pUp,    pMid, tXY, 20);
  moveLinearSegment(pMid,   pEnd, tDown, 20);
}

// ======================================================
// INITIALISATION DEPUIS UNE POSITION CARTESIENNE
// ======================================================
bool initializeFromCartesianPosition(float x, float y, float z) {
  JointCmd initCmd = computeIK(x, y, z);

  if (!initCmd.reachable) {
    Serial.println("Position initiale non atteignable");
    return false;
  }

  currentX = x;
  currentY = y;
  currentZ = z;

  currentCmdA = initCmd.a;
  currentCmdB = initCmd.b;
  currentCmdZ = initCmd.z;

  writeServosMicroseconds(currentCmdA, currentCmdB, currentCmdZ);
  return true;
}
*/