#include <Servo.h>

Servo servoA;
Servo servoB;
Servo servoZ;

// ======================================================
// PARAMETRES BRAS
// ======================================================

// Longueurs des deux bras (mm)
float L1 = 150.0;
float L2 = 143.0;

// Décalages mécaniques du pivot du bras par rapport à l'origine globale
float offsetX = 20.0;
float offsetZ = 75.0;

// Offsets servo (calibration)
float degOffsetA = -20.0;
float degOffsetB = -10.0;
float degOffsetZ = -2.5;

// ======================================================
// PARAMETRES PREHENSEUR
// ======================================================

// Le point piloté est la pointe de l'outil.
// Le poignet est donc décalé par rapport à cette pointe.
float toolLength = 100.0;   // mm, horizontal, toujours parallèle au sol
float toolHeight = -10.0;  // mm, outil 10 mm plus bas que le poignet

// ======================================================
// PARAMETRES TRAJECTOIRE / COMMANDE
// ======================================================

// Limitation de variation angulaire par cycle
float maxStepA = 0.2;
float maxStepB = 0.2;
float maxStepZ = 0.5;

// Position initiale de l'outil (pointe préhenseur)
float initX = 160.0;
float initY = 150.0;
float initZ = 30.0;

// Position courante estimée de la pointe outil
float currentX = initX;
float currentY = initY;
float currentZ = initZ;

// Commandes servo courantes
float currentCmdA = 0.0;
float currentCmdB = 0.0;
float currentCmdZ = 0.0;

// Debug
bool debugIK = false;
bool debugTraj = true;

// ======================================================
// STRUCTURES
// ======================================================
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

struct DebugIKData {
  float x;
  float y;
  float z;

  float rTool;
  float rWrist;
  float zWrist;

  float rp;
  float zp;
  float D;

  float degA;
  float degB;
  float degZ;

  float cmdA;
  float cmdB;
  float cmdZ;

  bool reachable;
};

// ======================================================
// OUTILS DEBUG
// ======================================================
void printLine() {
  Serial.println("----------------------------------------");
}

void printPoint(const char* label, const Point3D& p) {
  Serial.print(label);
  Serial.print(" : ");
  Serial.print(p.x);
  Serial.print(", ");
  Serial.print(p.y);
  Serial.print(", ");
  Serial.println(p.z);
}

void printIKDebug(const DebugIKData& d) {
  printLine();

  Serial.print("Cible outil : x=");
  Serial.print(d.x);
  Serial.print(" y=");
  Serial.print(d.y);
  Serial.print(" z=");
  Serial.println(d.z);

  Serial.print("rTool = ");
  Serial.print(d.rTool);
  Serial.print(" | rWrist = ");
  Serial.print(d.rWrist);
  Serial.print(" | zWrist = ");
  Serial.println(d.zWrist);

  Serial.print("rp = ");
  Serial.print(d.rp);
  Serial.print(" | zp = ");
  Serial.print(d.zp);
  Serial.print(" | D = ");
  Serial.println(d.D);

  Serial.print("Angles geo -> A: ");
  Serial.print(d.degA);
  Serial.print(" | B: ");
  Serial.print(d.degB);
  Serial.print(" | Z: ");
  Serial.println(d.degZ);

  Serial.print("Cmd servo  -> A: ");
  Serial.print(d.cmdA);
  Serial.print(" | B: ");
  Serial.print(d.cmdB);
  Serial.print(" | Z: ");
  Serial.println(d.cmdZ);

  Serial.print("Reachable  -> ");
  Serial.println(d.reachable ? "OUI" : "NON");
}

void printRectangularTrajectory(const Point3D& pStart, const Point3D& pUp, const Point3D& pMid, const Point3D& pEnd) {
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
int degToUs(float deg) {
  deg = constrain(deg, 0.0, 180.0);
  return (int)(500.0 + (deg / 180.0) * 2000.0); // 500 à 2500 us
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

  Serial.print("out.a = ");
  Serial.println(out.a);
  Serial.print("out.b = ");
  Serial.println(out.b);
  Serial.print("out.z = ");
  Serial.println(out.z);
  return out;
}

// ======================================================
// DEPLACEMENT DIRECT VERS UNE COMMANDE CIBLE
// ======================================================
void moveJointsSmoothTo(JointCmd target, unsigned long dtMs = 5) {
  if (!target.reachable) return;

  bool done = false;

  while (!done) {
    currentCmdA = limitStep(currentCmdA, target.a, maxStepA);!target.reachable
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
void moveLinearSegment(Point3D p0, Point3D p1, unsigned long dureeMs, unsigned long dtMs = 5) {
  if (dureeMs < dtMs) dureeMs = dtMs;

  unsigned long t0 = millis();

  int steps = max((int)(dureeMs / dtMs), 20);

for (int i = 0; i <= steps; i++) {

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
  Point3D pStart = {currentX, currentY, currentZ};
  float zLift = max(currentZ, zf) + liftHeight;

  Point3D pUp  = {currentX, currentY, zLift};
  Point3D pMid = {xf, yf, zLift};
  Point3D pEnd = {xf, yf, zf};

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

// ======================================================
// SETUP
// ======================================================
void setup() {
  servoA.attach(5);
  servoB.attach(9);
  servoZ.attach(11);

  Serial.begin(9600);
  delay(1000);
  Serial.println("Demarrage...");

  initializeFromCartesianPosition(initX, initY, initZ);
  delay(1000);
}

// ======================================================
// LOOP
// ======================================================
void loop() {

  JointCmd test = computeIK(100, 100, 100);
  moveRectangular(160, 150, 10, 100.0, 500, 2000, 1000);
  delay(1000);

  moveRectangular(-160, 115, 10, 100.0, 500, 2000, 1000);
  delay(1000);
}