#ifndef CINEMATICS_H
#define CINEMATICS_H

#include "common.h"

//Enumeration
enum SpeedProfileType {
  CONSTANT,
  TRAPESOIDAL_LINEAR,
  TRAPESOIDAL_EXPONENTIAL
};

////Structures
typedef struct {
  float maxStep;
  float angleCommand; //command in microseconds (angle unit)
  float currentAngle; //current angle (same remark) 
  float angleOffset;
}ServoParams; //regroups all parameters linked to ONE servo

typedef struct {
  ServoParams servoLeft;
  ServoParams servoRight;
  ServoParams servoZ;
  int reachable; 
}ServoSet;


void coordinatesChange (Coordinates* coordinates, float xPosition, float yPosition, float zPosition);
void coordinatesToAngles(Coordinates* coordinates, ServoSet* servoSet);
void applyServoCommand(ServoSet* servoSet, int delayStepCloserToCurrent);
float limitStep(float currentValue, float targetValue, float maxStep);
//void delay(int milli_seconds); //only for debbuging
int speedProfileApplication(enum SpeedProfileType speedProfileType);


#endif