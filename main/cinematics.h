#ifndef CINEMATICS_H
#define CINEMATICS_H

#include "common.h"

//Enumerations

//How the speed profile behaves between 2 points
enum SpeedProfileType {
  CONSTANT, // constant between points
  TRAPESOIDAL_LINEAR, // accelerate and decelerate linearly
  TRAPESOIDAL_EXPONENTIAL // accelerate and decelerate exponentially
};

////Structures

//Angle parameters linked to one servo
typedef struct {
  float servoMaxStep; //physical max angle a servo can perform in the defined cycle period (see datasheet)
  float startAngle; //initial position of the servo before the servo cycle starts
  float maxStep; //maximum step a servo can perform in one applyCommand (unit) cycle
  float angleCommand; //command in microseconds (angle unit), destination to reach
  float currentAngle; //current angle (same remark) 
  float angleOffset; //offset of a servo
}ServoParams; //regroups all parameters linked to ONE servo

//Groups all servos into one entity
typedef struct {
  ServoParams servoLeft;
  ServoParams servoRight;
  ServoParams servoZ;
  int reachable; 
}ServoSet;


void coordinatesChange (Coordinates* coordinates, float xPosition, float yPosition, float zPosition);
void coordinatesToAngles(Coordinates* coordinates, ServoSet* servoSet, Object* currentObject);
void applyServoCommand(ServoSet *servoSet, int delayStepCloserToCommand , SpeedProfileType speedProfileType, int depthPercentage, 
    CycleMode cycleMode, int *elapsedTimeSinceServoCycleStart, float *anglePerformedDuringAcceleration,
    int *remainingCycleTime);
float limitStep(float currentValue, float targetValue, float maxStep);
void delay(int milli_seconds); //only for debbuging
void speedProfileApplication(ServoSet* servoSet, enum SpeedProfileType speedProfileType, int depthPercentage, 
    CycleMode cycleMode, int elapsedTimeSinceServoCycleStart, int delayCommandServo, float *anglePerformedDuringAcceleration,
     int *remainingCycleTime);
void anglesToCoordinates(ServoSet* servoSet, Coordinates* coordinates, Object* currentObject);
void initServoSet(ServoSet* servoSet, int delayStepCloserToCommand);
void affectInitialServoPosition(ServoSet* servoSet);
void objectListInit();
void initRobotOffsets(Robot* robot);

#endif