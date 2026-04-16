#include <Servo.h>
#include "common.h" //lists structs and other elements shared by different modules
#include "cinematics.h" //cinematic-related functions and structs (position + servo controls)
#include "trajectories.h" //code related to registered trajectories and speed profiles
#include "userInterface.h" //manages the control panel components and screens
#include "maintenance.h" //serial communication with a computer using serial port (maintenance mode)

//Prototypes
void fullyApplyCommandToServos(ServoSet* servoSet, int delayCommand);

//Components pins
#define SERVO_LEFT_PIN 5
#define SERVO_RIGHT_PIN 9
#define SERVO_Z_PIN 11

//servo objects declarations
Servo servoLeft;
Servo servoRight;
Servo servoZ;

ServoSet servoSet;
Coordinates currentPosition;
Coordinates destination;
Coordinates IntermediatePosition;

PrehensionStatus prehensionStatus;
IntermediatePoint intermediatePoint;
int cycleStepIndex;


// ======================================================
// SETUP
// ======================================================
void setup() {

  Serial.begin(9600);

  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);
  servoZ.attach(SERVO_Z_PIN);

  initCoordinates();
  initZone();
  initServoSet(&servoSet);
  prehensionStatus=UNKNOWN_STATUS;
  cycleStepIndex=0;

    
  currentPosition=initPosition;
  coordinatesToAngles(&currentPosition, &servoSet);

  
  servoLeft.write(servoSet.servoLeft.angleCommand);
  servoRight.write(servoSet.servoRight.angleCommand);
  servoZ.write(servoSet.servoZ.angleCommand); 


  servoSet.servoLeft.currentAngle = servoSet.servoLeft.angleCommand;
  servoSet.servoRight.currentAngle = servoSet.servoRight.angleCommand;
  servoSet.servoZ.currentAngle = servoSet.servoZ.angleCommand; 
  delay(200);
}

// ======================================================
// LOOP
// ======================================================
void loop() {

  do {
    cycleExecution(&destination, &prehensionStatus, &cycleStepIndex); //cycleStepIndex becomes the one of the next step here
    if(prehensionStatus==0 || prehensionStatus==1) {
      Serial.println("prehension action");
      Serial.println(prehensionStatus);
      Serial.println("");
      //actionPrehension();
      delay(2000);
    }
    do {
      intermediatePosition = trajectoryProfile(currentPosition, destination, 100, PERFORMANCE, &intermediatePoint);
      coordinatesToAngles(&intermediatePosition, &servoSet);
      fullyApplyCommandToServos(&servoSet, 50);
      anglesToCoordinates(&servoSet, &currentPosition);
    }while(intermediatePoint!=DESTINATION_POINT) ;
  }while(cycleStepIndex!=0);
}


void fullyApplyCommandToServos(ServoSet* servoSet, int delayCommand) {
  if (!servoSet->reachable) return;

  while (!(abs(servoSet->servoLeft.currentAngle - servoSet->servoLeft.angleCommand) < 0.05 &&
      abs(servoSet->servoRight.currentAngle - servoSet->servoRight.angleCommand) < 0.05 &&
      abs(servoSet->servoZ.currentAngle - servoSet->servoZ.angleCommand) < 0.05)) {

        applyServoCommand(servoSet, delayCommand); 
        servoLeft.write(servoSet->servoLeft.currentAngle*0.944);
        servoRight.write(servoSet->servoRight.currentAngle*0.955);
        servoZ.write(map(servoSet->servoZ.currentAngle, 7, 173, 0,180));

        delay(delayCommand);
  }
}
