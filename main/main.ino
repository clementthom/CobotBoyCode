#include <Servo.h>
#include "common.h" //lists structs and other elements shared by different modules
#include "cinematics.h" //cinematic-related functions and structs (position + servo controls)
#include "debug.h" //debug functions, mainly console tests returns
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
Coordinates destination;
PrehensionStatus prehensionStatus;
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
  prehensionStatus=UNKNOWN_STATUS;
  cycleStepIndex=0;

  servoSet.servoLeft.angleOffset=-20.0;
  servoSet.servoRight.angleOffset=10.0;
  servoSet.servoZ.angleOffset=2.5;

  servoSet.servoLeft.currentAngle=15;
  servoSet.servoRight.currentAngle=30;
  servoSet.servoZ.currentAngle=45;

  servoSet.servoLeft.maxStep = 1.2;
  servoSet.servoRight.maxStep = 1.2;
  servoSet.servoZ.maxStep = 1.5;

  Serial.println("Demarrage...");

  Coordinates currentPosition;
  //initialisation position initiale de l'outil (pointe préhenseur, point théorique)
  currentPosition.x=100.0;
  currentPosition.x=-100.0;
  currentPosition.x=100.0;


  coordinatesToAngles(&currentPosition, &servoSet);
  Serial.print("angle 1 : ");
  Serial.println(servoSet.servoLeft.angleCommand);
  Serial.print("angle 2 : ");
  Serial.println(servoSet.servoRight.angleCommand);
  Serial.print("angle 3 : ");
  Serial.println(servoSet.servoZ.angleCommand);
  servoLeft.write(servoSet.servoLeft.angleCommand);
  servoRight.write(servoSet.servoRight.angleCommand);
  servoZ.write(servoSet.servoZ.angleCommand); 

  servoSet.servoLeft.currentAngle = servoSet.servoLeft.angleCommand;
  servoSet.servoRight.currentAngle = servoSet.servoRight.angleCommand;
  servoSet.servoZ.currentAngle = servoSet.servoZ.angleCommand;
  
  delay(5000);
  /*
  initializeFromCartesianPosition(initX, initY, initZ); //takes obstacles into account
  */
}

// ======================================================
// LOOP
// ======================================================
void loop() {

  Serial.println("loop");
  
  do {
      cycleExecution(&destination, &prehensionStatus, &cycleStepIndex); //cycleStepIndex becomes the one of the next step here
      coordinatesToAngles(&destination, &servoSet);
      applyServoCommand(&servoSet, 50);
      if(prehensionStatus==0 || prehensionStatus==1) {
          Serial.println("prehension action");
          Serial.println(prehensionStatus);
          Serial.println("");
          delay(2000);
      }
      Serial.println("////////////////////////////////////////////////");
      Serial.println("Prehension status : ");
      Serial.print(prehensionStatus);
      Serial.println("cycleStep index : ");
      Serial.print(cycleStepIndex-1);
      Serial.println("etape réussie : ");
      Serial.print(servoSet.reachable);
      Serial.println("");
      delay(1500);
    }while(cycleStepIndex!=0);
}


void fullyApplyCommandToServos(ServoSet* servoSet, int delayCommand) {
  if (!servoSet->reachable) return;

  while (!(abs(servoSet->servoLeft.currentAngle - servoSet->servoLeft.angleCommand) < 0.05 &&
      abs(servoSet->servoRight.currentAngle - servoSet->servoRight.angleCommand) < 0.05 &&
      abs(servoSet->servoZ.currentAngle - servoSet->servoZ.angleCommand) < 0.05)) {

        applyServoCommand(servoSet, delayCommand);
        servoLeft.write(servoSet->servoLeft.currentAngle);
        servoRight.write(servoSet->servoRight.currentAngle);
        servoZ.write(servoSet->servoZ.currentAngle);
  }

  delay(delayCommand);
}
