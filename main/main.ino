#include <Servo.h>
#include "common.h" //lists structs and other elements shared by different modules
#include "cinematics.h" //cinematic-related functions and structs (position + servo controls)
#include "trajectories.h" //code related to registered trajectories and speed profiles
#include "maintenance.h" //serial communication with a computer using serial port (maintenance mode)

//Prototypes
void fullyApplyCommandToServos(ServoSet *servoSet, int delayCommandAngle , SpeedProfileType speedProfileType, int depthPercentage, 
    CycleMode cycleMode, int *elapsedTimeServoCycle, float *anglePerformedDuringAcceleration, int *remainingCycleTime);

//Components pins
#define SERVO_LEFT_PIN 5
#define SERVO_RIGHT_PIN 9
#define SERVO_Z_PIN 11

//servo objects declarations
Servo servoLeft;
Servo servoRight;
Servo servoZ;

// current coordinates, servo set and object declaration
Coordinates currentPosition;
Coordinates destination;
Coordinates intermediatePosition;
ServoSet servoSet;
Object currentObject;

//cycle related variables declaration
PrehensionStatus prehensionStatus;
SelectedCycle selectedCycle;
CycleMode cycleMode;
IntermediatePoint intermediatePoint;
int cycleStepIndex;
int delayCommandServo; //delay between servo command execution (servo cycle) in ms (servo cycle step duration)

//speed profile related declaration
SpeedProfileType speedProfileType;
int elapsedTimeServoCycle;
int remainingCycleTime;
float anglePerformedDuringAcceleration = 0;
int depthPercentage = 100;


bool cycleExecuted = false;

// ======================================================
// SETUP
// ======================================================
void setup() {

  Serial.begin(9600);

  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);
  servoZ.attach(SERVO_Z_PIN);


  currentObject = objectList.gomme;

  prehensionStatus=UNKNOWN_STATUS;
  selectedCycle = CYCLE1;
  cycleMode = PERFORMANCE;
  delayCommandServo=10;
  cycleStepIndex=0;
  

  speedProfileType = TRAPESOIDAL_LINEAR;
  elapsedTimeServoCycle = 0;
  remainingCycleTime=0;
  anglePerformedDuringAcceleration=0;
  depthPercentage = 50;



  initCoordinates(&selectedCycle);
  initZone();
  initServoSet(&servoSet, delayCommandServo);
  objectListInit();
  currentPosition=initPosition;
  currentObject = objectList.gomme;



  coordinatesToAngles(&currentPosition, &servoSet, &currentObject);
  servoLeft.write(servoSet.servoLeft.angleCommand);
  servoRight.write(servoSet.servoRight.angleCommand);
  servoZ.write(servoSet.servoZ.angleCommand); 


  servoSet.servoLeft.currentAngle = servoSet.servoLeft.angleCommand;
  servoSet.servoRight.currentAngle = servoSet.servoRight.angleCommand;
  servoSet.servoZ.currentAngle = servoSet.servoZ.angleCommand; 
  delay(200);

  destination.x=160;
  destination.y=-30;
  destination.z=100;
}

// ======================================================
// LOOP
// ======================================================
void loop() {


  coordinatesToAngles(&destination, &servoSet, &currentObject);
  fullyApplyCommandToServos(&servoSet, delayCommandServo, speedProfileType, depthPercentage, cycleMode, 
        &elapsedTimeServoCycle, &anglePerformedDuringAcceleration, &remainingCycleTime);
  anglesToCoordinates(&servoSet, &currentPosition, &currentObject);

  /*
  if(!cycleExecuted) {*1.06-4
  do {
    cycleExecution(&destination, &prehensionStatus, &cycleStepIndex); //cycleStepIndex becomes the one of the next step here
    if(prehensionStatus==0 || prehensionStatus==1) {
      Serial.println("prehension action");
      Serial.println(prehensionStatus);
      Serial.println("");
      //actionPrehension();
      delay(500);
    }
    else {
      do {
        intermediatePosition = trajectoryProfile(currentPosition, destination, 100, PERFORMANCE, &intermediatePoint);
        affectInitialServoPosition(&servoSet);
        coordinatesToAngles(&intermediatePosition, &servoSet);
        fullyApplyCommandToServos(&servoSet, delayCommandServo, speedProfileType, depthPercentage, cycleMode, 
        &elapsedTimeServoCycle, &anglePerformedDuringAcceleration, &remainingCycleTime);
        anglesToCoordinates(&servoSet, &currentPosition);

        elapsedTimeServoCycle=0;
        remainingCycleTime=0;
        anglePerformedDuringAcceleration = 0;
      }while(intermediatePoint!=DESTINATION_POINT) ;
        
      intermediatePoint=DEPART_POINT; //re-initialisation trajectory profile   
    }
  }while(cycleStepIndex!=0);
  }
  */
  cycleExecuted = false;
}


void fullyApplyCommandToServos(ServoSet *servoSet, int delayCommandAngle , SpeedProfileType speedProfileType, int depthPercentage, 
    CycleMode cycleMode, int *elapsedTimeServoCycle, float *anglePerformedDuringAcceleration, int *remainingCycleTime) {

  *anglePerformedDuringAcceleration=0;
  //this variable will be used for non-constant profiles to decelerate - value to 0 here to allow it to be initialised
  *remainingCycleTime = -1;

  if (!servoSet->reachable) return;

  while (!(abs(servoSet->servoLeft.currentAngle - servoSet->servoLeft.angleCommand) < 0.05 &&
      abs(servoSet->servoRight.currentAngle - servoSet->servoRight.angleCommand) < 0.05 &&
      abs(servoSet->servoZ.currentAngle - servoSet->servoZ.angleCommand) < 0.05)) {

        applyServoCommand(servoSet, delayCommandAngle, speedProfileType, depthPercentage, cycleMode, elapsedTimeServoCycle, 
                        anglePerformedDuringAcceleration, remainingCycleTime);
        servoLeft.write(servoSet->servoLeft.currentAngle-37); 
        servoRight.write(servoSet->servoRight.currentAngle*0.955);
        servoZ.write(map(servoSet->servoZ.currentAngle, 0, 173, 0,180));

        delay(delayCommandAngle);
        *elapsedTimeServoCycle +=delayCommandAngle;
        if(*remainingCycleTime!=-1) {//if remainingCycleTime initialised
            *remainingCycleTime-=delayCommandAngle;
        }
  }
}


///Mapping

//robot 2
//servoZ : map(angle, 0,173,0,180)

//servoLeft map(angle, 0,215,33,173)
//servoLeft.write(90-36); //pour 90°
//angle de butée (dans le write) du servoLeft : 174° (au delà le robot peut s'endommager)


//servoRight.write(90*log(angle)-314); (régression linéaire)
