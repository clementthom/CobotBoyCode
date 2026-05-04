#include <stdio.h>
#include "../main/cinematics.h"
#include "../main/cinematics.cpp"


/////Prototypes
void testCoordinatesChange();
void testAngles();
void testApplyServoCommand();
void testLimitStep();
void testAnglesAndCoordinates();
void testSpeedProfileLinear();
void testSpeedProfileTrapesoidalLinear();
void testSpeedProfileTrapesoidalExp(); 

///////////////

int main() {
    //testCoordinatesChange();
    //testAngles();
    //testApplyServoCommand();
    //testLimitStep();
    //testAnglesAndCoordinates();
    //testSpeedProfileLinear(); 
    testSpeedProfileTrapesoidalLinear();
    //testSpeedProfileTrapesoidalExp();
    printf("fin");
}



void testCoordinatesChange() {
    Coordinates coordinates;
    coordinates.x=0;
    coordinates.y=0;
    coordinates.z=0;

    coordinatesChange(&coordinates, 10, 10,10);
    printf("x : %f \n y : %f \n z : %f \n ", coordinates.x, coordinates.y, coordinates.z);
}

void testAngles() {
    Coordinates coordinates;
    ServoSet servoSet;

    coordinates.x=-0.0;
    coordinates.y=-150.0;
    coordinates.z=10.0;

    Object object;
    object.consigne=0.0;
    object.height=0.0;
    object.objectName=GOMME;
    object.radius=0.0;


    coordinatesToAngles(&coordinates, &servoSet, &object);

    printf("positions : \n x : %f   \ny : %f    \nz : %f \n \n", coordinates.x, coordinates.y, coordinates.z);
    printf("angleLeft : %f\nangleRight : %f \nangleZ : %f \n",
        servoSet.servoLeft.angleCommand, servoSet.servoRight.angleCommand, servoSet.servoZ.angleCommand);
    printf("reacheable : %d \n", servoSet.reachable);
}

void testApplyServoCommand() {
    Coordinates coordinates;
    ServoSet servoSet;

    coordinates.x=-155.0;
    coordinates.y=-19.0;
    coordinates.z=5.0;

    servoSet.servoLeft.angleOffset=-20.0;
    servoSet.servoRight.angleOffset=10.0;
    servoSet.servoZ.angleOffset=2.5;

    servoSet.servoLeft.currentAngle=15;
    servoSet.servoRight.currentAngle=30;
    servoSet.servoZ.currentAngle=45;

    servoSet.servoLeft.maxStep = 1.2;
    servoSet.servoRight.maxStep = 1.2;
    servoSet.servoZ.maxStep = 1.5;

    Object object;
    object.consigne=0.0;
    object.height=0.0;
    object.objectName=GOMME;
    object.radius=0.0;


    coordinatesToAngles(&coordinates, &servoSet, &object);
    printf("theta 1 : %f \n theta 2 : %f \n theta 3 : %f \n",
        servoSet.servoLeft.angleCommand, servoSet.servoRight.angleCommand, servoSet.servoZ.angleCommand);
    printf("reacheable : %d \n", servoSet.reachable);
    //applyServoCommand(&servoSet, 5); //works with a former version of the code
}

void testLimitStep() {
    
    float currentValue = 10.0;
    float commandValue = 25.0;
    float step = 2.0;

    float result = limitStep(currentValue, commandValue, step);
    printf("result : %f \n \n \n", result);

    Coordinates coordinates;
    ServoSet servoSet;

    coordinates.x=100;
    coordinates.y=100;
    coordinates.z=100;

    servoSet.servoLeft.angleOffset=-20.0;
    servoSet.servoRight.angleOffset=10.0;
    servoSet.servoZ.angleOffset=3.0;

    servoSet.servoLeft.currentAngle=15;
    servoSet.servoRight.currentAngle=30;
    servoSet.servoZ.currentAngle=45;

    // Limitation de variation angulaire par cycle
    servoSet.servoLeft.maxStep = 1.2;
    servoSet.servoRight.maxStep = 1.2;
    servoSet.servoZ.maxStep = 1.5;

    Object object;
    object.consigne=0.0;
    object.height=0.0;
    object.objectName=GOMME;
    object.radius=0.0;


    coordinatesToAngles(&coordinates, &servoSet, &object);
    printf("theta 1 : %f \n theta 2 : %f \n theta 3 : %f \n",
        servoSet.servoLeft.angleCommand, servoSet.servoRight.angleCommand, servoSet.servoZ.angleCommand);
    printf("reacheable : %d \n", servoSet.reachable);


    servoSet.servoLeft.currentAngle = limitStep(servoSet.servoLeft.currentAngle, servoSet.servoLeft.angleCommand, 
        servoSet.servoLeft.maxStep); //limits angle variation accordingly to the set servo parameter

    printf("angle servoLeft : %f", servoSet.servoLeft.currentAngle);
}


void testAnglesAndCoordinates() {
    Coordinates input;
    Coordinates output;
    ServoSet servoSet;

    input.x=0.0;
    input.y=-180.0;
    input.z=100.0;

    servoSet.servoLeft.angleOffset=-8.0;
    servoSet.servoRight.angleOffset=-7.0;
    servoSet.servoZ.angleOffset=2.5;

    servoSet.servoLeft.maxStep = 1.2;
    servoSet.servoRight.maxStep = 1.2;
    servoSet.servoZ.maxStep = -2.0;

    Object object;
    object.consigne=0.0;
    object.height=0.0;
    object.objectName=GOMME;
    object.radius=0.0;


    coordinatesToAngles(&input, &servoSet, &object);
    servoSet.servoLeft.currentAngle=servoSet.servoLeft.angleCommand;
    servoSet.servoRight.currentAngle=servoSet.servoRight.angleCommand;
    servoSet.servoZ.currentAngle=servoSet.servoZ.angleCommand;

    anglesToCoordinates(&servoSet, &output, &object);
    printf("\n\ninput x : %f\ninput y : %f\ninput z : %f", input.x, input.y, input.z);
    printf("\n\noutput x : %f\noutput y : %f\noutput z : %f", output.x, output.y, output.z);

}


void testSpeedProfileLinear() {
    Coordinates currentPosition;
    Coordinates destination;
    Coordinates intermediatePosition;
    ServoSet servoSet;

    int elapsedTime=0;
    int elapsedTimeBeforeDeceleration = 0; 
    int remainingCycleTime=0;
    int delayCommandAngle = 20;
    float anglePerformedDuringAcceleration = 0;
    initServoSet(&servoSet, delayCommandAngle);


    currentPosition.x=-100.0;
    currentPosition.y=-20.0;
    currentPosition.z=5.0;

    intermediatePosition.x=-0.0;
    intermediatePosition.y=-200.0;
    intermediatePosition.z=5.0;

    destination.x=100.0;
    destination.y=-20.0;
    destination.z=100.0;


    Object object;
    object.consigne=0.0;
    object.height=0.0;
    object.objectName=GOMME;
    object.radius=0.0;


    coordinatesToAngles(&currentPosition, &servoSet, &object);
    servoSet.servoLeft.currentAngle=servoSet.servoLeft.angleCommand;
    servoSet.servoRight.currentAngle=servoSet.servoRight.angleCommand;
    servoSet.servoZ.currentAngle=servoSet.servoZ.angleCommand;


    affectInitialServoPosition(&servoSet);
    coordinatesToAngles(&intermediatePosition, &servoSet, &object);
    applyServoCommand(&servoSet, delayCommandAngle, CONSTANT, 100, PERFORMANCE, &elapsedTime, 
                        &anglePerformedDuringAcceleration, &remainingCycleTime);
    anglesToCoordinates(&servoSet, &currentPosition, &object);
    
    if(abs(currentPosition.x-intermediatePosition.x)<1 && 
        abs(currentPosition.y-intermediatePosition.y)<1 &&
        abs(currentPosition.z-intermediatePosition.z)<1) {

        printf("\n\nintermediate position reached\n\n");
    }

    affectInitialServoPosition(&servoSet);
    coordinatesToAngles(&destination, &servoSet, &object);
    applyServoCommand(&servoSet, delayCommandAngle, CONSTANT, 100, PERFORMANCE, &elapsedTime, 
                        &anglePerformedDuringAcceleration, &remainingCycleTime);
    anglesToCoordinates(&servoSet, &currentPosition, &object);


    if(abs(currentPosition.x==intermediatePosition.x)<1 && 
        abs(currentPosition.y==intermediatePosition.y)<1 &&
        abs(currentPosition.z==intermediatePosition.z)<1) {

        printf("\n\ndestination position reached\n\n");
    }
}


void testSpeedProfileTrapesoidalLinear() {
    Coordinates currentPosition;
    Coordinates destination;
    Coordinates intermediatePosition;
    ServoSet servoSet;

    int delayCommandAngle = 20;
    int elapsedTime=0;
    int elapsedTimeBeforeDeceleration = 0; 
    int remainingCycleTime=0;
    float anglePerformedDuringAcceleration = 0;
    initServoSet(&servoSet, delayCommandAngle);


    currentPosition.x=-100.0;
    currentPosition.y=-20.0;
    currentPosition.z=5.0;

    intermediatePosition.x=-0.0;
    intermediatePosition.y=-200.0;
    intermediatePosition.z=5.0;

    destination.x=100.0;
    destination.y=-20.0;
    destination.z=100.0;


    Object object;
    object.consigne=0.0;
    object.height=0.0;
    object.objectName=GOMME;
    object.radius=0.0;


    coordinatesToAngles(&currentPosition, &servoSet, &object);
    servoSet.servoLeft.currentAngle=servoSet.servoLeft.angleCommand;
    servoSet.servoRight.currentAngle=servoSet.servoRight.angleCommand;
    servoSet.servoZ.currentAngle=servoSet.servoZ.angleCommand;


    affectInitialServoPosition(&servoSet);
    coordinatesToAngles(&intermediatePosition, &servoSet, &object);
    applyServoCommand(&servoSet, delayCommandAngle, TRAPESOIDAL_LINEAR, 100, PERFORMANCE, &elapsedTime, 
                        &anglePerformedDuringAcceleration, &remainingCycleTime);
    anglesToCoordinates(&servoSet, &currentPosition, &object);
    
    if(abs(currentPosition.x-intermediatePosition.x)<1 && 
        abs(currentPosition.y-intermediatePosition.y)<1 &&
        abs(currentPosition.z-intermediatePosition.z)<1) {

        printf("\n\nintermediate position reached\n\n");
    }

    elapsedTime=0;
    elapsedTimeBeforeDeceleration = 0; 
    remainingCycleTime=0;
    anglePerformedDuringAcceleration = 0;


    affectInitialServoPosition(&servoSet);
    coordinatesToAngles(&destination, &servoSet, &object);
    applyServoCommand(&servoSet, delayCommandAngle, TRAPESOIDAL_LINEAR, 100, PERFORMANCE, &elapsedTime, 
                        &anglePerformedDuringAcceleration, &remainingCycleTime);
    anglesToCoordinates(&servoSet, &currentPosition, &object);


    if(abs(currentPosition.x==intermediatePosition.x)<1 && 
        abs(currentPosition.y==intermediatePosition.y)<1 &&
        abs(currentPosition.z==intermediatePosition.z)<1) {

        printf("\n\ndestination position reached\n\n");
    }

    elapsedTime=0;
    elapsedTimeBeforeDeceleration = 0; 
    remainingCycleTime=0;
    anglePerformedDuringAcceleration = 0;
}



void testSpeedProfileTrapesoidalExp() {
    Coordinates currentPosition;
    Coordinates destination;
    Coordinates intermediatePosition;
    ServoSet servoSet;

    int delayCommandAngle = 20;
    int elapsedTime=0;
    int elapsedTimeBeforeDeceleration = 0; 
    int remainingCycleTime=0;
    float anglePerformedDuringAcceleration = 0;
    initServoSet(&servoSet, delayCommandAngle);


    currentPosition.x=-100.0;
    currentPosition.y=-20.0;
    currentPosition.z=5.0;

    intermediatePosition.x=-0.0;
    intermediatePosition.y=-200.0;
    intermediatePosition.z=5.0;

    destination.x=100.0;
    destination.y=-20.0;
    destination.z=100.0;


    Object object;
    object.consigne=0.0;
    object.height=0.0;
    object.objectName=GOMME;
    object.radius=0.0;


    coordinatesToAngles(&currentPosition, &servoSet, &object);
    servoSet.servoLeft.currentAngle=servoSet.servoLeft.angleCommand;
    servoSet.servoRight.currentAngle=servoSet.servoRight.angleCommand;
    servoSet.servoZ.currentAngle=servoSet.servoZ.angleCommand;


    affectInitialServoPosition(&servoSet);
    coordinatesToAngles(&intermediatePosition, &servoSet, &object);
    applyServoCommand(&servoSet, delayCommandAngle, TRAPESOIDAL_EXPONENTIAL, 100, PERFORMANCE, &elapsedTime, 
                        &anglePerformedDuringAcceleration, &remainingCycleTime);
    anglesToCoordinates(&servoSet, &currentPosition, &object);
    
    if(abs(currentPosition.x-intermediatePosition.x)<1 && 
        abs(currentPosition.y-intermediatePosition.y)<1 &&
        abs(currentPosition.z-intermediatePosition.z)<1) {

        printf("\n\nintermediate position reached\n\n");
    }

    elapsedTime=0;
    elapsedTimeBeforeDeceleration = 0;
    remainingCycleTime=0;


    affectInitialServoPosition(&servoSet);
    coordinatesToAngles(&destination, &servoSet, &object);
    applyServoCommand(&servoSet, delayCommandAngle, TRAPESOIDAL_EXPONENTIAL, 100, PERFORMANCE, &elapsedTime, 
                        &anglePerformedDuringAcceleration, &remainingCycleTime);
    anglesToCoordinates(&servoSet, &currentPosition, &object);


    if(abs(currentPosition.x==intermediatePosition.x)<1 && 
        abs(currentPosition.y==intermediatePosition.y)<1 &&
        abs(currentPosition.z==intermediatePosition.z)<1) {

        printf("\n\ndestination position reached\n\n");
    }

    elapsedTime=0;
    elapsedTimeBeforeDeceleration = 0; 
    remainingCycleTime=0;
    anglePerformedDuringAcceleration = 0;
}