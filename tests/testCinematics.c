#include <stdio.h>
#include "../main/cinematics.h"
#include "../main/cinematics.c"


/////Prototypes
void testCoordinatesChange();
void testAngles();
void testApplyServoCommand();
void testLimitStep();

///////////////

int main() {
    //testCoordinatesChange();
    //testAngles();
    testApplyServoCommand();
    //testLimitStep();
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

    coordinates.x=100;
    coordinates.y=100;
    coordinates.z=100;

    servoSet.servoLeft.angleOffset=-20.0;
    servoSet.servoRight.angleOffset=10.0;
    servoSet.servoZ.angleOffset=3.0;

    coordinatesToAngles(&coordinates, &servoSet);

    printf("theta 1 : %f \n theta 2 : %f \n theta 3 : %f \n",
        servoSet.servoLeft.angleCommand, servoSet.servoRight.angleCommand, servoSet.servoZ.angleCommand);
    printf("reacheable : %d \n", servoSet.reachable);
}

void testApplyServoCommand() {
    Coordinates coordinates;
    ServoSet servoSet;

    coordinates.x=100;
    coordinates.y=100;
    coordinates.z=100;

    servoSet.servoLeft.angleOffset=-20.0;
    servoSet.servoRight.angleOffset=10.0;
    servoSet.servoZ.angleOffset=2.5;

    servoSet.servoLeft.currentAngle=15;
    servoSet.servoRight.currentAngle=30;
    servoSet.servoZ.currentAngle=45;

    servoSet.servoLeft.maxStep = 1.2;
    servoSet.servoRight.maxStep = 1.2;
    servoSet.servoZ.maxStep = 1.5;

    coordinatesToAngles(&coordinates, &servoSet);
    printf("theta 1 : %f \n theta 2 : %f \n theta 3 : %f \n",
        servoSet.servoLeft.angleCommand, servoSet.servoRight.angleCommand, servoSet.servoZ.angleCommand);
    printf("reacheable : %d \n", servoSet.reachable);
    applyServoCommand(&servoSet, 5);
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

    coordinatesToAngles(&coordinates, &servoSet);
    printf("theta 1 : %f \n theta 2 : %f \n theta 3 : %f \n",
        servoSet.servoLeft.angleCommand, servoSet.servoRight.angleCommand, servoSet.servoZ.angleCommand);
    printf("reacheable : %d \n", servoSet.reachable);


    servoSet.servoLeft.currentAngle = limitStep(servoSet.servoLeft.currentAngle, servoSet.servoLeft.angleCommand, 
        servoSet.servoLeft.maxStep); //limits angle variation accordingly to the set servo parameter

    printf("angle servoLeft : %f", servoSet.servoLeft.currentAngle);
}