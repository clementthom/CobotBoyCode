#include "../main/common.h" //lists structs and other elements shared by different modules
#include "../main/cinematics.h" //cinematic-related functions and structs (position + servo controls)
#include "../main/debug.h" //debug functions, mainly console tests returns
#include "../main/trajectories.h" //code related to registered trajectories and speed profiles
#include "../main/userInterface.h" //manages the control panel components and screens
#include "../main/maintenance.h" //serial communication with a computer using serial port (maintenance mode)

#include "../main/cinematics.cpp" //cinematic-related functions and structs (position + servo controls)
#include "../main/debug.cpp" //debug functions, mainly console tests returns
#include "../main/trajectories.cpp" //code related to registered trajectories and speed profiles
#include "../main/userInterface.cpp" //manages the control panel components and screens
#include "../main/maintenance.cpp" //serial communication with a computer using serial port (maintenance mode)


void testApplyCycle();
void testCyclePerf();;


int main() {
    //testApplyCycle();
    testCyclePerf();
}


void testApplyCycle() {
    Coordinates coordinates;
    ServoSet servoSet;
    PrehensionStatus prehensionStatus = UNKNOWN_STATUS;
    int cycleStepIndex=0;

    initCoordinates();
    initZone();
    printf("convoyeur entrée : \n x : %f\n  y : %f\n  z : %f\n", convoyeurEntree.x, convoyeurEntree.y,
    convoyeurEntree.z);

    coordinates.x=0.0;
    coordinates.y=-100.0;
    coordinates.z=50.0;

    servoSet.servoLeft.angleOffset=-20.0;
    servoSet.servoRight.angleOffset=10.0;
    servoSet.servoZ.angleOffset=2.5;

    servoSet.servoLeft.maxStep = 1.2;
    servoSet.servoRight.maxStep = 1.2;
    servoSet.servoZ.maxStep = 1.5;


    coordinatesToAngles(&coordinates, &servoSet);

    servoSet.servoLeft.currentAngle=servoSet.servoLeft.angleCommand;
    servoSet.servoRight.currentAngle=servoSet.servoRight.angleCommand;
    servoSet.servoZ.currentAngle=servoSet.servoZ.angleCommand;

    printf("0theta 1 : %f \n 0theta 2 : %f \n 0theta 3 : %f \n",
            servoSet.servoLeft.angleCommand, servoSet.servoRight.angleCommand, servoSet.servoZ.angleCommand);
    printf("0reacheable : %d \n", servoSet.reachable);

    
    printf("\n Prehension status : %d \n cycleStep index : %d\n", prehensionStatus, cycleStepIndex);
    do {
        cycleExecution(&coordinates, &prehensionStatus, &cycleStepIndex); //cycleStepIndex becomes the one of the next step here
        coordinatesToAngles(&coordinates, &servoSet);
        applyServoCommand(&servoSet, 50);
        if(prehensionStatus==0 || prehensionStatus==1) {
            printf("\n\n prehension action : %d \n\n", prehensionStatus);
            delay(2000);
        }
        printf("////////////////////////////////////////////////");
        printf("\n Prehension status : %d \n cycleStep index : %d\n etape réussie : %d\n", prehensionStatus, cycleStepIndex-1, servoSet.reachable);
        delay(1500);
    }while(cycleStepIndex!=0);
    
}


void testCyclePerf() {
    Coordinates currentPosition = initPosition;
    Coordinates destination;
    Coordinates intermediatePosition;
    ServoSet servoSet;

    PrehensionStatus prehensionStatus;
    IntermediatePoint intermediatePoint;
    int cycleStepIndex=0;

    initCoordinates();
    initZone();
    currentPosition=initPosition;

    servoSet.servoLeft.angleOffset=-20.0;
    servoSet.servoRight.angleOffset=10.0;
    servoSet.servoZ.angleOffset=2.5;

    servoSet.servoLeft.maxStep = 1.2;
    servoSet.servoRight.maxStep = 1.2;
    servoSet.servoZ.maxStep = 1.5;

    coordinatesToAngles(&currentPosition, &servoSet);
    servoSet.servoLeft.currentAngle=servoSet.servoLeft.angleCommand;
    servoSet.servoRight.currentAngle=servoSet.servoRight.angleCommand;
    servoSet.servoZ.currentAngle=servoSet.servoZ.angleCommand;


    do {
        cycleExecution(&destination, &prehensionStatus, &cycleStepIndex); //cycleStepIndex becomes the one of the next step here
        if(prehensionStatus==0 || prehensionStatus==1) {
            delay(500);
        }
        else {
            do {
                if(abs(currentPosition.x-convoyeurEntree.x)<1.0 && abs(currentPosition.y-convoyeurEntree.y)<1.0 
                && abs(currentPosition.z-convoyeurEntree.z)<1.0) {
                    printf("\n\nconvoyeurEntree ok\n");
                }
                if(abs(currentPosition.x+155.0)<1.0 && abs(currentPosition.y+19.0)<1.0 
                && abs(currentPosition.z-100.0)<1.0) {
                    printf("\n\npoint A ok\n");
                }
                if(abs(currentPosition.x+155.0)<1.0 && abs(currentPosition.y+152.0)<1.0 
                && abs(currentPosition.z-100.0)<1.0) {
                    printf("\n\nPoint B ok\n");
                }
                if(abs(currentPosition.x-machineA.x)<1.0 && abs(currentPosition.y-machineA.y)<1.0 
                && abs(machineA.z-convoyeurEntree.z)<1.0) {
                    printf("\n\nmachine A ok\n");
                }

                intermediatePosition = trajectoryProfile(currentPosition, destination, 100, PERFORMANCE, &intermediatePoint);
                coordinatesToAngles(&intermediatePosition, &servoSet);
                applyServoCommand(&servoSet, 50);
                anglesToCoordinates(&servoSet, &currentPosition);
            }while(intermediatePoint!=DESTINATION_POINT) ;
        
            intermediatePoint=DEPART_POINT; //re-initialisation trajectory profile
        
        }
    }while(cycleStepIndex!=0);
}

