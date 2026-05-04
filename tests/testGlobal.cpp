#include "../main/common.h" //lists structs and other elements shared by different modules
#include "../main/cinematics.h" //cinematic-related functions and structs (position + servo controls)
#include "../main/trajectories.h" //code related to registered trajectories and speed profiles
#include "../main/maintenance.h" //serial communication with a computer using serial port (maintenance mode)

#include "../main/cinematics.cpp" //cinematic-related functions and structs (position + servo controls)
#include "../main/trajectories.cpp" //code related to registered trajectories and speed profiles
#include "../main/maintenance.cpp" //serial communication with a computer using serial port (maintenance mode)


void testApplyCycle();
void testCyclePerf();
void testCycleEco();
void testApplyCycleSpeedProfile();


int main() {
    //testApplyCycle();
    //testCyclePerf();
    //testCycleEco();
    testApplyCycleSpeedProfile();
}


void testApplyCycle() {
    Coordinates coordinates;
    ServoSet servoSet;
    PrehensionStatus prehensionStatus = UNKNOWN_STATUS;
    int cycleStepIndex=0;
    SelectedCycle selectedCycle = CYCLE2;

    initCoordinates(&selectedCycle);
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

    Object object;
    object.consigne=0.0;
    object.prehensionHeight=0.0;
    object.objectName=GOMME;
    object.radius=0.0;


    coordinatesToAngles(&coordinates, &servoSet, &object);

    servoSet.servoLeft.currentAngle=servoSet.servoLeft.angleCommand;
    servoSet.servoRight.currentAngle=servoSet.servoRight.angleCommand;
    servoSet.servoZ.currentAngle=servoSet.servoZ.angleCommand;

    printf("0theta 1 : %f \n 0theta 2 : %f \n 0theta 3 : %f \n",
            servoSet.servoLeft.angleCommand, servoSet.servoRight.angleCommand, servoSet.servoZ.angleCommand);
    printf("0reacheable : %d \n", servoSet.reachable);

    
    printf("\n Prehension status : %d \n cycleStep index : %d\n", prehensionStatus, cycleStepIndex);
    do {
        cycleExecution(&coordinates, &prehensionStatus, &cycleStepIndex, &selectedCycle); //cycleStepIndex becomes the one of the next step here
        coordinatesToAngles(&coordinates, &servoSet, &object);
        //applyServoCommand(&servoSet, 50); works with older definition of applyServoCommand
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
    /*
    Coordinates currentPosition = initPosition;
    Coordinates destination;
    Coordinates intermediatePosition;
    ServoSet servoSet;

    PrehensionStatus prehensionStatus;
    IntermediatePoint intermediatePoint;
    int cycleStepIndex = 0;
    int elapsedTimeServoCycle = 0;
    int delayCommandAngle = 20;

    initCoordinates();
    initZone();
    initServoSet(&servoSet, delayCommandAngle);
    currentPosition=initPosition;


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
                applyServoCommand(&servoSet, delayCommandAngle, CONSTANT, 100, PERFORMANCE, &elapsedTimeServoCycle);
                anglesToCoordinates(&servoSet, &currentPosition);
            }while(intermediatePoint!=DESTINATION_POINT) ;
        
            intermediatePoint=DEPART_POINT; //re-initialisation trajectory profile
        
        }
    }while(cycleStepIndex!=0);
    */
}

void testCycleEco() {
    /*
    Coordinates currentPosition = initPosition;
    Coordinates intermediatePosition;
    Coordinates destination;
    ServoSet servoSet;

    PrehensionStatus prehensionStatus;
    IntermediatePoint intermediatePoint;
    int cycleStepIndex = 0;
    int elapsedTimeServoCycle = 0;
    int delayCommandAngle = 20;

    initCoordinates();
    initZone();
    initServoSet(&servoSet, delayCommandAngle);
    currentPosition=initPosition;


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
                && abs(currentPosition.z-80.0)<1.0) {
                    printf("\n\npoint A-1 ok\n");
                }
                if(abs(currentPosition.x+155.0)<1.0 && abs(currentPosition.y+45.6)<1.0 
                && abs(currentPosition.z-100.0)<1.0) {
                    printf("\n\npoint A-2 ok\n");
                }
                if(abs(currentPosition.x+155.0)<1.0 && abs(currentPosition.y+125.0)<1.0 
                && abs(currentPosition.z-100.0)<1.0) {
                    printf("\n\nPoint B-1 ok\n");
                }
                if(abs(currentPosition.x+155.0)<1.0 && abs(currentPosition.y+152.0)<1.0 
                && abs(currentPosition.z-80.0)<1.0) {
                    printf("\n\nPoint B-2 ok\n");
                }
                if(abs(currentPosition.x-machineA.x)<1.0 && abs(currentPosition.y-machineA.y)<1.0 
                && abs(machineA.z-convoyeurEntree.z)<1.0) {
                    printf("\n\nmachine A ok\n");
                }
                intermediatePosition = trajectoryProfile(currentPosition, destination, 100, ECO, &intermediatePoint);
                coordinatesToAngles(&intermediatePosition, &servoSet);
                applyServoCommand(&servoSet, delayCommandAngle, CONSTANT, 100, PERFORMANCE, &elapsedTimeServoCycle);
                anglesToCoordinates(&servoSet, &currentPosition);
                affectInitialServoPosition(&servoSet);

            }while(intermediatePoint!=DESTINATION_POINT) ;
        
            intermediatePoint=DEPART_POINT; //re-initialisation trajectory profile        
        }
    }while(cycleStepIndex!=0);
    */
}


void testApplyCycleSpeedProfile() {
    Coordinates currentPosition;
    Coordinates destination;
    Coordinates intermediatePosition;
    ServoSet servoSet;

    PrehensionStatus prehensionStatus = UNKNOWN_STATUS;
    SelectedCycle selectedCycle = CYCLE1;
    IntermediatePoint intermediatePoint = DEPART_POINT;
    int delayCommandAngle = 20;
    int cycleStepIndex = 0;

    int elapsedTimeServoCycle=0;
    int remainingCycleTime=0;
    float anglePerformedDuringAcceleration = 0;
    int depthPercentage = 50;
    SpeedProfileType speedProfileType = TRAPESOIDAL_EXPONENTIAL;


    Object object = objectList.gomme;


    initCoordinates(&selectedCycle);
    initZone();
    initServoSet(&servoSet, delayCommandAngle);
    currentPosition=initPosition;


    coordinatesToAngles(&currentPosition, &servoSet, &object);
    servoSet.servoLeft.currentAngle=servoSet.servoLeft.angleCommand;
    servoSet.servoRight.currentAngle=servoSet.servoRight.angleCommand;
    servoSet.servoZ.currentAngle=servoSet.servoZ.angleCommand;

    
    printf("\n Prehension status : %d \n cycleStep index : %d\n", prehensionStatus, cycleStepIndex);
    do {
        cycleExecution(&destination, &prehensionStatus, &cycleStepIndex, &selectedCycle);  //cycleStepIndex becomes the one of the next step here
        if(prehensionStatus==0 || prehensionStatus==1) {
            delay(500);
            printf("////////////////////////////////////////////////\n");
        }
        else {
            do {
                intermediatePosition = trajectoryProfile(currentPosition, destination, 100, PERFORMANCE, &intermediatePoint);
                affectInitialServoPosition(&servoSet);
                coordinatesToAngles(&intermediatePosition, &servoSet, &object);
                applyServoCommand(&servoSet, delayCommandAngle, speedProfileType, depthPercentage, PERFORMANCE, &elapsedTimeServoCycle, 
                        &anglePerformedDuringAcceleration, &remainingCycleTime);
                anglesToCoordinates(&servoSet, &currentPosition, &object);
                
                elapsedTimeServoCycle=0;
                remainingCycleTime=0;
                anglePerformedDuringAcceleration = 0;

            }while(intermediatePoint!=DESTINATION_POINT) ;
        
            intermediatePoint=DEPART_POINT; //re-initialisation trajectory profile
        }
        printf("\n Prehension status : %d \n cycleStep index : %d\n", prehensionStatus, cycleStepIndex);
    }while(cycleStepIndex!=0);
    
}