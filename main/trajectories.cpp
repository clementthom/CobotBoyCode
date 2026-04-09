#include "trajectories.h"


Coordinates convoyeurEntree;
Coordinates convoyeurSortie;
Coordinates machineA;
Coordinates machineB;
Coordinates initPosition;

Zone obstacle1;
Zone workingZone;


/**
 * Function : initCoordinates
 * --------------
 * affects values to action points coordinates (initialisation)
 * 
 * no parameters
 * 
 * returns : no returns (void)
 */

void initCoordinates() {
    convoyeurEntree.x=-150.0;
    convoyeurEntree.y=-5.0;
    convoyeurEntree.z=0.5;

    convoyeurSortie.x=150.0;
    convoyeurSortie.y=-5.0;
    convoyeurSortie.z=0.5;

    machineA.x=-150.0;
    machineA.y=-180.0;
    machineA.z=50.0;

    machineB.x=150.0;
    machineB.y=-180.0;
    machineB.z=0.5;

    initPosition.x=0;
    initPosition.y=-180;
    initPosition.z=100;
}
/**
 * Function : initZone
 * --------------
 * affects values to points coordinates delimiting all declared zones (initialisation)
 * 
 * no parameters
 * 
 * returns : no returns (void)
 */
void initZone() {
    workingZone.origin.x=-300.0;
    workingZone.origin.y=250.0;
    workingZone.origin.z=0.0;

    workingZone.farthestPointFromOrigin.x=300.0;
    workingZone.farthestPointFromOrigin.y=-250.0;
    workingZone.farthestPointFromOrigin.z=700.0;

    workingZone.zoneType=WORKING_ZONE;


    obstacle1.origin.x=50.0;
    obstacle1.origin.y=-120.0;
    obstacle1.origin.z=0.0;

    obstacle1.farthestPointFromOrigin.x=120.0;
    obstacle1.farthestPointFromOrigin.y=-250.0;
    obstacle1.farthestPointFromOrigin.z=110.0;

    obstacle1.zoneType=OBSTACLE;
}

/**
 * Function : checkIfInZone
 * --------------------
 * detects if a point is in a zone
 * 
 * coordinates : the coordinates of the point to check
 * zone : the zone in which the presence of the point should be verified
 * 
 * returns : 0 if the point is not in the zone, 1 if it is
 */
int checkIfInZone(Coordinates coordinates, Zone zone) {
    //check for each axis if the point is in bounds
    if((abs(coordinates.x) < abs(zone.origin.x)-5.0 || abs(coordinates.x)< abs(zone.farthestPointFromOrigin.x)-5.0) &&
    (abs(coordinates.y) < abs(zone.origin.y)-5.0 || abs(coordinates.y)<abs(zone.farthestPointFromOrigin.y))-5.0 &&
    (abs(coordinates.z) < abs(zone.origin.z)-5.0 || abs(coordinates.z)<abs(zone.farthestPointFromOrigin.z)-5.0)) {
        return 1;
    }
    else {
        return 0;
    }
} 

/**
 * Function : checkIfCloseToObstacle
 * --------------------
 * computes the distances between an object and a zone boundaries for all axis (x, y, z)
 * 
 * coordinates : the coordinates of the point to check
 * zone : the zone to compare with the point
 * 
 * returns : the minimum distance for each axis and the zone type (see ZoneDistance struct)
 */
ZoneDistances checkIfCloseToObstacle(Coordinates coordinates, Zone zone) {
    ZoneDistances zoneDistance;
    zoneDistance.distances.x=1000.0;
    zoneDistance.distances.y=1000.0;
    zoneDistance.distances.z=1000.0;


    float difference = coordinates.x - zone.origin.x;
    if(abs(difference) < abs(zoneDistance.distances.x)) {
        zoneDistance.distances.x = difference;
    }
    difference = coordinates.x - zone.farthestPointFromOrigin.x;
    if(abs(difference) < abs(zoneDistance.distances.x)) {
        zoneDistance.distances.x = difference;
    }
    difference = coordinates.y - zone.origin.y;
    if(abs(difference) < abs(zoneDistance.distances.y)) {
        zoneDistance.distances.y = difference;
    }
    difference = coordinates.y - zone.farthestPointFromOrigin.y;
    if(abs(difference) < abs(zoneDistance.distances.y)) {
        zoneDistance.distances.y = difference;
    }
    difference = coordinates.z - zone.origin.z;
    if(abs(difference) < abs(zoneDistance.distances.z)) {
        zoneDistance.distances.z = difference;
    }
    difference = abs(coordinates.z)<abs(zone.farthestPointFromOrigin.z);
    if(abs(difference) < abs(zoneDistance.distances.z)) {
        zoneDistance.distances.z = difference;
    }


    //modification (if necessary) of the difference coordinates to accentuate their effects on the angle variations commands
    float* smallestDistance = NULL;
    if (zoneDistance.distances.x<zoneDistance.distances.y) {
        if(zoneDistance.distances.x<zoneDistance.distances.z) {
            smallestDistance = &zoneDistance.distances.x;
        }
        else {
            smallestDistance = &zoneDistance.distances.z;
        }
    }
    else {
        smallestDistance = &zoneDistance.distances.y;
    }
    printf("smallestDistance : %f", *smallestDistance);

    if(*smallestDistance<2.0) { //if smallest distance from one of the axis is smaller than 2 mm
        zoneDistance.distances.x = 1000.0; //big values so the angles of the servo don't answer to these parameters
        zoneDistance.distances.y = 1000.0;
        zoneDistance.distances.z = 1000.0;

        *smallestDistance = 1.0; //100/1.0 = 100% of servo power
    }
    if(*smallestDistance>80.0) {
        zoneDistance.distances.x = 0.0;//no need for trajectory correction
        zoneDistance.distances.y = 0.0;
        zoneDistance.distances.z = 0.0;
    }
    //else : the distance coordinates stay the same : no ponderation
    zoneDistance.zoneType = zone.zoneType;

    return zoneDistance;
}


void changeServosSteps() {
    
}


/**
 * Function : stepActions
 * --------------------
 * - change the input coordinates depending on the selected cycle step and define the action the prehension system has to perform
 * 
 * 
 * - stepToken : name of the selected robot action (CycleStep enum)
 * - coordinates : pointer of the coordinates to change (destination coordinates)
 * 
 * 
 * - returns : the action the prehension has to perform depending on the cycle step (PrehensionStatus enum), after coordinates modification
 */
PrehensionStatus stepActions(CycleStep stepToken, Coordinates* coordinates) {
    switch (stepToken) {
    case CONVOYEUR_ENTREE:
        *coordinates=convoyeurEntree;
        return KEEP_CURRENT_PREHENSION_POSITION;

    case CONVOYEUR_SORTIE:
        *coordinates=convoyeurSortie;
        return KEEP_CURRENT_PREHENSION_POSITION;

    case MACHINE_A:
        *coordinates=machineA;
        return KEEP_CURRENT_PREHENSION_POSITION;

    case MACHINE_B:
        *coordinates=machineB;
        return KEEP_CURRENT_PREHENSION_POSITION;

    case INIT:
        *coordinates=initPosition;
        return RELEASE;

    case GRAB_OBJECT:
        return GRAB;

    case RELEASE_OBJECT:
        return RELEASE;
    }
    return UNKNOWN_STATUS;
}

/**
 * Function : cycleExecution
 * --------------------
 * - executes the cycle steps sequence; defines the entire working cycle
 * 
 * - Parameters :
 * -- coordinates : pointer of the coordinates to change (destination coordinates)
 * -- actionPrehension : pointer of the action the prehension system will perform upon the step the cycle is at
 * -- nextStepIndex : pointer of the step to perform in the sequence
 * 
 * 
 * returns : nothing, changes the destination coordinates, defines the action to transmit to the prehension system and increments/reset to 0 the step sequence index (pointers)
 */
void cycleExecution(Coordinates* destinationCoordinates, PrehensionStatus* actionPrehension, int* nextStepIndex) {
    switch (*nextStepIndex) {
    case 0: //step 0 : initialisation
        *actionPrehension = stepActions(INIT, destinationCoordinates);
        *nextStepIndex ++;
        break;
    case 1: 
        *actionPrehension = stepActions(CONVOYEUR_ENTREE, destinationCoordinates);
        *nextStepIndex ++;
        break;
    case 2:
        *actionPrehension = stepActions(GRAB_OBJECT, destinationCoordinates);
        *nextStepIndex ++;
        break;
    case 3: 
        *actionPrehension = stepActions(MACHINE_A, destinationCoordinates);
        *nextStepIndex ++;
        break;
    case 4: 
        *actionPrehension = stepActions(RELEASE_OBJECT, destinationCoordinates);
        *nextStepIndex ++;
        break;
    case 5:
        *actionPrehension = stepActions(GRAB_OBJECT, destinationCoordinates);
        *nextStepIndex ++;
        break;
    case 6: 
        *actionPrehension = stepActions(MACHINE_B, destinationCoordinates);
        *nextStepIndex ++;
        break;
    case 7: 
        *actionPrehension = stepActions(RELEASE_OBJECT, destinationCoordinates);
        *nextStepIndex ++;
        break;
    case 8: 
        *actionPrehension = stepActions(GRAB_OBJECT, destinationCoordinates);
        *nextStepIndex ++;
        break;
    case 9:
        *actionPrehension = stepActions(CONVOYEUR_SORTIE, destinationCoordinates);
        *nextStepIndex ++;
        break;
    case 10: 
        *actionPrehension = stepActions(RELEASE_OBJECT, destinationCoordinates);
        *nextStepIndex =0;
        break;
    default:
        printf("error : this cycle step isn't defined.\n");
    } 

}