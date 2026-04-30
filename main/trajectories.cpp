#include "trajectories.h"

/////////Global variables
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
void initCoordinates(SelectedCycle* selectedCycle) {

    if(*selectedCycle==CYCLE1) {
        convoyeurEntree.x=155.0;
        convoyeurEntree.y=-20.0;
        convoyeurEntree.z=50.0;

        convoyeurSortie.x=-155.0;
        convoyeurSortie.y=-20.0;
        convoyeurSortie.z=50.0;

        machineA.x=-150.0;
        machineA.y=-150.0;
        machineA.z=50.0;

        machineB.x=-160.0;
        machineB.y=-150.0;
        machineB.z=0.0;

        initPosition.x=0.0;
        initPosition.y=-130.0;
        initPosition.z=120.0;
    }

    if(*selectedCycle==CYCLE2) {
        convoyeurEntree.x=-155.0;
        convoyeurEntree.y=-25.0;
        convoyeurEntree.z=50.0;

        convoyeurSortie.x=155.0;
        convoyeurSortie.y=-20.0;
        convoyeurSortie.z=50.0;

        machineA.x=-150.0;
        machineA.y=-150.0;
        machineA.z=50.0;

        machineB.x=155.0;
        machineB.y=-97.0;
        machineB.z=50.0;

        initPosition.x=0.0;
        initPosition.y=-130.0;
        initPosition.z=120.0;
    }
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
    if((fabs(coordinates.x) < fabs(zone.origin.x)-5.0 || fabs(coordinates.x)< fabs(zone.farthestPointFromOrigin.x)-5.0) &&
    (fabs(coordinates.y) < fabs(zone.origin.y)-5.0 || fabs(coordinates.y)<fabs(zone.farthestPointFromOrigin.y))-5.0 &&
    (fabs(coordinates.z) < fabs(zone.origin.z)-5.0 || fabs(coordinates.z)<fabs(zone.farthestPointFromOrigin.z)-5.0)) {
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
    if(fabs(difference) < fabs(zoneDistance.distances.x)) {
        zoneDistance.distances.x = difference;
    }
    difference = coordinates.x - zone.farthestPointFromOrigin.x;
    if(fabs(difference) < fabs(zoneDistance.distances.x)) {
        zoneDistance.distances.x = difference;
    }
    difference = coordinates.y - zone.origin.y;
    if(fabs(difference) < fabs(zoneDistance.distances.y)) {
        zoneDistance.distances.y = difference;
    }
    difference = coordinates.y - zone.farthestPointFromOrigin.y;
    if(fabs(difference) < fabs(zoneDistance.distances.y)) {
        zoneDistance.distances.y = difference;
    }
    difference = coordinates.z - zone.origin.z;
    if(fabs(difference) < fabs(zoneDistance.distances.z)) {
        zoneDistance.distances.z = difference;
    }
    difference = fabs(coordinates.z)<fabs(zone.farthestPointFromOrigin.z);
    if(fabs(difference) < fabs(zoneDistance.distances.z)) {
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
        return GRAB;;

    case GRAB_OBJECT:
        return GRAB;

    case RELEASE_OBJECT:
        return RELEASE;
    }
    return UNKNOWN_STATUS; //To return an error so it can be processed
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
 * -- selectedCycle : the cycle to perform
 * 
 * 
 * returns : nothing, changes the destination coordinates, defines the action to transmit to the prehension system and increments/reset to 0 the step sequence index (pointers)
 */
void cycleExecution(Coordinates* destinationCoordinates, PrehensionStatus* actionPrehension,
                    int* nextStepIndex, SelectedCycle* selectedCycle) {

    switch (*selectedCycle) {
    case CYCLE1:
        switch (*nextStepIndex) {
        case 0: //step 0 : initialisation
            *actionPrehension = stepActions(INIT, destinationCoordinates);
            *nextStepIndex +=1; //for some reason it doesn't work with ++ (verified with debugger)
            break;
        case 1: 
            *actionPrehension = stepActions(CONVOYEUR_ENTREE, destinationCoordinates);
            *nextStepIndex +=1;
            break;
        case 2:
            *actionPrehension = stepActions(GRAB_OBJECT, destinationCoordinates);
            *nextStepIndex +=1;
            break;
        case 3: 
            *actionPrehension = stepActions(MACHINE_A, destinationCoordinates);
            *nextStepIndex +=1;
            break;
        case 4: 
            *actionPrehension = stepActions(RELEASE_OBJECT, destinationCoordinates);
            *nextStepIndex +=1;
            break;
        default:
            //printf("error : this cycle step isn't defined.\n");
            *actionPrehension = stepActions(INIT, destinationCoordinates);
            *nextStepIndex =0;
        }
        break;

    case CYCLE2:
        switch (*nextStepIndex) {
        case 0: //step 0 : initialisation
            *actionPrehension = stepActions(INIT, destinationCoordinates);
            *nextStepIndex +=1; //for some reason it doesn't work with ++ (verified with debugger)
            break;
        case 1: 
            *actionPrehension = stepActions(CONVOYEUR_ENTREE, destinationCoordinates);
            *nextStepIndex +=1;
            break;
        case 2:
            *actionPrehension = stepActions(GRAB_OBJECT, destinationCoordinates);
            *nextStepIndex +=1;
            break;
        case 3: 
            *actionPrehension = stepActions(MACHINE_A, destinationCoordinates);
            *nextStepIndex +=1;
            break;
        case 4: 
            *actionPrehension = stepActions(RELEASE_OBJECT, destinationCoordinates);
            *nextStepIndex +=1;
            break;
        case 5:
            *actionPrehension = stepActions(GRAB_OBJECT, destinationCoordinates);
            *nextStepIndex +=1;
            break;
        case 6: 
            *actionPrehension = stepActions(MACHINE_B, destinationCoordinates);
            *nextStepIndex +=1;
            break;
        case 7: 
            *actionPrehension = stepActions(RELEASE_OBJECT, destinationCoordinates);
            *nextStepIndex +=1;
            break;
        case 8: 
            *actionPrehension = stepActions(GRAB_OBJECT, destinationCoordinates);
            *nextStepIndex +=1;
            break;
        case 9:
            *actionPrehension = stepActions(CONVOYEUR_SORTIE, destinationCoordinates);
            *nextStepIndex +=1;
            break;
        case 10: 
            *actionPrehension = stepActions(RELEASE_OBJECT, destinationCoordinates);
            *nextStepIndex +=1;
            break;
        case 11: //step 0 : initialisation
            *actionPrehension = stepActions(INIT, destinationCoordinates);
            *nextStepIndex =0; 
            break;
        default:
            //printf("error : this cycle step isn't defined.\n");
            *actionPrehension = stepActions(INIT, destinationCoordinates);
            *nextStepIndex =0;
        }
        break;

    default:
        break;
    }
    
}


/**
 * Function : trajectoryProfile
 * --------------------
 * - modifies the trajectory from one point to another by creating intermediate points by which the prehension center
 * will pass by
 * 
 * - Parameters :
 * -- currentCoordinates : pointer of the coordinates of the prehension system at the time the function is called
 * -- destination : pointer of the coordinates of the final destination of the cycle step
 * -- selectedMaxAlt : maximum altitude, defined by the user, of the trajectory
 * -- cycleMode : defines if the trajectory mode is ECO or PERFORMANCE (ie PERF.)
 * -- intermediatePoint : pointer of the coordinates of the current intermediate destination (is modified after the function run)
 * 
 * 
 * returns : command coordinates, that will be affected to the intermediate point coordinates outside of the function scope
 */
Coordinates trajectoryProfile(Coordinates currentCoordinates, Coordinates destination, 
                                int selectedMaxAlt, CycleMode cycleMode, IntermediatePoint* intermediatePoint) {
    int trajMaxAltitude;//maximum altitude the prehension system will reach during cycle
    Coordinates intermediateDestination=currentCoordinates;//coordinates of a point withing the trajectory profile 

    //if current position is cycle step destination then the trajectory cycle is finished
    if(fabs(currentCoordinates.x-destination.x)<1.0 && fabs(currentCoordinates.y-destination.y)<1.0 
            && fabs(currentCoordinates.z-destination.z)<1.0) {
            *intermediatePoint=DESTINATION_POINT;
    }

    //max altitude defined, by default user defined, except if out of bounds or inferior to destination alt.
    trajMaxAltitude=selectedMaxAlt;
    if (selectedMaxAlt<=destination.z) {
        trajMaxAltitude=destination.z*2;
    }
    if(trajMaxAltitude>=workingZone.farthestPointFromOrigin.z) {
        trajMaxAltitude=workingZone.farthestPointFromOrigin.z;
    }

    switch (*intermediatePoint) {
    case DEPART_POINT:
        switch (cycleMode) {
        case PERFORMANCE:
            intermediateDestination.z=trajMaxAltitude;
            break;
        case ECO:
            intermediateDestination.z=trajMaxAltitude*0.8; //max alt. not reached because of cut corners
            break;
        }
        *intermediatePoint=INTERM_POINT_1;
        break;

    case INTERM_POINT_1:
        switch (cycleMode) {
        case PERFORMANCE:
            intermediateDestination=destination;
            intermediateDestination.z=trajMaxAltitude;
            *intermediatePoint=INTERM_POINT_2;
            break;
        case ECO: // we cut the first trajectory triangle (see drawing 2 : trajectoryProfile.svg)
            if(fabs(trajMaxAltitude-currentCoordinates.z)>1.0) {
                intermediateDestination.x=currentCoordinates.x+(destination.x-currentCoordinates.x)*0.2;
                intermediateDestination.y=currentCoordinates.y+(destination.y-currentCoordinates.y)*0.2;
                intermediateDestination.z=trajMaxAltitude;
            }
            else {
                intermediateDestination.x=currentCoordinates.x+(destination.x-currentCoordinates.x)*0.75;//already 20% of initial x and y have been travelled -> 80% left to do --> to do 60% of travel : 0.8*0.75=0.6
                intermediateDestination.y=currentCoordinates.y+(destination.y-currentCoordinates.y)*0.75;
                *intermediatePoint=INTERM_POINT_2; //not before, otherwise the corner won't be cut off
            }
            break;
        }
        break;

    case INTERM_POINT_2:
    switch (cycleMode) {
        case PERFORMANCE:
            intermediateDestination=destination;//no corners to cut : next stop is step destination
            *intermediatePoint=DESTINATION_POINT;
            break;
        case ECO:
            if(fabs(trajMaxAltitude-currentCoordinates.z)<1.0) {
                intermediateDestination.x=destination.x;
                intermediateDestination.y=destination.y;//we are above the destination point
                intermediateDestination.z=trajMaxAltitude*0.8;
            }
            else {
                intermediateDestination=destination;
                *intermediatePoint=DESTINATION_POINT; //can only proceed if the second corner is cut
            }
            break;
        }
        
        break;
    default :
        *intermediatePoint=DESTINATION_POINT; //cycle destination reached
        break;
    }

    return intermediateDestination;
}