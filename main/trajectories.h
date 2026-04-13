#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include "common.h" //for cartesian coordinates structs and cycle steps



/////////Global variables
Coordinates convoyeurEntree;
Coordinates convoyeurSortie;
Coordinates machineA;
Coordinates machineB;
Coordinates initPosition;

Zone obstacle1;
Zone workingZone;


/////////Prototypes
void initCoordinates();
void initZone();
int checkIfInZone(Coordinates coordinates, Zone zone);
PrehensionStatus stepActions(CycleStep stepToken, Coordinates* coordinates);
void cycleExecution(Coordinates* destinationCoordinates, PrehensionStatus* actionPrehension, int* nextStepIndex);

#endif