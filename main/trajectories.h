#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include "common.h" //for cartesian coordinates structs and cycle steps


/////////Prototypes
void initCoordinates();
void initZone();
int checkIfInZone(Coordinates coordinates, Zone zone);
PrehensionStatus stepActions(CycleStep stepToken, Coordinates* coordinates);
void cycleExecution(Coordinates* destinationCoordinates, PrehensionStatus* actionPrehension, int* nextStepIndex);

#endif