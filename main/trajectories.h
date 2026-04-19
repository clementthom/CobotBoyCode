#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include "common.h" //for cartesian coordinates structs and cycle steps

/////////Enums

//Current intermediate point position in a trajectory cycle
typedef enum {
    DEPART_POINT,
    INTERM_POINT_1,
    INTERM_POINT_2,
    DESTINATION_POINT
}IntermediatePoint;


/////////Prototypes
void initCoordinates();
void initZone();
int checkIfInZone(Coordinates coordinates, Zone zone);
PrehensionStatus stepActions(CycleStep stepToken, Coordinates* coordinates);
void cycleExecution(Coordinates* destinationCoordinates, PrehensionStatus* actionPrehension, int* nextStepIndex);
Coordinates trajectoryProfile(Coordinates currentCoordinates, Coordinates destination, 
                                int selectedMaxAlt, CycleMode cycleMode, IntermediatePoint* intermediatePoint);

#endif