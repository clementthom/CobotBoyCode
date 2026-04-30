#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include "common.h" //for cartesian coordinates structs and cycle steps
extern Coordinates initPosition;
extern Coordinates convoyeurEntree;

/////////Enums

//Current intermediate point position in a trajectory cycle
typedef enum {
    DEPART_POINT,
    INTERM_POINT_1,
    INTERM_POINT_2,
    DESTINATION_POINT
}IntermediatePoint;

//Cycle the robot will execute
typedef enum {
    CYCLE1,
    CYCLE2,
    CUSTOM
}SelectedCycle;


/////////Prototypes
void initCoordinates(SelectedCycle* selectedCycle);
void initZone();
int checkIfInZone(Coordinates coordinates, Zone zone);
PrehensionStatus stepActions(CycleStep stepToken, Coordinates* coordinates);
void cycleExecution(Coordinates* destinationCoordinates, PrehensionStatus* actionPrehension,
                    int* nextStepIndex, SelectedCycle* selectedCycle);
Coordinates trajectoryProfile(Coordinates currentCoordinates, Coordinates destination, 
                                int selectedMaxAlt, CycleMode cycleMode, IntermediatePoint* intermediatePoint);

#endif