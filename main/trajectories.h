#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include "common.h" //for cartesian coordinates structs


/////////Prototypes
void initCoordinates();
void initZone();
int checkIfInZone(Coordinates coordinates, Zone zone);

#endif