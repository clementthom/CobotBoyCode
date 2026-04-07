#ifndef COMMON_H
#define COMMON_H

#include <stdlib.h>//to get the abs() function
#include <math.h>
#include <stdio.h>


////Enumerations

//type of the zone, either an OBSTACLE or the WORKING ZONE
typedef enum {
    OBSTACLE,
    WORKING_ZONE
}ZoneType;


////Structures

//Tri-dimensional coordinates : x, y and z
typedef struct {
  float x;
  float y;
  float z;
}Coordinates;

//2 tri-dimensional points make a rectangular volume perpendicular to the cartesian coordinate system
typedef struct {
    Coordinates origin; 
    Coordinates farthestPointFromOrigin; //point making a diagonal (going through the whole volume) with the origin
    ZoneType zoneType;
}Zone; //fictional volume taking the shape of a rectangular bloc

//distance to an object boundaries for all axis and the object(zone) type (obstacle or working zone)
typedef struct {
    Coordinates distances;
    ZoneType zoneType;
}ZoneDistances;

#endif