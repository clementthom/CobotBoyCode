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

//Cycle steps, including destination and prehension commands
typedef enum {
    CONVOYEUR_ENTREE,
    CONVOYEUR_SORTIE,
    MACHINE_A,
    MACHINE_B,
    INIT,
    GRAB_OBJECT,
    RELEASE_OBJECT
}CycleStep;


//- lists the different actions possible for the prehension subsystem
typedef enum {
    GRAB,
    RELEASE,
    KEEP_CURRENT_PREHENSION_POSITION,
    UNKNOWN_STATUS //when no valid instruction is given
}PrehensionStatus;


//Type of trajectory profile the robot will undertake
typedef enum {
    ECO,
    PERFORMANCE
}CycleMode;


//Names of objects
typedef enum {
    GOMME,
    GOBELET,
    CYLINDRE
}ObjectName;

//robot currently in use
typedef enum {
    ROBOT1,
    ROBOT2
}RobotName;

//selected prehension system
typedef enum {
    LASSO,
    EYE
}PrehensionSystemType;

/////////////////////Structures

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

//parameters of object to grab
typedef struct {
    int radius;
    int prehensionHeight;
    ObjectName objectName;
    double consigne;
}Object;


//lists of all objects
typedef struct {
    Object gomme;
    Object gobelet;
    Object cyclindre;
}ObjectList;

//Robots parameters
typedef struct {
    RobotName robotName;
    PrehensionSystemType prehensionSystemType;
}Robot;

#endif