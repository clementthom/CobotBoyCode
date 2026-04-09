#include <stdint.h>
#include <stdio.h>

#include "../main/trajectories.h"
#include "../main/trajectories.cpp"

void testInit();
void testCheckIfInZone();
void testCheckCloseToObstacle();


int main() {
    //testInit();
    testCheckIfInZone();
}

void testInit() {
    initCoordinates();;
    initZone();
}

void testCheckIfInZone() {
    Coordinates point;
    initCoordinates();
    initZone();
    point.x=100.0;
    point.y=0.0;
    point.z=0.0;

    printf("coordonnees zone : \n origin x : %f\n origin y : %f\n origin z : %f\n", workingZone.origin.x, workingZone.origin.y,
    workingZone.origin.z);
    printf("pointLoin x : %f\n pointLoin y : %f\n pointLoin z : %f\n\n", workingZone.farthestPointFromOrigin.x, 
        workingZone.farthestPointFromOrigin.y, workingZone.farthestPointFromOrigin.z);
    printf("dedans : %d", checkIfInZone(point, workingZone));
}

void testCheckCloseToObstacle() {

}