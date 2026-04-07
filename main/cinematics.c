#include <time.h> //for debbuging only --> natively present in arduino IDE compilation process

#include "cinematics.h"


// ======================================================
// PARAMETRES BRAS
// ======================================================

// Longueurs des deux bras (mm)
const int lenghtArm1 = 150; //1-2
const int lengthArm2= 143; //2-3

// Décalages mécaniques du pivot du bras 1 par rapport à l'origine globale (en mm)
const int xOffsetPivot1 = 20;
const int zOffsetPivot1 = 75;

// Offsets servo (calibration, en degrés)
const int degOffsetLeft = -20;
const int degOffsetRight = -10;
const int degOffsetZ = -3;

// ======================================================
// PARAMETRES PREHENSEUR
// ======================================================

// Le point piloté est la pointe de l'outil.
// Le poignet est donc décalé par rapport à cette pointe.
const int toolLength = 57;   // mm, horizontal, toujours parallèle au sol
const int toolHeight = -15;  // mm, outil 10 mm plus bas que le poignet
//on travaille en cylindrique --> pas d'angle ici

// ======================================================
// PARAMETRES TRAJECTOIRE / COMMANDE
// ======================================================

// Limitation de variation angulaire par cycle
const float maxStepLeft = 1.2;
const float maxStepRight = 1.2;
const float maxStepZ = 1.5;

// Position courante estimée de la pointe outil
Coordinates currentPosition;


/**
 * Function : coordinatesChange
 * -------------
 * affects given coordinates to a Coordinates struct instance
 * 
 * coordinates : the instance in which new parameters are to be printed into
 * x : new given x parameter
 * y : new given y parameter
 * z : new given z parameter
 * 
 * returns : no returns (void).
 */
void coordinatesChange(Coordinates* coordinates, float xPosition, float yPosition, float zPosition) {
  coordinates->x=xPosition;
  coordinates->y=yPosition;
  coordinates->z=zPosition;
}

/**
 * Function : coordinatesToAngles
 * ------------
 * Takes a tridimensional point as input and write the corresponding angle commands into the servo set
 * 
 * coordinates : input point with x, y and z coordinate values
 * servoSet : includes the 3 servos, of which the command angle values will be affected after computation
 * 
 * returns : no returns (void).
 */
void coordinatesToAngles(Coordinates* coordinates, ServoSet* servoSet) {
    /*we will represent the system into a simplified version : 1 vertical pivot joint (point 0, system origin); 2 radial pivot 
    joints (point 1 and 2); 2 arms (from 1 to 2 and 2 to 3 respectively) and 1 offset from the vertical to the first 
    radial pivot. Point 3 is the Wrist end and 4 is the prehension system centre point*/

    
    int radiusZServoToPrehensionCenter = sqrt(pow(coordinates->x,2.0)+pow(coordinates->y,2.0)); //radius between 0 and 4, pow 2 --> square

    ///coordinates wrist end position (wrist-prehension system mechanical connection, 3)
    int radiusZServoToWristEnd = radiusZServoToPrehensionCenter-toolLength; //radius between 0 and 3
    int zWristEnd = coordinates->z-toolHeight; //height between 0 and 3
    //no angle, point 3 keeps its distance to 4 constant


    int radiusZservoToPivot1 = sqrt(pow(xOffsetPivot1,2.0)+pow(zOffsetPivot1,2.0)); //radius between 0 and 1
    int zPivot1 = zOffsetPivot1; //pivot joint 1 height


    int distancePivot1WristEnd = sqrt(pow(coordinates->x - xOffsetPivot1- toolLength, 2.0)
        + pow(coordinates->z-zOffsetPivot1-toolHeight, 2.0));
    //use of Al-Kashi's theorem to get Pivot 2 height and radius --> a²=b²+c²-2bccos(alpha), a opposed to alpha
    float angleLine13ToLine12 = acos((pow(distancePivot1WristEnd,2.0)+pow(lenghtArm1,2.0)-pow(lengthArm2, 2.0))
        / (2*lenghtArm1*lengthArm2));
    float angleHorizontalToLine13 = atan2(coordinates->x-xOffsetPivot1- toolLength, coordinates->z-zOffsetPivot1- toolHeight);
    float angleHorizontalToLine12 = angleHorizontalToLine13+angleLine13ToLine12;
    
    int zPivot2 = sin(angleHorizontalToLine12)*lenghtArm1 + zPivot1;
    int radiusZservoToPivot2 = sqrt(pow(cos(angleHorizontalToLine12)*lenghtArm1 + xOffsetPivot1, 2.0)
                                +pow(sin(angleHorizontalToLine12)*lenghtArm1 + zOffsetPivot1, 2.0)) ;

    
    //angles in rad 
    //angle between the 2-3 arm and the ground (angle1-2+angle0-1) - theta1 
    float angleArm2ToHorizontalCorrected = -asin((float)(zPivot2-zWristEnd)/(float)lengthArm2); 
    //angle between the 1-2 arm and the ground - theta2
    float angleHorizontalToArm1Corrected = angleHorizontalToLine12; 
    //horizontal angle between 0 and 4 - theta3
    float angleZServoPrehensionCenterCorrected = atan(coordinates->x/coordinates->y); 

    
    servoSet->servoLeft.angleCommand = angleArm2ToHorizontalCorrected*(180.0/M_PI)+180.0 + degOffsetLeft;
    servoSet->servoRight.angleCommand = 180.0 - (angleHorizontalToArm1Corrected+angleArm2ToHorizontalCorrected)*(180.0/M_PI) + degOffsetRight;
    servoSet->servoZ.angleCommand = angleZServoPrehensionCenterCorrected*(180.0/M_PI) + degOffsetZ;

    //check angles validity
    if(servoSet->servoLeft.angleCommand > 10 && servoSet->servoLeft.angleCommand < 160 && servoSet->servoRight.angleCommand > 0
    || servoSet->servoRight.angleCommand  < 180  & servoSet->servoZ.angleCommand > 0  && servoSet->servoZ.angleCommand < 180) {
        servoSet->reachable=1;
    }
    else {
        servoSet->reachable=0;
        printf("Selected destination non reachable");
    }
}

/**
 * Function : applyServoCommand
 * ------------
 * Change progessively the current positions of the servos to match their respective commands
 * 
 * servoSet : includes the 3 servos, of which the command angle values will be affected after computation
 * delayStepCloserToCommand : the delay between iterations of current position getting closer to command position (ms)
 * 
 * returns : no returns (void).
 */
void applyServoCommand(ServoSet *servoSet, int delayStepCloserToCommand) {
  /*  
  if (!servoSet->reachable) return;

  int done = 0; //target not reached yet

  while (!done) { */
    printf("servoLeft \n = ");
    servoSet->servoLeft.currentAngle = limitStep(servoSet->servoLeft.currentAngle, servoSet->servoLeft.angleCommand, 
        servoSet->servoLeft.maxStep); //limits angle variation accordingly to the set servo parameter
    printf("servoRight \n = ");
    servoSet->servoRight.currentAngle = limitStep(servoSet->servoRight.currentAngle, servoSet->servoRight.angleCommand, 
        servoSet->servoRight.maxStep);
    printf("servoZ \n = ");
    servoSet->servoZ.currentAngle = limitStep(servoSet->servoZ.currentAngle, servoSet->servoZ.angleCommand, 
        servoSet->servoZ.maxStep);

    //writeServosMicroseconds(currentCmdA, currentCmdB, currentCmdZ);
    /*
    //done if angle difference between command and current position is smaller than 0.05 degrees
    if(abs(servoSet->servoLeft.currentAngle - servoSet->servoLeft.angleCommand) < 0.05 &&
      abs(servoSet->servoRight.currentAngle - servoSet->servoRight.angleCommand) < 0.05 &&
      abs(servoSet->servoZ.currentAngle - servoSet->servoZ.angleCommand) < 0.05) {

        done ++;
    }
    */

    //delay(delayStepCloserToCommand);
  //}
}

/**
 * Function : limitStep
 * ------------
 * returns the sum of the current angle plus or minus the step depending on the target angle value
 * 
 * currentValue : angle of the servo before processing
 * targetValue : the aimed final angle value for the servo
 * maxStep : the maximum angle variation applied to the servo for one servo rotation cycle
 * 
 * returns : the new angle value, closer to the target angle than the input
 */
float limitStep(float currentValue, float targetValue, float maxStep) {
  float delta = targetValue - currentValue;

  printf("currentValue : %f\n targetValue : %f\n maxStep : %f\n \n", currentValue, targetValue, maxStep);

  if (delta > maxStep) return currentValue + maxStep;
  if (delta < -maxStep) return currentValue - maxStep;
  return targetValue;
}

/*
//only for debbuging --> native in ArduinoIDE
void delay(int number_of_seconds)
{
	// Converting time into milli_seconds
	int milli_seconds = 1000 * number_of_seconds;

	// Storing start time
	clock_t start_time = clock();

	// looping till required time is not achieved
	while (clock() < start_time + milli_seconds)
		;
}

*/

/*
int speedProfileApplication(enum SpeedProfileType speedProfileType, int depthPercentage) {

    if(depthPercentage<=0) {
        return 0;
    }
    if(depthPercentage>=100) {
        return 100;
    }

    switch (speedProfileType)
    {
    case CONSTANT:
        return 100;
        break;
    
    case TRAPESOIDAL_LINEAR:
        
        
        break;
    case TRAPESOIDAL_EXPONENTIAL:

        break;
    }
    return 0;
}

*/