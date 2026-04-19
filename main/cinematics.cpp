#include <time.h> //for debbuging only --> natively present in arduino IDE compilation process

#include "cinematics.h"


// ======================================================
// PARAMETRES BRAS
// ======================================================

// Longueurs des deux bras (mm)
const float lenghtArm1 = 150.0; //1-2
const float lengthArm2= 143.0; //2-3

// Décalages mécaniques du pivot du bras 1 par rapport à l'origine globale (en mm) (cylindrical coordinates)
const float rOffsetPivot1XY = 20.0; //radial offset on the XY plane in degrees
const float zOffsetPivot1 = 75.0; //height offset in mm


//prehension system parameters
// Le point piloté est la pointe de l'outil.
// Le poignet est donc décalé par rapport à cette pointe.
const float toolLength = 57.0;   // mm, horizontal, toujours parallèle au sol
const float toolHeight = -15.0;  // mm, outil 15 mm plus bas que le poignet
//on travaille en cylindrique --> pas d'angle ici


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
    radial pivot. Point 3 is the Wrist end and 4 is the prehension system centre point. 
    A radius is in 3D (spherical), except if specified by the XY mention : in that case, the radius is the one of the cylindrical
    coordinate system (2D projection of the 3D radius on the XY plane).
    */

    //- radius between 0 and 4 on the (x;y) plane
    float radiusZServoToPrehensionCenterXY = sqrt((coordinates->x*coordinates->x)+(coordinates->y*coordinates->y));

    ///- coordinates wrist end position (wrist-prehension system mechanical connection, 3)
    //- radius between 0 and 3 projected on the (x;y) plane
    float radiusZServoToWristEndXY = radiusZServoToPrehensionCenterXY-toolLength;
    //- radius between 0 and 3 in 3D
    float radiusZServoToWristEnd = sqrt((radiusZServoToWristEndXY)*(radiusZServoToWristEndXY)
                                        +((coordinates->z-toolHeight)*(coordinates->z-toolHeight))); 
    float zWristEnd = coordinates->z-toolHeight; //- Height between 0 and 3, toolHeight negative value
    //- no angle, point 3 keeps its distance to 4 constant


    float radiusZservoToPivot1 = sqrt((rOffsetPivot1XY*rOffsetPivot1XY)+(zOffsetPivot1*zOffsetPivot1)); //- radius between 0 and 1
    float zPivot1 = zOffsetPivot1; //pivot joint 1 height

    //- projection on plane (x;y) plane of the distance between 1 and 3
    float distancePivot1WristEndXY = radiusZServoToWristEndXY-rOffsetPivot1XY;
    //- distance between 1 and 3 in 3D --> hypothenus of 13-3Horizontal-Horizontal1 triangle
    float distancePivot1WristEnd = sqrt((distancePivot1WristEndXY*distancePivot1WristEndXY)
        + ((zWristEnd-zOffsetPivot1)*(zWristEnd-zOffsetPivot1)));
    //- use of Al-Kashi's theorem to get Pivot 2 height and radius --> a²=b²+c²-2bccos(alpha), a line2-3 opposed to alpha, b length Arm1, c length line 1-3
    //- angle between line 1-3 and Arm 1  (alpha=arcos((b²+c²-a²) / 2bc))
    float angleLine13ToLine12 = acos(((lenghtArm1*lenghtArm1)
                                    +(distancePivot1WristEnd*distancePivot1WristEnd) - (lengthArm2*lengthArm2))
        / (2*lenghtArm1*distancePivot1WristEnd));
    //- angle between the x axis and the line 1-3
    float angleHorizontalToLine13 = atan2(zWristEnd-zOffsetPivot1, distancePivot1WristEndXY);
    //- angle between the x axis and Arm1 (theta2) --> sum of the angles between the (x;y) plane and the line 1-3, and angle between line 1-3 and Arm 1
    float angleHorizontalToLine12 = angleHorizontalToLine13+angleLine13ToLine12;
    
    //- height of pivot2
    float zPivot2 = sin(angleHorizontalToLine12)*lenghtArm1 + zPivot1;
    //- projection on plane (x;y) to distance between 0 and 2
    float radiusZServoToPivot2XY = rOffsetPivot1XY+ cos(angleHorizontalToLine12)*lenghtArm1;
    //- distance between 0 and 2
    float radiusZservoToPivot2 = sqrt((zPivot2*zPivot2)
        +(radiusZServoToPivot2XY)*radiusZServoToPivot2XY);
    
    //angles in rad 
    //angle between the 2-3 arm and the ground (angle1-2+angle0-1) - theta1 
    float angleArm2ToHorizontalCorrected = atan2((zWristEnd-zPivot2),(radiusZServoToWristEndXY-radiusZServoToPivot2XY)); 
    //printf("theta1 : %f\n", angleArm2ToHorizontalCorrected);
    //angle between the 1-2 arm and the ground - theta2
    float angleHorizontalToArm1Corrected = angleHorizontalToLine12; 
    //printf("theta2 : %f\n", angleHorizontalToArm1Corrected);
    //horizontal angle between 0 and 4 - theta3
    float angleZServoPrehensionCenterCorrected = atan2(-coordinates->y,coordinates->x); 
    //printf("theta3 : %f\n", angleZServoPrehensionCenterCorrected);

    
    servoSet->servoLeft.angleCommand = 180+angleArm2ToHorizontalCorrected*(180.0/M_PI) + servoSet->servoLeft.angleOffset;
    servoSet->servoRight.angleCommand = 180 - angleHorizontalToArm1Corrected*(180.0/M_PI) + servoSet->servoRight.angleOffset;
    servoSet->servoZ.angleCommand = angleZServoPrehensionCenterCorrected*(180.0/M_PI) + servoSet->servoZ.angleOffset; //0° is full left

    //check angles validity
    if(servoSet->servoLeft.angleCommand > 10 && servoSet->servoLeft.angleCommand < 160 && servoSet->servoRight.angleCommand > 0
    && servoSet->servoRight.angleCommand  < 180  && servoSet->servoZ.angleCommand > 0  && servoSet->servoZ.angleCommand < 180) {
        servoSet->reachable=1;
    }
    else {
        servoSet->reachable=0;
        //printf("Selected destination non reachable");
    }
}

/**
 * Function : applyServoCommand
 * ------------
 * Change progessively the current positions of the servos to match their respective commands
 * 
 * servoSet : includes the 3 servos, of which the command angle values will be affected after computation
 * delayStepCloserToCommand : the delay between iterations of current position getting closer to command position, duration of a servo cycle step (ms)
 * 
 * returns : no returns (void).
 */
void applyServoCommand(ServoSet *servoSet, int delayStepCloserToCommand , SpeedProfileType speedProfileType, int depthPercentage, 
    CycleMode cycleMode, int *elapsedTimeSinceServoCycleStart) {

     //PC part - for debugging (delay() from computer)
    
    if (!servoSet->reachable) return;

    int done = 0; //target not reached yet
    printf("servoLeft, servoRight, servoZ\n");

    while (!done) { 
        speedProfileApplication(servoSet, speedProfileType, 100, cycleMode, *elapsedTimeSinceServoCycleStart, delayStepCloserToCommand);
        
        //elapsedTimeSinceServoCycleStart is to be replaced by a timer

        //printf("servoLeft \n = ");
        servoSet->servoLeft.currentAngle = limitStep(servoSet->servoLeft.currentAngle, servoSet->servoLeft.angleCommand, 
        servoSet->servoLeft.maxStep); //limits angle variation accordingly to the set servo parameter
        printf("%f, ", servoSet->servoLeft.currentAngle);
        //printf("servoRight \n = ");
        servoSet->servoRight.currentAngle = limitStep(servoSet->servoRight.currentAngle, servoSet->servoRight.angleCommand, 
        servoSet->servoRight.maxStep);
        printf("%f, ", servoSet->servoRight.currentAngle);
        //printf("servoZ \n = ");
        servoSet->servoZ.currentAngle = limitStep(servoSet->servoZ.currentAngle, servoSet->servoZ.angleCommand, 
        servoSet->servoZ.maxStep);
        printf("%f \n", servoSet->servoZ.currentAngle);
    
        //done if angle difference between command and current position is smaller than 0.05 degrees
        if(abs(servoSet->servoLeft.currentAngle - servoSet->servoLeft.angleCommand) < 0.05 &&
        abs(servoSet->servoRight.currentAngle - servoSet->servoRight.angleCommand) < 0.05 &&
        abs(servoSet->servoZ.currentAngle - servoSet->servoZ.angleCommand) < 0.05) {

            done ++;
        }
        delay(delayStepCloserToCommand);
        *elapsedTimeSinceServoCycleStart +=20;
    }
    
   /*
    
    //arduino part : to uncomment when flashing to the arduino mega

    //printf("servoLeft \n = ");
    servoSet->servoLeft.currentAngle = limitStep(servoSet->servoLeft.currentAngle, servoSet->servoLeft.angleCommand, 
        servoSet->servoLeft.maxStep); //limits angle variation accordingly to the set servo parameter
    //printf("servoRight \n = ");
    servoSet->servoRight.currentAngle = limitStep(servoSet->servoRight.currentAngle, servoSet->servoRight.angleCommand, 
        servoSet->servoRight.maxStep);
    //printf("servoZ \n = ");
    servoSet->servoZ.currentAngle = limitStep(servoSet->servoZ.currentAngle, servoSet->servoZ.angleCommand, 
        servoSet->servoZ.maxStep);
    */
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

    //printf("currentValue : %f\n targetValue : %f\n maxStep : %f\n \n", currentValue, targetValue, maxStep);

    if (delta > maxStep) return currentValue + maxStep;
    if (delta < -maxStep) return currentValue - maxStep;
    return targetValue;
}


//timer only for debbuging --> native in ArduinoIDE
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



void speedProfileApplication(ServoSet* servoSet, enum SpeedProfileType speedProfileType, int depthPercentage, 
    CycleMode cycleMode, int elapsedTimeSinceServoCycleStart, int delayCommandServo) {

    //those variables will be affected a value depending on the maximum angle to perform out of the 3 servos
    float angleToPerformServoLeft = abs(servoSet->servoLeft.currentAngle-servoSet->servoLeft.angleCommand);//here, just the angle servoLeft has to perform to get to its destination
    float angleToPerformServoRight = abs(servoSet->servoRight.currentAngle-servoSet->servoRight.angleCommand);
    float angleToPerformServoZ= abs(servoSet->servoZ.currentAngle-servoSet->servoZ.angleCommand);

    //steps ponderation depending on the biggest one (allows all angles destinations to be reached at the same time)
    //if servoLeft has a bigger angle to perform than the others --> must go the quickest
    if(angleToPerformServoLeft>angleToPerformServoRight && angleToPerformServoLeft>angleToPerformServoZ) {
        //
        //definition max angle step (for the servo cycle step) of servoLeft - can be modified by speed profiles
        servoSet->servoLeft.maxStep=servoSet->servoLeft.servoMaxStep;//servo step = max servo step 
        // ponderation for the other servos
        servoSet->servoRight.maxStep=servoSet->servoRight.servoMaxStep*((float)angleToPerformServoRight/angleToPerformServoLeft);
        servoSet->servoZ.maxStep=servoSet->servoZ.servoMaxStep*((float)angleToPerformServoZ/angleToPerformServoLeft);
    }
    else if(angleToPerformServoRight>angleToPerformServoZ) { //the right servo has the most important distance to travel
        servoSet->servoRight.maxStep=servoSet->servoRight.servoMaxStep;
        servoSet->servoLeft.maxStep=servoSet->servoLeft.servoMaxStep*((float)angleToPerformServoLeft/angleToPerformServoRight);
        servoSet->servoZ.maxStep=servoSet->servoZ.servoMaxStep*((float)angleToPerformServoZ/angleToPerformServoRight);
    }
    else {//the Z servo has the most important distance to travel
        servoSet->servoZ.maxStep=servoSet->servoZ.servoMaxStep;
        servoSet->servoLeft.maxStep=servoSet->servoLeft.servoMaxStep*((float)angleToPerformServoLeft/angleToPerformServoZ);
        servoSet->servoRight.maxStep=servoSet->servoRight.servoMaxStep*((float)angleToPerformServoRight/angleToPerformServoZ);
    }
    

    if(depthPercentage<=0) { //maxStep=0 --> current angles unchanged
        servoSet->servoLeft.maxStep=0.0;
        servoSet->servoRight.maxStep=0.0;
        servoSet->servoZ.maxStep=0.0;
        speedProfileType=CONSTANT; //to not undertake unnecessary processing
    }

    int timeSpeedVariation;
    switch (cycleMode) {
    case PERFORMANCE:
        timeSpeedVariation=200;//duration in which the system will accelerate or decelerate (shorter in PERF. mode = quicker)
        break;
    
    case ECO:
        timeSpeedVariation=500;
        break;
    }

    switch (speedProfileType) {
    case CONSTANT: // unchanged current angles if not even one servo reached its destination, otherwise the other finish their angle
        if(servoSet->servoLeft.maxStep==0 || servoSet->servoRight.maxStep==0 || servoSet->servoZ.maxStep==0) { //if one servo doesn't have to move anymore
            servoSet->servoLeft.maxStep=servoSet->servoLeft.angleCommand-servoSet->servoLeft.currentAngle;//step = difference between command and current angle --> command is reached after this step
            servoSet->servoRight.maxStep=servoSet->servoRight.angleCommand-servoSet->servoRight.currentAngle;
            servoSet->servoLeft.maxStep=servoSet->servoRight.angleCommand-servoSet->servoRight.currentAngle;
        }
        break;
    
    case TRAPESOIDAL_LINEAR: { //we use brackets to create a scope for the int declarations (servoCycleDuration and remainingCycleTime)
        int servoCycleDuration = angleToPerformServoLeft/servoSet->servoLeft.maxStep;
        if(elapsedTimeSinceServoCycleStart<timeSpeedVariation) {
            servoSet->servoLeft.maxStep*=(elapsedTimeSinceServoCycleStart/timeSpeedVariation);
            servoSet->servoRight.currentAngle*=(elapsedTimeSinceServoCycleStart/timeSpeedVariation);
            servoSet->servoZ.currentAngle*=(elapsedTimeSinceServoCycleStart/timeSpeedVariation);
        }
        if(elapsedTimeSinceServoCycleStart<servoCycleDuration*delayCommandServo*0.9){//also works with right or z servos
            int remainingCycleTime = servoCycleDuration-elapsedTimeSinceServoCycleStart;//to have shorter expressions below
            servoSet->servoLeft.currentAngle*=(remainingCycleTime/timeSpeedVariation);
            servoSet->servoRight.currentAngle*=(remainingCycleTime/timeSpeedVariation);
            servoSet->servoZ.currentAngle*=(remainingCycleTime/timeSpeedVariation);
        }
        break;
    }
    case TRAPESOIDAL_EXPONENTIAL: {
        int servoCycleDuration = angleToPerformServoLeft/servoSet->servoLeft.maxStep*delayCommandServo;//number of steps per cycle times duration of one step
        int exponentialCoeff; //coefficient by which we'll multiply servos steps to get exponential accelerations/decelerations

        if(elapsedTimeSinceServoCycleStart<timeSpeedVariation) {
            //exponentialCoeff value
            if(elapsedTimeSinceServoCycleStart==0) {
                exponentialCoeff=0;
            }
            else {
                exponentialCoeff=1-exp(-elapsedTimeSinceServoCycleStart/timeSpeedVariation);// 0 for t=0, 1 for t=timeSpeedVariation
            }

            servoSet->servoLeft.currentAngle*=exponentialCoeff;
            servoSet->servoRight.currentAngle*=exponentialCoeff;
            servoSet->servoZ.currentAngle*=exponentialCoeff;
        }
        if(elapsedTimeSinceServoCycleStart>servoCycleDuration*0.9){//if currentTime is passed  - also works with right or z servos
            int remainingCycleTime = servoCycleDuration-elapsedTimeSinceServoCycleStart;//to have shorter expressions below
            //we use the same formula as before - an alternative would be to use the circle equation : exponentialCoeff=sqrt(1-((cycleDuration-elapsedTime)/timeSpeedVariation)²)
            if(elapsedTimeSinceServoCycleStart==0) {
                exponentialCoeff=0;
            }
            else {
                exponentialCoeff=1-exp(-remainingCycleTime/timeSpeedVariation);// 0 for t=0, 1 for t=timeSpeedVariation
            }
            servoSet->servoLeft.currentAngle*=exponentialCoeff;
            servoSet->servoRight.currentAngle*=exponentialCoeff;
            servoSet->servoZ.currentAngle*=exponentialCoeff;
        }
        break;
    }
    }
}





/**
 * Function : anglesToCoordinates
 * ------------
 * Takes the servo angles as input and computes the corresponding 3D coordinates
 * of the prehension system centre point (point 4).
 *
 * servoSet : includes the 3 servos with their current angleCommand values
 * coordinates : output point with x, y and z coordinate values
 *
 * returns : no returns (void).
 */
void anglesToCoordinates(ServoSet* servoSet, Coordinates* coordinates) {
    /*
    We reverse the inverse kinematics chain:
    - Recover the three angles (theta1, theta2, theta3) from servo commands
    - Reconstruct pivot positions forward from the base (point 0)
    - Derive the wrist end (point 3), then add back tool offsets to get point 4
    */

    // --- Recover angles from servo commands (undo the offset and mapping) ---
    // theta1 : angle between Arm2 (2-3) and horizontal
    float angleArm2ToHorizontal = (servoSet->servoLeft.currentAngle - servoSet->servoLeft.angleOffset - 180) * (M_PI / 180.0);
    // theta2 : angle between Arm1 (1-2) and horizontal
    float angleHorizontalToArm1 = (180 - servoSet->servoRight.currentAngle + servoSet->servoRight.angleOffset) * (M_PI / 180.0);
    // theta3 : horizontal rotation angle around Z servo axis
    float angleZServoPrehensionCenter = (servoSet->servoZ.currentAngle - servoSet->servoZ.angleOffset) * (M_PI / 180.0);

    // --- Forward chain: reconstruct pivot 1 position ---
    float zPivot1 = zOffsetPivot1;
    float rPivot1XY = rOffsetPivot1XY;

    // --- Forward chain: reconstruct pivot 2 from pivot 1 and Arm1 ---
    float radiusZServoToPivot2XY = rPivot1XY + cos(angleHorizontalToArm1) * lenghtArm1;
    float zPivot2 = zPivot1 + sin(angleHorizontalToArm1) * lenghtArm1;

    // --- Forward chain: reconstruct wrist end (point 3) from pivot 2 and Arm2 ---
    float radiusZServoToWristEndXY = radiusZServoToPivot2XY + cos(angleArm2ToHorizontal) * lengthArm2;
    float zWristEnd = zPivot2 + sin(angleArm2ToHorizontal) * lengthArm2;

    // --- Recover point 4 (prehension centre) from wrist end (point 3) ---
    // Add back tool offsets (inverse of: zWristEnd = z - toolHeight, rXY = rXY_4 - toolLength)
    float radiusZServoToPrehensionCenterXY = radiusZServoToWristEndXY + toolLength;
    float zPrehensionCenter = zWristEnd + toolHeight;

    // --- Project back to Cartesian coordinates using theta3 ---
    // theta3 = atan2(-y, x)  =>  x = r*cos(theta3), y = -r*sin(theta3)
    coordinates->x = radiusZServoToPrehensionCenterXY * cos(angleZServoPrehensionCenter);
    coordinates->y = -radiusZServoToPrehensionCenterXY * sin(angleZServoPrehensionCenter);
    coordinates->z = zPrehensionCenter;
}


/**
 * Function : initServoSet
 * ------------
 * Initialises the values of each servo inside the servo set
 *
 * servoSet : includes the 3 servos with their current angleCommand values
 *
 * returns : no returns (void), but offsets and maxSteps are initialised.
 */
void initServoSet(ServoSet* servoSet, int delayStepCloserToCommand) {
    //offset obtainted by linear regression --> see spreadsheet results
    servoSet->servoLeft.angleOffset=-8.0;
    servoSet->servoRight.angleOffset=-7.0;
    servoSet->servoZ.angleOffset=2.5;

    //servo time to 180° : 510 ms (0.51 seconds in 4.8V, see servo datasheet)
    servoSet->servoLeft.servoMaxStep = (delayStepCloserToCommand*180 / 510); 
    servoSet->servoRight.servoMaxStep = (delayStepCloserToCommand*180 / 510);
    servoSet->servoZ.servoMaxStep = (delayStepCloserToCommand*180 / 510);
}

