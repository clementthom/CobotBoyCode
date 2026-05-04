#ifndef PREHENSION_H
#define PREHENSION_H

#include <AutoPID.h>
#include "common.h"

// =====================================================
// PID TUNING CONSTANTS
// =====================================================
#define KP_GRIP 90.0
#define KI_GRIP 0.0
#define KD_GRIP 2.5

// =====================================================
// EXTERN GLOBALS — defined in prehension.cpp
// =====================================================
extern double consigneOuverture;
extern double consigneFermeture;
extern double seuilOuvertureMax;
extern double seuilFermetureMax;

extern GripperState gripperState;
extern double       gripperPWM_PID;
extern double       measuredTension;
extern double       gripperSetpoint;
extern double       filteredTension;
extern double       previousGripPWM;
extern bool         gripperDone;
extern AutoPID      pidGripper;
extern bool         objectDetected;
extern bool         openLimitDetected;

// =====================================================
// FUNCTION PROTOTYPES
// =====================================================
double readVoltage(int pin);
void   applyGripperOpening(double pwm);
void   applyGripperClosing(double pwm);
void   stopGripper();
void   openGripper();
void   closeGripper();
void   updateGripperPID();

#endif // PREHENSION_H
