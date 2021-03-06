#pragma once
#include "Interactions.h"
#include "Constants.h"
#include <frc/GenericHID.h>
#include <frc/XboxController.h>

class Interactions {
  private:
     frc::XboxController *controller1;
     frc::XboxController *controller2;
 public:
    Interactions( frc::XboxController *icontroller1, 
            frc::XboxController *icontroller2 );
    double getTurn();
    double getDrive();
    bool getShiftGear();
    bool enterPIDFXClosedLoop();
    double elevatorControl();
    bool toggleLimeLightCamera();
    bool deployPneumatic_Intake();
    bool visionControl();
    bool driveElevatorDown();
    bool runIntake();
    bool turnEleServo();
    bool conveyorHardStop();
};
