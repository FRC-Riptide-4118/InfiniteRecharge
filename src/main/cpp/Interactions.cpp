#include "Interactions.h"
#include "Constants.h"

#include <frc/GenericHID.h>
#include <frc/XboxController.h>


Interactions::Interactions( frc::XboxController *icontroller1,
        frc::XboxController *icontroller2  ) {
    controller1 = icontroller1;
    controller2 = icontroller2;
}


double Interactions::getTurn() {
    return controller1->GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand);
}

double Interactions::getDrive() {
    return controller1->GetY(frc::GenericHID::JoystickHand::kLeftHand);
}

bool Interactions::getShiftGear() {
    return controller1->GetBButtonPressed();
}

bool Interactions::enterPIDFXClosedLoop() {
    return controller1->GetXButtonPressed();
}

double Interactions::shooterRawSpeed() {
    return controller1->GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand);
}