#include "Interactions.h"
#include "Constants.h"

#include <frc/GenericHID.h>
#include <frc/XboxController.h>

/*
This is the mapping of all buttons on the controller:

    -Left Bumper
        - Unassigned
    -Right Bumper
        - Unassigned
    -Left Joystick
        - The Y axis of the computer for driving
    -Right Joystick
        - The X axis of the computer for driving/turning
    -Right Trigger
        - Changing the value of the Falcon 500
    -Left Trigger
        - Unassigned
    -X Button
        - Enters closed loop target positioning
    -A Button
        - toggleCameraMode
    -B Button 
        - High/low gear shifting
    -Y Button
        - 
    -Left Joystick Button
        - Unassigned
    -Right Joystick Button
        - Unassigned
    -Start Button
        - Unassigned
    -Back Button
        -unassigned

*/


Interactions::Interactions( frc::XboxController *icontroller1, frc::XboxController *icontroller2  ) {
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

bool Interactions::toggleLimeLightCamera() {
    return controller1->GetAButtonPressed();
}   