#include "Interactions.h"
#include "Constants.h"
#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include "Launcher/VelocityTracking.h"

/*
This is the mapping of all buttons on the controller:

    -Left Bumper
        - Elevator down along with trigger
    -Right Bumper
        - Servos on the elevator
    -Left Joystick
        - The Y axis of the computer for driving
    -Right Joystick
        - The X axis of the computer for driving/turning
    -Right Trigger
        - Making the elevator go up and down
    -Left Trigger
        - Unassigned
    -X Button
        - Enters closed loop target velocity
    -A Button
        - toggleCameraMode
    -B Button 
        - Running the intake
    -Y Button
        - visionControl
    -Left Joystick Button
        - Unassigned
    -Right Joystick Button
        - Unassigned
    -Start Button
        - deploy the intake
    -Back Button
        -unassigned

*/

Interactions::Interactions( frc::XboxController *icontroller1, frc::XboxController *icontroller2  ) {
    controller1 = icontroller1;
    controller2 = icontroller2;
}

double Interactions::getTurn() {
    return controller1->GetX(frc::GenericHID::JoystickHand::kRightHand);
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

double Interactions::elevatorControl() {
    return controller1->GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand);
}

bool Interactions::toggleLimeLightCamera() {
    return controller1->GetAButtonPressed();
}   

bool Interactions::deployPneumatic_Intake() {
    return controller1->GetStartButtonPressed();
}

bool Interactions::visionControl() {
    return controller1->GetYButtonPressed();
}

bool Interactions::driveElevatorDown() {
    return controller1->GetBumper(frc::GenericHID::JoystickHand::kLeftHand);
}

bool Interactions::runIntake() {
    return controller1->GetBButton();
}

bool Interactions::turnEleServo() {
    return controller1->GetBumper(frc::GenericHID::JoystickHand::kRightHand);
}

bool Interactions::conveyorHardStop() {
    return controller1->GetStartButton();
}
