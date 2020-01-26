/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include "Robot.h"
#include "Subsystems/Drivetrain/Pneumatics.h"
#include "Subsystems/Drivetrain/Motors.h"
#include "Constants.h"

#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include "ctre/Phoenix.h"
#include <frc/DigitalInput.h>
#include <frc/Servo.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"


frc::SpeedControllerGroup left(srxFL, srxML, srxBL);
frc::SpeedControllerGroup right(srxFR, srxMR, srxBR);

frc::DifferentialDrive drive(left, right);

frc::XboxController * Controller1 = new frc::XboxController(0);


//Victor Sp setup for R2-Dan2
WPI_VictorSPX spxFL = {0};
WPI_VictorSPX spxML = {1};
WPI_VictorSPX spxBL = {2};

//Right setup

WPI_VictorSPX spxFR = {3};
WPI_VictorSPX spxMR = {4};
WPI_VictorSPX spxBR = {5};


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
        - Turning on the Falcon 500
    -Left Trigger
        - Unassigned
    -X Button
        - in working
    -A Button
        - Unassigned
    -B Button 
        - High gear shifting
    -Y Button
        - Low gear shifting
    -Left Joystick Button
        -
    -Right Joystick Button
        - 
    -Start Button
        - 
    -Back Button
        -

*/
std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");


void Robot::RobotInit() {

    FX1.ConfigFactoryDefault();

    //Sets the peak  and nominal outputs, 12v   

    FX1.ConfigNominalOutputForward(0);
    FX1.ConfigNominalOutputReverse(0);
    FX1.ConfigPeakOutputForward(1);
    FX1.ConfigPeakOutputReverse(-1);

    //Set closed loop gains in slot 0

    FX1->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
    FX1->Config_kP(kPIDLoopIdx, 0.0, kTimeoutMs);
    FX1->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
    FX1->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);


//Initial speed of the motors
        drive.ArcadeDrive(0, 0, 0);
//sensor setup
     FX1->ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
     srxBR.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0);
     srxBL.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0);

}

void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
    
//when teleop Initialy starts sets speed of all the motors
    drive.ArcadeDrive(0, 0, 0);
    FX1->Set(ControlMode::PercentOutput, 0);

}

void toggle() {

    if (Controller1->GetXButtonPressed()) {

        table->PutNumber("camMode", !table->GetNumber("camMode", 0));
        table->PutNumber("ledMode", !table->GetNumber("ledMode", 0));

    }

}

void rotateToTarget() {
    double tx = table->GetNumber("tx", 0.0);
    double output = LimelightConstants::kP*tx;
    drive.ArcadeDrive(0.0, output);
}
void Robot::TeleopPeriodic() {
//     // Get trigger axis and assigns x button as a bool value
//     double RTriggerAxis = Controller1.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand);
//     double MotorOutput = FX1.GetMotorOutputPercent();
//     bool XBtn = Controller1.GetXButton();
//     //prepares line to print
//     _sb.append("\tout:");
//     _sb.append(std::to_string(MotorOutput));
//     _sb.append("\tcur:");
//     _sb.append(std::to_string(FX1.GetOutputCurrent()));
// //  on XBtn pressed enters the closed loop mode
//     if (XBtn == 1) {
//         //Position mode
//         FX1.Set(ControlMode::Current, RTriggerAxis * 40); //40 amps on press
//     } else {
//         FX1.Set(ControlMode::PercentOutput, RTriggerAxis);
//     }
// //  If talon is in position closed loop print some more info
//     if (FX1.GetControlMode() == ControlMode::Current) {
//         // append more signals to print when in speed mode. 
// 			_sb.append("\terrNative:");
// 			_sb.append(std::to_string(FX1.GetClosedLoopError(kPIDLoopIdx)));
// 			_sb.append("\ttrg:");
// 			_sb.append(std::to_string(RTriggerAxis * 40));
// }

//     // Prints every ten loops
//     if (++_loops >= 10) {
//         _loops = 0;
//         printf("%s\n", _sb.c_str());
// }

//     _sb.clear();

// }

    std::cout << "Left Sensor Velocity: " << spxBL.GetSelectedSensorVelocity() << std::endl;
    std::cout << "Right Sensor Velocity: " << spxBR.GetSelectedSensorVelocity() << std::endl;

    while (Controller1.GetAButton()) {
        rotateToTarget();

    }
    toggle();

    std::cout << HEman.Get();
    if (HEman.Get()) {
        Blocker.Set(1);
}
    else {
        Blocker.Set(0);
}
    if (Controller1.GetBButtonPressed()) { 
        
        DsoleDTrain.Set(frc::DoubleSolenoid::kForward);
        
}

    else if (Controller1.GetYButtonPressed()) { 

        DsoleDTrain.Set(frc::DoubleSolenoid::kReverse);
        
}

    //Driving/Turning of the robot

    //Turns the .GetX into a double value where Arcadedrive can understand it
    double Turn = Controller1.GetX(frc::GenericHID::JoystickHand::kRightHand);

    //Turns the .GetY into a double value where Arcadedrive can understand it
    double Drive = Controller1.GetY(frc::GenericHID::JoystickHand::kLeftHand);

    drive.ArcadeDrive(Drive, Turn, true);

    //Flywheel motor
    // FX1.Set(ControlMode::PercentOutput, Controller1.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand));
}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
