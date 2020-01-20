/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include "Robot.h"
#include "ctre/Phoenix.h"
#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include <frc/Solenoid.h>
#include <frc/SolenoidBase.h>
#include <frc/DoubleSolenoid.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>



//Piston Fire-Solenoid setup    

frc::DoubleSolenoid dsole { 0, 0, 1};

//Sets up controller and Motor subsystems

frc::XboxController Controller1{0};

//Basic Motor Control bases

WPI_TalonSRX srxFL = {1};
WPI_TalonSRX srxML = {2};
WPI_TalonSRX srxFR = {4};
WPI_TalonSRX srxMR = {5};

 frc::SpeedControllerGroup left(srxFL, srxML);
 frc::SpeedControllerGroup right(srxFR, srxMR);

 frc::DifferentialDrive drive(left, right);

 //double variable for defining a turning # between -1 to 1 based on the axis of the right hand joystick

double turning = Controller1.GetX(frc::GenericHID::JoystickHand::kRightHand);

//Falcon 500 setup
TalonFX FX1 = {6};
TalonFX FX2 = {7};


void Robot::RobotInit() {

//Initial speed of the motors
        drive.ArcadeDrive(0, 0, 0);


    FX1.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
//when teleop Intialy starts sets speed of all the motors
    drive.ArcadeDrive(0, 0, 0);

    FX1.Set(ControlMode::PercentOutput, 0);

}
void Robot::TeleopPeriodic() {

    std::cout << FX1.GetSelectedSensorVelocity() << std::endl;

// Pneumatics control
   if (Controller1.GetAButtonPressed()) {
        dsole.Set(frc::DoubleSolenoid::Value::kForward);
        std::cout << Controller1.GetAButton() << std::endl;
    }

    else if (Controller1.GetAButtonReleased()) {
        dsole.Set(frc::DoubleSolenoid::Value::kReverse);
        std::cout << Controller1.GetAButton() << std::endl;
    }

// Motor control
    drive.ArcadeDrive(0.6, 0, true);

// turning function
    drive.ArcadeDrive(0, turning, true));


    // left.Set(ControlMode::PercentOutput, Controller1.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand));
    // right.Set(ControlMode::PercentOutput, Controller1.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand));

    //   srxFL.Set(ControlMode::PercentOutput, Controller1.GetY(frc::GenericHID::JoystickHand::kLeftHand));
    //   srxFR.Set(ControlMode::PercentOutput, Controller1.GetY(frc::GenericHID::JoystickHand::kLeftHand));
    //   srxML.Set(ControlMode::PercentOutput, Controller1.GetY(frc::GenericHID::JoystickHand::kLeftHand));
    //   srxMR.Set(ControlMode::PercentOutput, Controller1.GetY(frc::GenericHID::JoystickHand::kLeftHand));

//Shooter Control

    if (Controller1.GetXButtonPressed()) {

        FX1.Set(ControlMode::PercentOutput, Controller1.GetY(frc::GenericHID::JoystickHand::kLeftHand));
        FX2.Set(ControlMode::PercentOutput, Controller1.GetY(frc::GenericHID::JoystickHand::kLeftHand));

    }

    else if (Controller1.GetXButtonReleased()) {
        FX1.Set(ControlMode::PercentOutput, 0);
        FX2.Set(ControlMode::PercentOutput, 0);

    }
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
