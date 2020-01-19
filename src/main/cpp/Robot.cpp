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
frc::DoubleSolenoid ds { 0, 0, 1};

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

//Falcon 500 setup
TalonFX FX1 = {6};
TalonFX FX2 = {7};


void Robot::RobotInit() {

    srxFL.Set(ControlMode::PercentOutput, 0);
    srxFR.Set(ControlMode::PercentOutput, 0);
    srxML.Set(ControlMode::PercentOutput, 0);
    srxMR.Set(ControlMode::PercentOutput, 0);

    FX1.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
//Intial speed set of the motors
    // srxFL.Set(ControlMode::PercentOutput, 0);
    // srxFR.Set(ControlMode::PercentOutput, 0);
    // srxML.Set(ControlMode::PercentOutput, 0);
    // srxMR.Set(ControlMode::PercentOutput, 0);

    FX1.Set(ControlMode::PercentOutput, 0);
    FX2.Set(ControlMode::PercentOutput, 0);
}
void Robot::TeleopPeriodic() {

    std::cout << FX1.GetSelectedSensorVelocity() << std::endl;

// Pneumatics control
   if (Controller1.GetAButtonPressed()) {
        dsole.Set(frc::DoubleSolenoid::Value::kForward);
    }

    else if (Controller1.GetAButtonReleased()) {
        dsole.Set(frc::DoubleSolenoid::Value::kReverse);
    }
    
      srxFL.Set(ControlMode::PercentOutput, Controller1.GetY(frc::GenericHID::JoystickHand::kLeftHand));
      srxFR.Set(ControlMode::PercentOutput, Controller1.GetY(frc::GenericHID::JoystickHand::kLeftHand));
      srxML.Set(ControlMode::PercentOutput, Controller1.GetY(frc::GenericHID::JoystickHand::kLeftHand));
      srxMR.Set(ControlMode::PercentOutput, Controller1.GetY(frc::GenericHID::JoystickHand::kLeftHand));
     
    //if (Controller1.GetXButtonPressed()) {
       // FX1.Set(ControlMode::PercentOutput, Controller1.GetY(frc::GenericHID::JoystickHand::kLeftHand));
       // FX2.Set(ControlMode::PercentOutput, Controller1.GetY(frc::GenericHID::JoystickHand::kLeftHand));
   // }

    //if (Controller1.GetXButtonReleased()) {
    //    FX1.Set(ControlMode::PercentOutput, 0);
    //    FX2.Set(ControlMode::PercentOutput, 0);

   // }
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
