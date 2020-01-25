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
#include <frc/DigitalInput.h>
#include <frc/Servo.h>






//Piston Fire-Solenoid setup    

frc::DoubleSolenoid Dsole { 0, 0, 1};
frc::DoubleSolenoid DsoleDTrain {1, 0, 1};

//Sets up controller

frc::XboxController Controller1{0};

//Basic Motor Control bases

//Left speed group
WPI_TalonSRX srxFL = {1};
WPI_TalonSRX srxML = {2};
//for 2019 robot
WPI_TalonSRX srxBL = {3};

//Right speed group
WPI_TalonSRX srxFR = {4};
WPI_TalonSRX srxMR = {5};
//for 2019 robot
WPI_TalonSRX srxBR = {6};

frc::Servo Blocker {0};

 frc::SpeedControllerGroup left(srxFL, srxML, srxBL);
 frc::SpeedControllerGroup right(srxFR, srxMR, srxBR);

 frc::DifferentialDrive drive(left, right);


//Falcon 500 setup
TalonFX FX1 = {6};



void Robot::RobotInit() {


//Initial speed of the motors
        drive.ArcadeDrive(0, 0, 0);
//sensor setup
     FX1.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
     srxFL.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
     srxFR.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

    
}

void Robot::AutonomousInit() {
    Dsole.Set(frc::DoubleSolenoid::Value::kForward);

}
void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
//when teleop Initialy starts sets speed of all the motors
    drive.ArcadeDrive(0, 0, 0);
    FX1.Set(ControlMode::PercentOutput, 0);
}

void Robot::TeleopPeriodic() {

    if (Controller1.GetBumperPressed(frc::GenericHID::JoystickHand::kLeftHand)) {

        DsoleDTrain.Set(frc::DoubleSolenoid::Value::kForward);

    } 
    
    else if (Controller1.GetBumperPressed(frc::GenericHID::JoystickHand::kRightHand)) {

        DsoleDTrain.Set(frc::DoubleSolenoid::Value::kReverse);

    }

    double Turn = Controller1.GetX(frc::GenericHID::JoystickHand::kRightHand);
    double Drive = Controller1.GetY(frc::GenericHID::JoystickHand::kLeftHand);

    drive.ArcadeDrive(Drive, Turn, true);

    FX1.Set(ControlMode::PercentOutput, Controller1.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand));

    //std::cout << FX1.GetSelectedSensorVelocity() << std::endl;

    if (Controller1.GetStickButtonPressed(frc::GenericHID::JoystickHand::kLeftHand)) {

        Dsole.Set(frc::DoubleSolenoid::Value::kReverse); 
    
    }

    // turning & Driving function

    //Shooter Control
    if (Controller1.GetAButtonPressed()) { 
        
        Dsole.Set(frc::DoubleSolenoid::Value::kForward); 
        
    }

    FX1.Set(ControlMode::PercentOutput, Controller1.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand));

    if (Controller1.GetStartButton()) {

        std::cout << "Sensor Left Velocity: " << srxFL.GetSelectedSensorVelocity() << std::endl;
        std::cout << "Sensor Left Position: " << srxFL.GetSelectedSensorPosition() << std::endl;
        std::cout << "Left Output %: " << srxFL.GetMotorOutputPercent() << std::endl;

        std::cout << "Sensor Right Velocity: " << srxFR.GetSelectedSensorVelocity() << std::endl;
        std::cout << "Sensor Right Position: " << srxFR.GetSelectedSensorPosition() << std::endl;
        std::cout << "Right Output %: " << srxFR.GetMotorOutputPercent() << std::endl;

    }

}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
