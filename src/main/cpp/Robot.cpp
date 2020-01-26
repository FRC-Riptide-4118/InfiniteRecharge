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
#include "Constants.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

// //Victor Sp setup for R2-Dan2
WPI_VictorSPX spxFL = {0};
WPI_VictorSPX spxML = {1};
WPI_VictorSPX spxBL = {2};

// //Right setup

WPI_VictorSPX spxFR = {3};
WPI_VictorSPX spxMR = {4};
WPI_VictorSPX spxBR = {5};
// //Hal Effect sensor setuo

frc::DigitalInput HEman {0};

//Piston Fire-Solenoid setup    

frc::DoubleSolenoid Dsole { 1, 0, 1};
frc::DoubleSolenoid DsoleDTrain {0, 0, 1};

//Sets up controller

frc::XboxController Controller1{0};

//Basic Motor Control bases

//Left speed group
// WPI_TalonSRX srxFL = {1};
// WPI_TalonSRX srxML = {2};
// //for 2019 robot
// WPI_TalonSRX srxBL = {3};

// //Right speed group
// WPI_TalonSRX srxFR = {4};
// WPI_TalonSRX srxMR = {5};
// //for 2019 robot
// WPI_TalonSRX srxBR = {6};

frc::Servo Blocker {0};

 frc::SpeedControllerGroup left(spxFL, spxML, spxBL);
 frc::SpeedControllerGroup right(spxFR, spxMR, spxBR);

 frc::DifferentialDrive drive(left, right);


//Falcon 500 setup & Vision tracking
TalonFX FX1 = {6};

std::string _sb;
int _loops = 0;


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

    FX1.Config_kF(kPIDLoopIdx, 0.0);
    FX1.Config_kP(kPIDLoopIdx, 0.0);
    FX1.Config_kI(kPIDLoopIdx, 0.0);
    FX1.Config_kD(kPIDLoopIdx, 0.0);


//Initial speed of the motors
        drive.ArcadeDrive(0, 0, 0);
//sensor setup
    //  FX1.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
    //  spxBR.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0);
    //  spxBL.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0);

     Blocker.Set(0);

}

void Robot::AutonomousInit() {

    Dsole.Set(frc::DoubleSolenoid::Value::kForward);

}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {


    // double RTriggerAxis = Controller1.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand);
    // double MotorOutput = FX1.GetMotorOutputPercent();


    DsoleDTrain.Set(frc::DoubleSolenoid::kReverse);

    Blocker.Set(0);
    
//when teleop Initialy starts sets speed of all the motors
    drive.ArcadeDrive(0, 0, 0);
    FX1.Set(ControlMode::PercentOutput, 0);

}

void toggle() {

    if (_LoopCount++ > 10) {

        _LoopCount = 0;
        PigIMU1.GetYawPitchRoll(double ypr);
        std::cout << "Pidgeon Yaw is:" << ypr << std::endl;

    }
        std::cout << "Pidgeon Yaw is: " << ypr[0] << std::endl;

    std::cout << "Left Sensor Velocity: " << srxBL.GetSelectedSensorVelocity() << std::endl;
    std::cout << "Right Sensor Velocity: " << srxBR.GetSelectedSensorVelocity() << std::endl;


    std::cout << HEman.Get();

    if (HEman.Get()) {

        Blocker.Set(1);

    }

    else {

        Blocker.Set(0);

    }


    if (Controller1.GetBumperPressed(frc::GenericHID::JoystickHand::kLeftHand)) { 
        
        DsoleDTrain.Set(frc::DoubleSolenoid::kForward);
        
    }

    else if (Controller1.GetBumperPressed(frc::GenericHID::JoystickHand::kRightHand)) { 

        DsoleDTrain.Set(frc::DoubleSolenoid::kReverse);
        
        }

    double Turn = Controller1.GetX(frc::GenericHID::JoystickHand::kRightHand);
    double Drive = Controller1.GetY(frc::GenericHID::JoystickHand::kLeftHand);
    drive.ArcadeDrive(Drive, Turn, true);

    FX1.Set(ControlMode::PercentOutput, Controller1.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand));

    //std::cout << FX1.GetSelectedSensorVelocity() << std::endl;

    if (Controller1.GetStickButtonPressed(frc::GenericHID::JoystickHand::kLeftHand)) { Dsole.Set(frc::DoubleSolenoid::Value::kReverse); }

    // turning & Driving function

    //Shooter Control
    if (Controller1.GetAButtonPressed()) { Dsole.Set(frc::DoubleSolenoid::Value::kForward); }

    FX1.Set(ControlMode::PercentOutput, Controller1.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand));

    // if (Controller1.GetStartButton()) {
    //     std::cout << "Sensor Left Velocity: " << srxFL.GetSelectedSensorVelocity() << std::endl;
    //     std::cout << "Sensor Left Position: " << srxFL.GetSelectedSensorPosition() << std::endl;
    //     std::cout << "Left Output %: " << srxFL.GetMotorOutputPercent() << std::endl;
    //     std::cout << "Sensor Right Velocity: " << srxFR.GetSelectedSensorVelocity() << std::endl;
    //     std::cout << "Sensor Right Position: " << srxFR.GetSelectedSensorPosition() << std::endl;
    //     std::cout << "Right Output %: " << srxFR.GetMotorOutputPercent() << std::endl;
    // }
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
