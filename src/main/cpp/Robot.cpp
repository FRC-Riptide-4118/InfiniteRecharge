/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include "Robot.h"
#include "Interactions.h"
#include "CompBot/Drivetrain/GearShifter.h"
#include "CompBot/Drivetrain/GearBoxMotors.h"
#include "Constants.h"
#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include "ctre/Phoenix.h"
#include <frc/DigitalInput.h>
#include <frc/Servo.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

#include <frc/smartdashboard/smartdashboard.h>


 std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
 frc::Servo falcon_Test(0);

 frc::DigitalInput limitSwitch_Test {1};


void Robot::RobotInit() {
    shifter = new GearShifter();
    intakeDeploy = new Pneumatic_Intake();
    FX1 = new TalonFX(6);
    Controller1 = new frc::XboxController(0);
    Controller2 = new frc::XboxController(1);
    interaction = new Interactions( Controller1, Controller2 );


    FX1->ConfigFactoryDefault();

    FX1->ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
    FX1->SetSensorPhase(true);

    FX1->ConfigNominalOutputForward(0, kTimeoutMs);
    FX1->ConfigNominalOutputReverse(0, kTimeoutMs);
    FX1->ConfigPeakOutputForward(1, kTimeoutMs);
    FX1->ConfigPeakOutputReverse(-1, kTimeoutMs);

    FX1->Config_kF(kPIDLoopIdx, 0.1097, kTimeoutMs);
    FX1->Config_kP(kPIDLoopIdx, 0.22, kTimeoutMs);
    FX1->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
    FX1->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

//Initial speed of the motors
        drive.ArcadeDrive(0, 0, 0);
//sensor setup
     srx_right_back.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0);
     srx_left_back.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0);

}

void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
    
//when teleop Initialy starts sets speed of all the motors
    drive.ArcadeDrive(0, 0);
    FX1->Set(ControlMode::PercentOutput, 0);

}

void Robot::toggleCameraMode() {

    if (interaction->toggleLimeLightCamera()) {

        table->PutNumber("camMode", !table->GetNumber("camMode", 0));
        table->PutNumber("ledMode", !table->GetNumber("ledMode", 0));

    }

}


void Robot::TeleopPeriodic() {

    // get gamepad axis
    double TriggerAxis = Controller1->GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand);   
    double motorOutput = FX1->GetMotorOutputPercent();
    std::string _sb;

    _sb.append("\tout");
    _sb.append(std::to_string(motorOutput));
    _sb.append("\tspd: ");
    _sb.append(std::to_string(FX1->GetSelectedSensorVelocity(kPIDLoopIdx)));

    if(Controller1->GetXButtonPressed()) {

        double targetVelocity_UnitsPer100Ms = TriggerAxis * 500.0 * 4096 / 600;

        FX1->Set(ControlMode::Velocity, targetVelocity_UnitsPer100Ms);


        _sb.append("\terrNative:");
        _sb.append(std::to_string(FX1->GetClosedLoopError(kPIDLoopIdx)));
        _sb.append("\ttrg:");
        _sb.append(std::to_string(targetVelocity_UnitsPer100Ms));
    } else {

        FX1->Set(ControlMode::PercentOutput, TriggerAxis);
    }

    if (++_loops >= 10) {
        _loops = 0;
        printf("%s\n", _sb.c_str());
    }
   

//   std::cout << "Left Sensor Velocity: " << srx_Left_Back.GetSelectedSensorVelocity() << std::endl;

//   std::cout << "Right Sensor Velocity: " << srxBR.GetSelectedSensorVelocity() << std::endl;

    toggleCameraMode();

    if ( interaction->getShiftGear() ) {
        shifter->ShiftGear();
    }

    if (interaction->deployPneumatic_Intake() ) {
        intakeDeploy->deployIntake();
    }

    //Driving/Turning of the robot
    double Turn = interaction->getTurn();
    double Drive = interaction->getDrive();

    // drive.ArcadeDrive(Drive, Turn, true);


// ctre::pheonix::sensors::PigeonIMU::PigeonIMU(srx_left_back);

//     // SmartDashboard telemetry
//     frc::SmartDashboard::PutNumber("Red", detectedColor.red);
//     frc::SmartDashboard::PutNumber("Green", detectedColor.green);
//     frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
//     frc::SmartDashboard::PutString("Color Decision", colorString);
//     frc::SmartDashboard::PutNumber("IR", IR);

// }

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
