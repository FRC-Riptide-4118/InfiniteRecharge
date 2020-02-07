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
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>
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

 frc::Servo elevator_Stop_Left  (0);
 frc::Servo elevator_Stop_Right (1);
 frc::Servo conveyor_Hard_Stop  (2);

 frc::DigitalInput limitSwitch_Test {1};



WPI_TalonSRX srx_left_front     = {0};
WPI_TalonSRX srx_left_middle    = {1};
WPI_VictorSPX spx_left_back     = {2};

WPI_TalonSRX srx_right_front    = {3};
WPI_TalonSRX srx_right_middle   = {4};
WPI_VictorSPX spx_right_back    = {5};

frc::SpeedControllerGroup left(srx_left_front, srx_left_middle, spx_left_back);
frc::SpeedControllerGroup right(srx_right_front, srx_right_middle, spx_right_back);

frc::DifferentialDrive drive(left, right);

void Robot::RobotInit() {
    shifter = new GearShifter();
    intakeDeploy = new Pneumatic_Intake();
    FX1 = new TalonFX(6);
    Controller1 = new frc::XboxController(0);
    Controller2 = new frc::XboxController(1);
    interaction = new Interactions( Controller1, Controller2 );
    pidgey = new PigeonIMU(&srx_left_middle);
    pidgey->SetFusedHeading(0);


    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kGreenTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);
    m_colorMatcher.AddColorMatch(kYellowTarget);

    FX1->ConfigFactoryDefault();

    FX1->ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
    FX1->SetSensorPhase(true);

    FX1->ConfigNominalOutputForward(0, kTimeoutMs);
    FX1->ConfigNominalOutputReverse(0, kTimeoutMs);
    FX1->ConfigPeakOutputForward(1, kTimeoutMs);
    FX1->ConfigPeakOutputReverse(-1, kTimeoutMs);

    FX1->ConfigAllowableClosedloopError(kPIDLoopIdx, 10, kTimeoutMs);

    FX1->Config_kF(kPIDLoopIdx, 0.045, kTimeoutMs);
    FX1->Config_kP(kPIDLoopIdx, 0.069, kTimeoutMs);
    FX1->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
    FX1->Config_kD(kPIDLoopIdx, 0.01, kTimeoutMs);

//Initial speed of the motors
    drive.ArcadeDrive(0, 0, 0);
//sensor setup
    srx_right_front.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0);
    srx_left_front.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0);

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

void Robot::TeleopPeriodic() {

    double ypr[3];
    double xyz_dps[3];
    double xyz_deg[3];
    std::string yprstr;

    // double TriggerAxis = Controller1->GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand);   
    // double motorOutput = FX1->GetMotorOutputPercent();
    // std::string _sb;

    // _sb.append("\tout");
    // _sb.append(std::to_string(motorOutput));
    // _sb.append("\tspd: ");

    // _sb.append(std::to_string(FX1->GetSelectedSensorVelocity(kPIDLoopIdx)));

    // if(Controller1->GetXButton()) {

    //     double targetVelocity_UnitsPer100Ms = 17000;

    //     FX1->Set(ControlMode::Velocity, targetVelocity_UnitsPer100Ms);


    //     _sb.append("\terrNative:");
    //     _sb.append(std::to_string(FX1->GetClosedLoopError(kPIDLoopIdx)));
    //     _sb.append("\ttrg:");
    //     _sb.append(std::to_string(targetVelocity_UnitsPer100Ms));
    // } else {

    //     FX1->Set(ControlMode::PercentOutput, TriggerAxis);
    // }

    // if (++_loops >= 10) {
    //     _loops = 0;
    //     printf("%s\n", _sb.c_str());
    // }

    toggleCameraMode();

    if ( interaction->getShiftGear() ) {
        shifter->ShiftGear();
    }

    if (interaction->deployPneumatic_Intake() ) {
        intakeDeploy->deployIntake();
    }


    std::cout << pidgey->GetFusedHeading() << std::endl;


    //Driving/Turning of the robot
    double Turn = interaction->getTurn();
    double Drive = interaction->getDrive();

    drive.ArcadeDrive(Drive, Turn, true);

    frc::Color detectedColor = m_colorSensor.GetColor();

    std::string colorString;
    double confidence = 0.0;
    frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

    if (matchedColor == kBlueTarget) {
        colorString = "Blue";
    } else if (matchedColor == kRedTarget) {
        colorString = "Red";
    } else if (matchedColor == kGreenTarget) {
        colorString = "Green";
    } else if (matchedColor == kYellowTarget) {
        colorString = "Yellow";
    } else {
        colorString = "Unknown";
    }

    // SmartDashboard telemetry
    frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    frc::SmartDashboard::PutNumber("Green", detectedColor.green);
    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
    frc::SmartDashboard::PutString("Color Decision", colorString);

}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}



void Robot::toggleCameraMode() {

    if (interaction->toggleLimeLightCamera()) {

        table->PutNumber("camMode", !table->GetNumber("camMode", 0));
        table->PutNumber("ledMode", !table->GetNumber("ledMode", 0));

    }

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
