/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include "Robot.h"
#include "Interactions.h"
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>
#include "Constants.h"
#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include "ctre/Phoenix.h"
#include <frc/DigitalInput.h>
#include <frc/Servo.h>
#include <frc/smartdashboard/smartdashboard.h>
#include "LimeLight.h"
#include <chrono>

std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

WPI_VictorSPX ele_left  = {7}; 
WPI_VictorSPX ele_right = {8};

frc::SpeedControllerGroup ele_leftSPG(ele_left);
frc::SpeedControllerGroup ele_rightSPG(ele_right);

frc::DifferentialDrive elevator(ele_leftSPG, ele_rightSPG);

WPI_VictorSPX intake_motor = {9};

WPI_VictorSPX conveyor_motor = {10};

frc::Servo elevator_Stop_Left  (0);
frc::Servo elevator_Stop_Right (1);
frc::Servo conveyor_Hard_Stop  (2);
frc::Servo conveyor_Hard_Stop2 (3);

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

frc::DoubleSolenoid shifter(0, 0, 1);

bool squareInputs = true;


void Robot::RobotInit() {
    intakeDeploy = new Pneumatic_Intake();
    FX1 = new TalonFX(6);
    Controller1 = new frc::XboxController(0);
    Controller2 = new frc::XboxController(1);
    interaction = new Interactions( Controller1, Controller2 );
    pidgey = new PigeonIMU(&srx_left_middle);
    pidgey->SetFusedHeading(0);

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
    FX1->Config_kI(kPIDLoopIdx, 0.000, kTimeoutMs);
    FX1->Config_kD(kPIDLoopIdx, 0.010, kTimeoutMs);

    // Color Matcher
    matcher = new ColorMatcher;

    //Initial speed of the motors
    drive.ArcadeDrive(0, 0, 0);

    //sensor setup
    srx_right_front.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0);
    srx_left_front.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0);
}

void Robot::AutonomousInit() {

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    drive.ArcadeDrive(-10, 0, squareInputs);
    if (std::chrono::steady_clock::now() - start > std::chrono::seconds(3)) {
        drive.ArcadeDrive(0, 0, squareInputs);
    }

}

void Robot::AutonomousPeriodic() {

    autoGearShifter();

    float Kp = -0.1;
    float min_command = 0.5;
    int i = 0;

    while (i < 50) {
    
    i++;
    double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);

        double   heading_error = -tx;
        double   steering_adjust = 0.0;

            if (tx > 1.0) {
                steering_adjust = Kp*heading_error - min_command;
            } else if (tx < 1.0) {
                steering_adjust = Kp*heading_error + min_command;
            }

    drive.ArcadeDrive(0, steering_adjust, squareInputs);

    }

    std::string _sb;
    double motorOutput = FX1->GetMotorOutputPercent();

        _sb.append("\tout");
        _sb.append(std::to_string(motorOutput));
        _sb.append("\tspd: ");

        _sb.append(std::to_string(FX1->GetSelectedSensorVelocity(kPIDLoopIdx)));

            double targetVelocity_UnitsPer100Ms = 17000;

            FX1->Set(ControlMode::Velocity, targetVelocity_UnitsPer100Ms);


            _sb.append("\terrNative:");
            _sb.append(std::to_string(FX1->GetClosedLoopError(kPIDLoopIdx)));
            _sb.append("\ttrg:");
            _sb.append(std::to_string(targetVelocity_UnitsPer100Ms));

        if (++_loops >= 10) {
            _loops = 0;
            printf("%s\n", _sb.c_str());
        }
        
        if (FX1->GetSelectedSensorVelocity(kPIDLoopIdx) > 16000 or FX1->GetSelectedSensorVelocity(kPIDLoopIdx) < 18000) {
        conveyor_motor.Set(ControlMode::PercentOutput, 1);
    } else {
        conveyor_motor.Set(ControlMode::PercentOutput, 0);
    }

    if (interaction->runIntake()) {
        intake_motor.Set(ControlMode::PercentOutput, 100);
    } else {
        intake_motor.Set(ControlMode::PercentOutput, 0);
    }

    if (interaction->conveyorHardStop()) {
        conveyor_Hard_Stop.Set(1);
        conveyor_Hard_Stop2.Set(1);
    } else {
        conveyor_Hard_Stop.Set(0);
        conveyor_Hard_Stop2.Set(0);
    }

    if (interaction->turnEleServo()) {
        elevator_Stop_Left.Set(1);
        elevator_Stop_Right.Set(1);
    } else {
        elevator_Stop_Left.Set(0);
        elevator_Stop_Right.Set(0);
    }

}

void Robot::TeleopInit() {

    // When teleop initialy starts sets speed of all the motors
    drive.ArcadeDrive(0, 0, 0);
    FX1->Set(ControlMode::PercentOutput, 0);

}

void Robot::TeleopPeriodic() {

    // double ypr[3];
    // double xyz_dps[3];
    // double xyz_deg[3];
    // std::string yprstr;

    autoGearShifter();
    shooterVelTracking();

    // std::cout << pidgey->GetFusedHeading() << std::endl;

    if (interaction->deployPneumatic_Intake()) {
        intakeDeploy->deployIntake();
    }

    //Driving/Turning of the robot
    double Turn = interaction->getTurn();
    double Drive = interaction->getDrive();
    drive.ArcadeDrive(Drive, Turn, squareInputs);

    // Color sensor
    matcher->getSeenColor();
    matcher->getMatchedColor();
    bool putColor = true;
    matcher->putDashboardTelemetry(putColor);

    visionTracking();
    elevatorControl();



}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::visionTracking() {
     
    toggleCameraMode();

    float Kp = -0.1;
    float min_command = 0.5;

    double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);

    if (interaction->visionControl()) {

        double   heading_error = -tx;
        double   steering_adjust = 0.0;

            if (tx > 1.0) {
                steering_adjust = Kp*heading_error - min_command;
            } else if (tx < 1.0) {
                steering_adjust = Kp*heading_error + min_command;
            }

            drive.ArcadeDrive( 0, steering_adjust, 1 );
    }

}

void Robot::toggleCameraMode() {
    if (interaction->toggleLimeLightCamera()) {
        table->PutNumber("camMode", !table->GetNumber("camMode", 0));
        table->PutNumber("ledMode", !table->GetNumber("ledMode", 0));
    }
}

void Robot::autoGearShifter() {
    if (srx_left_front.GetSelectedSensorVelocity() > 11000 and srx_right_front.GetSelectedSensorVelocity() > 11000) {
        shifter.Set(frc::DoubleSolenoid::kForward);
    } else if (srx_left_front.GetSelectedSensorVelocity() < 11000 and srx_right_front.GetSelectedSensorVelocity() < 11000) {
        shifter.Set(frc::DoubleSolenoid::kReverse);
    }
}

void Robot::shooterVelTracking() {
    double motorOutput = FX1->GetMotorOutputPercent();
    std::string _sb;

    _sb.append("\tout");
    _sb.append(std::to_string(motorOutput));
    _sb.append("\tspd: ");

    _sb.append(std::to_string(FX1->GetSelectedSensorVelocity(kPIDLoopIdx)));

    if(interaction->enterPIDFXClosedLoop()) {

        double targetVelocity_UnitsPer100Ms = 17000;

        FX1->Set(ControlMode::Velocity, targetVelocity_UnitsPer100Ms);


        _sb.append("\terrNative:");
        _sb.append(std::to_string(FX1->GetClosedLoopError(kPIDLoopIdx)));
        _sb.append("\ttrg:");
        _sb.append(std::to_string(targetVelocity_UnitsPer100Ms));
    } 

    if (++_loops >= 10) {
        _loops = 0;
        printf("%s\n", _sb.c_str());
    }

    if (FX1->GetSelectedSensorVelocity(kPIDLoopIdx) > 16000 or FX1->GetSelectedSensorVelocity(kPIDLoopIdx) < 18000) {
        conveyor_motor.Set(ControlMode::PercentOutput, 1);
    } else {
        conveyor_motor.Set(ControlMode::PercentOutput, 0);
    }

}

void Robot::elevatorControl() {
    double ele_control   = interaction->elevatorControl();

    elevator.ArcadeDrive(ele_control, 0, squareInputs);

    if (interaction->driveElevatorDown()) {
            elevator.ArcadeDrive(-ele_control, 0, squareInputs);
    } 

} 

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
