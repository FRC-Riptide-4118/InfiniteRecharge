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
#include <frc/smartdashboard/smartdashboard.h>
#include "LimeLight.h"

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

    // visiontracking = new VisionTracking(frc::XboxController *controller1, frc::DifferentialDrive drive);

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

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {

    // When teleop initialy starts sets speed of all the motors
    drive.ArcadeDrive(0, 0, 0);
    FX1->Set(ControlMode::PercentOutput, 0);
    }

}

void Robot::TeleopPeriodic() {

    // double ypr[3];
    // double xyz_dps[3];
    // double xyz_deg[3];
    // std::string yprstr;

    // double TriggerAxis = Controller1->GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand);   
    // double motorOutput = FX1->GetMotorOutputPercent();
    // std::string _sb;

    // _sb.append("\tout");
    // _sb.append(std::to_string(motorOutput));
    // _sb.append("\tspd: ");

    // _sb.append(std::to_string(FX1->GetSelectedSensorVelocity(kPIDLoopIdx)));

    // if(Controller1->GetXButton()) {

    //     double targetVelocity_UnitsPer100Ms = 17000;
                steering_adjust = Kp*heading_error + min_command;

    //     FX1->Set(ControlMode::Velocity, targetVelocity_UnitsPer100Ms);

    }

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


    // std::cout << pidgey->GetFusedHeading() << std::endl;



    if (interaction->getShiftGear()) {
        shifter->ShiftGear();
        
    }









    if (interaction->deployPneumatic_Intake()) {
        intakeDeploy->deployIntake();
    }

    //Driving/Turning of the robot
    double Turn = interaction->getTurn();
    double Drive = interaction->getDrive();
    bool squareInputs = true;
    drive.ArcadeDrive(Drive, Turn, squareInputs);

    // Color sensor
    matcher->getSeenColor();
    matcher->getMatchedColor();
    bool putColor = true;
    matcher->putDashboardTelemetry(putColor);

    if (Controller1->GetBumperPressed(frc::GenericHID::JoystickHand::kLeftHand)) {
        toggleCameraMode();
    }

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

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

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
    double ele_Up   = interaction->elevatorUp();
    double ele_Down = interaction->elevatorDown();

    elevator.ArcadeDrive(ele_Up, 0, squareInputs);

    if (interaction->driveElevatorDown()) {
            elevator.ArcadeDrive(-ele_Down, 0, squareInputs);
    } 

} 

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
