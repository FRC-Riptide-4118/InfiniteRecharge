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


void Robot::RobotInit() {
    shifter = new GearShifter();
    intakeDeploy = new Pneumatic_Intake();
    FX1 = new TalonFX(6);
    Controller1 = new frc::XboxController(0);
    Controller2 = new frc::XboxController(1);
    interaction = new Interactions( Controller1, Controller2 );

    FX1->ConfigFactoryDefault();

    //Sets the peak  and nominal outputs, 12v   

    FX1->ConfigNominalOutputForward(1, kTimeoutMs);
    FX1->ConfigNominalOutputReverse(-1, kTimeoutMs);
    FX1->ConfigPeakOutputForward(1, kTimeoutMs);
    FX1->ConfigPeakOutputReverse(-1, kTimeoutMs);

    //Set closed loop gains in slot 0

    FX1->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
    FX1->Config_kP(kPIDLoopIdx, 0.0, kTimeoutMs);
    FX1->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
    FX1->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);


//Initial speed of the motors
        drive.ArcadeDrive(0, 0, 0);
//sensor setup
     FX1->ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
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
    const double AMPS_CHANGE_DIRECTION = 30;    
    double motorOutput = FX1->GetOutputCurrent();
    std::string _sb;

    /* prepare line to print */
		_sb.append("\tout:");
		_sb.append(std::to_string(motorOutput));
		_sb.append("\tcur:");
		_sb.append(std::to_string(FX1->GetOutputCurrent()));
        std::cout << FX1->GetSelectedSensorVelocity() << std::endl;
		/* on x press enter closed-loop mode on target position */
		if (interaction->enterPIDFXClosedLoop()) {
			/* Position mode - button just pressed */
            std::cout << "enteredPIDFXClosedLoop" << std::endl;
			FX1->Set(ControlMode::Current, interaction->shooterRawSpeed() * AMPS_CHANGE_DIRECTION); /* 40 Amps in either direction */
		} else {
			FX1->Set(ControlMode::PercentOutput, interaction->shooterRawSpeed());
		}
		/* if Talon is in position closed-loop, print some more info */
		if (FX1->GetControlMode() == ControlMode::Current) {
			/* append more signals to print when in speed mode. */
			_sb.append("\terrNative:");
			_sb.append(std::to_string(FX1->GetClosedLoopError(kPIDLoopIdx)));
			_sb.append("\ttrg:");
			_sb.append(std::to_string(interaction->shooterRawSpeed() * AMPS_CHANGE_DIRECTION));
		}
		/* print every ten loops, printing too much too fast is generally bad for performance */
		if (++_loops >= 10) {
			_loops = 0;
			printf("%s\n", _sb.c_str());
		}
		_sb.clear();

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

    if (FX1->GetSelectedSensorVelocity() >= 20000) {
        falcon_Test.Set(1);
    } else {
        falcon_Test.Set(0);
    }



    // SmartDashboard telemetry
    frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    frc::SmartDashboard::PutNumber("Green", detectedColor.green);
    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
    frc::SmartDashboard::PutString("Color Decision", colorString);
    frc::SmartDashboard::PutNumber("IR", IR);

}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
