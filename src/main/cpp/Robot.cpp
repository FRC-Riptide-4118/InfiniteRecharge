/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include "Robot.h"
#include "Interactions.h"
#include "Subsystems/Drivetrain/GearShifter.h"
#include "Subsystems/Drivetrain/GearBoxMotors.h"
#include "Constants.h"
#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include "ctre/Phoenix.h"
#include <frc/DigitalInput.h>
#include <frc/Servo.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

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
        - Changing the value of the Falcon 500
    -Left Trigger
        - Unassigned
    -X Button
        - Enters closed loop target positioning
    -A Button
        - toggleCameraMode
    -B Button 
        - High/low gear shifting
    -Y Button
        - 
    -Left Joystick Button
        - Unassigned
    -Right Joystick Button
        - Unassigned
    -Start Button
        - Unassigned
    -Back Button
        -unassigned

*/
 std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");


void Robot::RobotInit() {
    shifter = new GearShifter();
    FX1 = new TalonFX(0);
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
     srx_Right_Back.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0);
     srx_Left_Back.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0);

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
    const double AMPS_CHANGE_DIRECTION = 20;    
    double motorOutput = FX1->GetOutputCurrent();
    std::string _sb;

    /* prepare line to print */
		_sb.append("\tout:");
		_sb.append(std::to_string(motorOutput));
		_sb.append("\tcur:");
		_sb.append(std::to_string(FX1->GetOutputCurrent()));
		/* on button1 press enter closed-loop mode on target position */
		if (interaction->enterPIDFXClosedLoop()) {
			/* Position mode - button just pressed */
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

    std::cout << "Left Sensor Velocity: " << srx_Left_Back.GetSelectedSensorVelocity() << std::endl;
//  std::cout << "Right Sensor Velocity: " << srxBR.GetSelectedSensorVelocity() << std::endl;

    toggleCameraMode();

    if ( interaction->getShiftGear() ) {
        shifter->shiftGear();
    }

    //Driving/Turning of the robot
    double Turn = interaction->getTurn();
    double Drive = interaction->getDrive();

    drive.ArcadeDrive(Drive, Turn, true);

}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
