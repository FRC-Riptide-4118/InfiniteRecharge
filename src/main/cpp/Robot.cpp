/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include "Robot.h"
#include "Interactions.h"
#include "Subsystems/Drivetrain/Pneumatics.h"
#include "Subsystems/Drivetrain/Motors.h"
#include "Constants.h"

#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include "ctre/Phoenix.h"
#include <frc/DigitalInput.h>
#include <frc/Servo.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"


frc::SpeedControllerGroup left(srx_Left_Front, srx_Left_Middle, srx_Left_Back);
frc::SpeedControllerGroup right(srx_Right_Front, srx_Right_Middle, srx_Right_Back);

frc::DifferentialDrive drive(left, right);

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
        - Enters closed loop target positioning
    -A Button
        - Unassigned
    -B Button 
        - High gear shifting
    -Y Button
        - Low gear shifting
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
    FX1 = new TalonFX(0);
    Controller1 = new frc::XboxController(0);
    Controller2 = new frc::XboxController(1);
    interaction = new Interactions( Controller1, Controller2 );
    shifter_highgear = false;

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

void Robot::toggle() {

    if (Controller1->GetAButtonPressed()) {

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
//    std::cout << "Right Sensor Velocity: " << srxBR.GetSelectedSensorVelocity() << std::endl;

    toggle();

    if ( interaction->getShiftGear() ) {
        shifter_highgear = !shifter_highgear;
    }

    if (shifter_highgear) { 
        Shifter.Set(frc::DoubleSolenoid::kForward);
    } else { 
        Shifter.Set(frc::DoubleSolenoid::kReverse);
    }

    //Driving/Turning of the robot

    //Turns the .GetX into a double value where Arcadedrive can understand it
    double Turn = interaction->getTurn();

    //Turns the .GetY into a double value where Arcadedrive can understand it
    double Drive = interaction->getDrive();

    drive.ArcadeDrive(Drive, Turn, true);

}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
