#pragma once
#include <frc/TimedRobot.h>
#include "ctre/Phoenix.h"
#include "Subsystems/Drivetrain/GearShifter.h"
#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include <frc/DigitalInput.h>
#include <frc/Servo.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "Interactions.h"

class Robot : public frc::TimedRobot {
 private:
  TalonFX * FX1;
  frc::XboxController * Controller1;
  frc::XboxController * Controller2;
  Interactions *interaction;
  int _loops = 0;
  bool shifter_highgear;
  GearShifter *shifter;

 public:
  void RobotInit() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void toggleCameraMode();
};
