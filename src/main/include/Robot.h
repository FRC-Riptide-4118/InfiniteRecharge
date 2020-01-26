#pragma once
#include <frc/TimedRobot.h>
#include "ctre/Phoenix.h"
#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include <frc/DigitalInput.h>
#include <frc/Servo.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;
};
