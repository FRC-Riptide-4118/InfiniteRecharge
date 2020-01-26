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
 private:
  TalonFX * FX1 = new TalonFX(0);
  frc::XboxController * Controller1 = new frc::XboxController(0);

  std::string _sb;
  int _loops = 0;

 public:
  void RobotInit() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;
};
