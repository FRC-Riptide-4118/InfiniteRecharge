#pragma once
#include <frc/TimedRobot.h>
#include "ctre/Phoenix.h"
#include "CompBot/Drivetrain/GearShifter.h"
#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include <frc/DigitalInput.h>
#include <frc/Servo.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "Interactions.h"
#include "CompBot/Intake/Pneumatic_Intake.h"
#include "rev/ColorSensorV3.h"
#include "rev/Colormatch.h"

class Robot : public frc::TimedRobot {
 private:
  TalonFX *FX1;
  frc::XboxController *Controller1;
  frc::XboxController *Controller2;
  Interactions *interaction;
  int _loops = 0;
  int _loopsIMU = 0;
  bool shifter_highgear;
  GearShifter *shifter;
  Pneumatic_Intake *intakeDeploy;
  PigeonIMU *pidgey;

 public:

 static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 m_colorSensor{i2cPort};
  rev::ColorMatch m_colorMatcher;

  static constexpr frc::Color kBlueTarget    = frc::Color(0.143, 0.427, 0.429);
  static constexpr frc::Color kGreenTarget   = frc::Color(0.197, 0.561, 0.240);
  static constexpr frc::Color kRedTarget     = frc::Color(0.561, 0.232, 0.114);
  static constexpr frc::Color kYellowTarget  = frc::Color(0.361, 0.524, 0.113);

  void RobotInit() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void toggleCameraMode();
};
