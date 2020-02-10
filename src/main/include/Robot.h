#pragma once
#include <frc/TimedRobot.h>
#include "ctre/Phoenix.h"
#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include <frc/DigitalInput.h>
#include <frc/Servo.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "Interactions.h"
#include "CompBot/Intake/Pneumatic_Intake.h"
#include "CompBot/ColorMatcher.h"
#include "LimeLight.h"

class Robot : public frc::TimedRobot {
  private:
    TalonFX *FX1;
    frc::XboxController *Controller1;
    frc::XboxController *Controller2;
    Interactions *interaction;
    int _loops = 0;
    int _loopsIMU = 0;
    Pneumatic_Intake *intakeDeploy;
    PigeonIMU *pidgey;
    ColorMatcher *matcher;
    // VisionTracking *visiontracking;

  public:

    void RobotInit() override;

    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;

    void TestInit() override;
    void TestPeriodic() override;

    void toggleCameraMode();

    void autoGearShifter();

    void shooterVelTracking();

    void visionTracking();

    void elevatorControl();
};
