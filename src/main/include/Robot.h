/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <GenericHID.h>

#include <VicorSPX.h>

#include <XboxController.h>

#include <DifferentialDrive.h>

#include "ctre/Phoenix.h"

#include <frc/TimedRobot.h>

#include <frc/smartdashboard/SendableChooser.h>


class Robot : public frc::TimedRobot {
  frc::DifferentialDrive m_robotDrive {

    frc::SpeedControllerGroup Left {

      ctre::CanTalonSRX m_right_Motor{0};
      ctre::CanTalonSRX m_backright_Motor{1};
      ctre::VictorSPX m_topright_Motor{2}

    };

     frc::SpeedControllerGroup Right {

      ctre::CanTalonSRX m_backLeft_Motor{3};
      ctre::CanTalonSRX m_Left_Motor{4};
      ctre::VictorSPX m_topleft_Motor{5};

    };
  };

  frc::XboxController m_driverController{0};

 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
