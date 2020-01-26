#include "ctre/Phoenix.h"
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>

WPI_VictorSPX spx_left_front    = {0};
WPI_VictorSPX spx_left_middle   = {1};
WPI_VictorSPX spx_left_back     = {2};


WPI_VictorSPX spx_right_front   = {3};
WPI_VictorSPX spx_right_middle  = {4};
WPI_VictorSPX spx_right_back    = {5};


frc::SpeedControllerGroup left_drive(spx_left_front, spx_left_middle, spx_left_back);
frc::SpeedControllerGroup righ_drive(spx_right_front, spx_right_middle, spx_right_back);

frc::DifferentialDrive drive(left_drive, right_drive);

class MotorSetup {
  private:
  public: 
};
