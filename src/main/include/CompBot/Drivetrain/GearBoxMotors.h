#include "ctre/Phoenix.h"
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>

WPI_TalonSRX srx_left_front     = {0};
WPI_TalonSRX srx_left_middle    = {1};
WPI_VictorSPX spx_left_back      = {2};

WPI_TalonSRX srx_right_front    = {3};
WPI_TalonSRX srx_right_middle   = {4};
WPI_VictorSPX spx_right_back     = {5};

frc::SpeedControllerGroup left(srx_left_front, srx_left_middle, spx_left_back);
frc::SpeedControllerGroup right(srx_right_front, srx_right_middle, spx_right_back);

frc::DifferentialDrive drive(left, right);

class MotorSetup {
  private:
  public: 
};

