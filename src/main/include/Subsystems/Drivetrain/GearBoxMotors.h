#include "ctre/Phoenix.h"
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>

WPI_TalonSRX srx_left_front     = {0};
WPI_TalonSRX srx_left_middle    = {1};
WPI_TalonSRX srx_left_back      = {2};

WPI_TalonSRX srx_right_front    = {3};
WPI_TalonSRX srx_right_middle   = {4};
WPI_TalonSRX srx_right_back     = {5};

WPI_VictorSPX spx_left_front    = {0};
WPI_VictorSPX spx_left_middle   = {1};
WPI_VictorSPX spx_left_back     = {2};


WPI_VictorSPX spx_right_front   = {3};
WPI_VictorSPX spx_right_middle  = {4};
WPI_VictorSPX spx_right_back    = {5};

frc::SpeedControllerGroup left(srx_Left_Front, srx_Left_Middle, srx_Left_Back);
frc::SpeedControllerGroup right(srx_Right_Front, srx_Right_Middle, srx_Right_Back);

frc::DifferentialDrive drive(left, right);

class MotorSetup {
  private:
  public: 
};

