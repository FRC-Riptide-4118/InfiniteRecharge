#include "ctre/Phoenix.h"
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>

WPI_TalonSRX srx_Left_Front = {0};
WPI_TalonSRX srx_Left_Middle = {1};
WPI_TalonSRX srx_Left_Back = {2};

WPI_TalonSRX srx_Right_Front = {3};
WPI_TalonSRX srx_Right_Middle = {4};
WPI_TalonSRX srx_Right_Back = {5};

WPI_VictorSPX spx_Left_Front = {0};
WPI_VictorSPX spx_Left_Middle = {1};
WPI_VictorSPX spx_Left_Back = {2};


WPI_VictorSPX spx_Right_Front = {3};
WPI_VictorSPX spx_Right_Middle = {4};
WPI_VictorSPX spx_Right_Back = {5};

frc::SpeedControllerGroup left(srx_Left_Front, srx_Left_Middle, srx_Left_Back);
frc::SpeedControllerGroup right(srx_Right_Front, srx_Right_Middle, srx_Right_Back);

frc::DifferentialDrive drive(left, right);

class MotorSetup {
  private:
  public: 
};

