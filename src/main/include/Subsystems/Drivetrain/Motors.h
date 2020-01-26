#include "ctre/Phoenix.h"
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>


//Basic Motor Control bases

//Left speed group
WPI_TalonSRX srx_Left_Front = {0};
WPI_TalonSRX srx_Left_Middle = {1};
WPI_TalonSRX srx_Left_Back = {2};

// //Right speed group
WPI_TalonSRX srx_Right_Front = {3};
WPI_TalonSRX srx_Right_Middle = {4};
WPI_TalonSRX srx_Right_Back = {5};

//Victor Sp setup for R2-Dan2
WPI_VictorSPX spx_Left_Front = {0};
WPI_VictorSPX spx_Left_Middle = {1};
WPI_VictorSPX spx_Left_Back = {2};

//Right setup

WPI_VictorSPX spx_Right_Front = {3};
WPI_VictorSPX spx_Right_Middle = {4};
WPI_VictorSPX spx_Right_Back = {5};