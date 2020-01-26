#include "ctre/Phoenix.h"
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>


//Basic Motor Control bases

//Left speed group
WPI_TalonSRX srxFL = {1};
WPI_TalonSRX srxML = {2};
WPI_TalonSRX srxBL = {3};

// //Right speed group
WPI_TalonSRX srxFR = {4};
WPI_TalonSRX srxMR = {5};
WPI_TalonSRX srxBR = {6};

 frc::SpeedControllerGroup left(srxFL, srxML, srxBL);
 frc::SpeedControllerGroup right(srxFR, srxMR, srxBR);

 frc::DifferentialDrive drive(left, right);