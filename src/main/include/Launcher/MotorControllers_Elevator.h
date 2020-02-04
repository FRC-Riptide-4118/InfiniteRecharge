#include "ctre/Pheonix.h"
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>

WPI_VictorSPX spx_Elev_Left    = {9};
WPI_VictorSPX spx_Elev_Right   = {10};

frc::SpeedControllerGroup Elev_Left(spx_Elev_Left);
frc::SpeedControllerGroup Elev_Right(spx_Elev_Right);

frc::DifferentialDrive Elevator(Elev_Left, Elev_Right);