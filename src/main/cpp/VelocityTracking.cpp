// #include <iostream>
// #include "ctre/Phoenix.h"
// #include "Constants.h"
// #include "Launcher/VelocityTracking.h"
// #include "Robot.h"

// VelocityTracking::VelocityTracking() {
//     is_tracking = false;
// }

// void VelocityTracking::Track_Velocity() {
//     if (is_tracking == true) {
//         double targetVelocity_UnitsPer100Ms = TriggerAxis * 500.0 * 4096 / 600;

//         FX1.Set(ControlMode::Velocity, targetVelocity_UnitsPer100Ms);


//         _sb.append("\terrNative:");
//         _sb.append(std::to_string(FX1.GetClosedLoopError(kPIDLoopIdx)));
//         _sb.append("\ttrg:");
//     } else {
//          FX1.Set(ControlMode::PercentOutput, TriggerAxis);
//     }
// }

// bool VelocityTracking::isVelocityTracking() {
//     return is_tracking;
// }