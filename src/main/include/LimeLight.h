// #pragma once
// #include "networktables/NetworkTable.h"
// #include "networktables/NetworkTableInstance.h"
// #include "Robot.h"
// #include "Interactions.h"
// #include <frc/GenericHID.h>
// #include <frc/XboxController.h>
// #include <frc/drive/DifferentialDrive.h>

// class VisionTracking {
//     private:
//         float Kp;
//         float min_command;
//         double tx; 
//         frc::XboxController Controller1;
//         frc::DifferentialDrive drive;
//         std::shared_ptr<NetworkTable> table;
//     public:
//         VisionTracking(frc::XboxController *controller1,
//             frc::DifferentialDrive drive);
//         void toggleCameraModes();
//         void adjustToTarget();
// };