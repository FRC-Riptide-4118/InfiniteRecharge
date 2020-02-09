// #include "LimeLight.h"



// VisionTracking::VisionTracking(frc::XboxController *controller1, frc::DifferentialDrive drive) {
//         Controller1 = controller1;
//         Kp  = -0.1;
//         min_command = 0.5;
//         table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
//         tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
// }

// void VisionTracking::toggleCameraModes() {
//         if (Controller1->GetStartButton()) {
//            table->PutNumber("camMode", !table->GetNumber("camMode", 0));
//            table->PutNumber("ledMode", !table->GetNumber("ledMode", 0));
//     }
// }

// void VisionTracking::adjustToTarget() {
    
//     if (Controller1->GetBumper(frc::GenericHID::GenericHID::JoystickHand::kRightHand)) {

//         double   heading_error = -tx;
//         double   steering_adjust = 0.0;

//             if (tx > 1.0) {
//                 steering_adjust = Kp*heading_error - min_command;
//             } else if (tx < 1.0) {
//                 steering_adjust = Kp*heading_error + min_command;
//             }

//             drive.ArcadeDrive( 0, steering_adjust, 1 );
//     }
// }   