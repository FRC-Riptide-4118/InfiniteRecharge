#include <iostream>
#include "CompBot/Intake/Pneumatic_Intake.h"
#include "frc/DoubleSolenoid.h"

Pneumatic_Intake::Pneumatic_Intake() {
    // Empty constructor?
}


bool Pneumatic_Intake::isDeployed() {
    return deployed;
}

void Pneumatic_Intake::deployIntake() {
    if (deployed) {
        intakeDeploy->Set(frc::DoubleSolenoid::kForward);
    } else {
        intakeDeploy->Set(frc::DoubleSolenoid::kReverse);
    }

}

void Pneumatic_Intake::DefaultIntakePneumatics() {
    deployed = false;
    intakeDeploy = new frc::DoubleSolenoid( 1, 0, 1);

}
