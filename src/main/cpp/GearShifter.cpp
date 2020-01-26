#include <frc/DoubleSolenoid.h>
#include <iostream>
#include "Subsystems/Drivetrain/GearShifter.h"

bool GearShifter::isHighGear() {
    return high_gear;
}

void GearShifter::shiftGear() {
    if (high_gear) { 
        shifter->Set(frc::DoubleSolenoid::kForward);
    } else { 
        shifter->Set(frc::DoubleSolenoid::kReverse);
    }

}

GearShifter::GearShifter() {
    high_gear = false;
    shifter = new frc::DoubleSolenoid( 0, 0, 1 );
}
