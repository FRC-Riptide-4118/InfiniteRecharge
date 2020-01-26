#include <frc/DoubleSolenoid.h>
#include <iostream>
#include "CompBot/Drivetrain/GearShifter.h"


GearShifter::GearShifter() {
    high_gear = false;
    shifter = new frc::DoubleSolenoid( 0, 0, 1 );
}

void GearShifter::ShiftGear() {
    if (high_gear) { 
        shifter->Set(frc::DoubleSolenoid::kForward);
    } else { 
        shifter->Set(frc::DoubleSolenoid::kReverse);
    }
}

bool GearShifter::IsHighGear() {
    return high_gear;
}
