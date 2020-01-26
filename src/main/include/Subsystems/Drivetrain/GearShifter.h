#pragma once
#include <frc/DoubleSolenoid.h>
#include <iostream>

class GearShifter {
  private:
      bool high_gear;
      frc::DoubleSolenoid *shifter;
  public:  
      GearShifter();
      bool ishighgear();
      void shiftgear();
};