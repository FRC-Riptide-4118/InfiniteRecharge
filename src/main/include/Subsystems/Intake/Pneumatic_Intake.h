#include <iostream>
#include "frc/DoubleSolenoid.h"

class Pneumatic_Intake {
    private:
        bool deployed;
        frc::DoubleSolenoid *intakeDeploy;
        
    public:
        Pneumatic_Intake();
        bool isDeployed();
        void deployIntake();

        void DefaultIntakePneumatics();        
};