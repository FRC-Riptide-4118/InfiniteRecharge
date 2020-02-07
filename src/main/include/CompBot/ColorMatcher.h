#include "rev/ColorSensorV3.h"
#include "rev/Colormatch.h"
#include "frc/smartdashboard/SmartDashboard.h"

class ColorMatcher {
    private:
        // Color creation
        static constexpr frc::Color kBlueTarget    = frc::Color(0.143, 0.427, 0.429);
        static constexpr frc::Color kGreenTarget   = frc::Color(0.197, 0.561, 0.240);
        static constexpr frc::Color kRedTarget     = frc::Color(0.561, 0.232, 0.114);
        static constexpr frc::Color kYellowTarget  = frc::Color(0.361, 0.524, 0.113);

        // Color sensor setup
        static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
        rev::ColorSensorV3 m_colorSensor{i2cPort};

        // Color matcher setup
        rev::ColorMatch m_colorMatcher;

        // Color detection variables
        frc::Color detectedColor;       // = m_colorSensor.GetColor();

        std::string colorString;
        double confidence = 0.0;

        frc::Color matchedColor;        // = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

    public:
        ColorMatcher();
        frc::Color getSeenColor();
        frc::Color getMatchedColor();
        std::string getColorString();
        void putDashboardTelemetry(bool);
        
};