#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"


static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
rev::ColorSensorV3 m_colorSensor{i2cPort};
rev::ColorMatch m_colorMatcher;

static constexpr frc::Color kBlueTarget    = frc::Color(0.143, 0.427, 0.429);
static constexpr frc::Color kGreenTarget   = frc::Color(0.197, 0.561, 0.240);
static constexpr frc::Color kRedTarget     = frc::Color(0.561, 0.232, 0.114);
static constexpr frc::Color kYellowTarget  = frc::Color(0.361, 0.524, 0.113);

void ColorWheel::AddColors() {
    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kGreenTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);
    m_colorMatcher.AddColorMatch(kYellowTarget);
}

frc::Color ColorWheel::GetColor() {
    return m_colorSensor.GetColor();
}


String ColorWheel::GetColorDecision() {
    
    frc::Color color = this.GetColor();

    if(matchedColor == kBlueTarget) colorString = "Blue";
    else if(matchedColor == kGreenTarget) colorString = "Green";
    else if(matchedColor == kRedTarget) colorString = "Red";
    else if(matchedColor == kYellowTarget) colorString = "Yellow";
    else colorString = "Defaulted. oops :/";

    return colorString;
}


double ColorWheel::GetIRValue() {
    return m_colorSensor.GetIR();
}
