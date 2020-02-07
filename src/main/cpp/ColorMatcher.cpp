#include "rev/ColorSensorV3.h"
#include "rev/Colormatch.h"
#include "CompBot/ColorMatcher.h"

// Constructors
ColorMatcher::ColorMatcher() {
    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kGreenTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);
    m_colorMatcher.AddColorMatch(kYellowTarget);
}


// Color decision methods
frc::Color ColorMatcher::getSeenColor() {
    detectedColor = m_colorSensor.GetColor();
    return detectedColor;
}

frc::Color ColorMatcher::getMatchedColor() {
    matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);
    return matchedColor;
}

std::string ColorMatcher::getColorString() {
    if (matchedColor == kBlueTarget) {
        colorString = "Blue";
    } else if (matchedColor == kRedTarget) {
        colorString = "Red";
    } else if (matchedColor == kGreenTarget) {
        colorString = "Green";
    } else if (matchedColor == kYellowTarget) {
        colorString = "Yellow";
    } else {
        colorString = "Unknown";
    }

    return colorString;
}


void ColorMatcher::putDashboardTelemetry(bool putColorString) {
    frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    frc::SmartDashboard::PutNumber("Green", detectedColor.green);
    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);

    if(putColorString) {
        frc::SmartDashboard::PutString("Color Decision", colorString);
    }
}