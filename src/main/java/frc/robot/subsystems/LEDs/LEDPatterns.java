// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.LEDPattern;

public enum LEDPatterns {

    // Enum list
    RAINBOW(LEDPattern.rainbow(255, 128)),
    SCROLLING_RAINBOW(RAINBOW.pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(5), LEDConstants.LED_SPACING));

    // An instance of this enum
    public final LEDPattern pattern;

    // The constructor
    LEDPatterns(LEDPattern pattern) {
        this.pattern = pattern;
    }

}
