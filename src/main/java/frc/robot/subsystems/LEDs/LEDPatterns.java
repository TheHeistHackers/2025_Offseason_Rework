// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public enum LEDPatterns {
    // Various LED Patterns to be called in LEDSubsystem
    RAINBOW(LEDPattern.rainbow(255, 128)),
    SCROLLING_RAINBOW(RAINBOW.pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(5), LEDConstants.LED_SPACING)),
    RED(LEDPattern.solid(new Color(0, 255, 0))),    // GRB
    BLUE(LEDPattern.solid(new Color(0, 0, 255)));   // GRB

    // An instance of this enum
    public final LEDPattern pattern;

    // The constructor
    LEDPatterns(LEDPattern pattern) {
        this.pattern = pattern;
    }

}
