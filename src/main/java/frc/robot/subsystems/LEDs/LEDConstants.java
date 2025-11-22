// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;

public final class LEDConstants {
    public static final int LED_PWM_PORT = 4;
    public static final int LED_COUNT = 300;
    public static final Distance LED_SPACING = Meters.of(1 / 65.0);                         // 65 LEDs per meter
    public static final Color HEIST_YELLOW = new Color(255, 255, 255);       // Update color code
    public static final Color HEIST_GREEN = new Color(255, 255, 255);        // Update color code
}
