// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Follow YETI code: https://github.com/yeti-robotics/reefscape-2025/tree/develop/src/main/java/frc/robot/subsystems/led

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;

  
  public LEDSubsystem() {
    led = new AddressableLED(LEDConstants.LED_PWM_PORT);
    ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
    

    setDefaultCommand(runPattern(LEDPatterns.SCROLLING_RAINBOW).ignoringDisable(true));

    // Auto LED Pattern Call
    new Trigger(DriverStation::isAutonomousEnabled)
            .onTrue(runPattern(LEDPatterns.SCROLLING_RAINBOW));

    // Teleop LED Pattern Call
    new Trigger(DriverStation::isTeleopEnabled)
            .and(LEDSubsystem::isRedAlliance)
            .onTrue(runPattern(LEDPatterns.RED));
    new Trigger(DriverStation::isTeleopEnabled)
            .and(() -> !isRedAlliance())
            .onTrue(runPattern(LEDPatterns.BLUE));

    // Disabled LED Pattern Call
    new Trigger(DriverStation::isDisabled)
            .onTrue(runPattern(LEDPatterns.SCROLLING_RAINBOW));
  }

  @Override
  public void periodic() {
    led.setData(ledBuffer);
  }

  public void applyPattern(LEDPattern pattern) {
    pattern.applyTo(ledBuffer);
  }

  public Command runPattern(LEDPatterns pattern) {
    return runOnce(() -> applyPattern(pattern.pattern)).repeatedly().ignoringDisable(true);
  }

  private static boolean isRedAlliance() {
    return DriverStation.getAlliance()
            .filter(value -> value == DriverStation.Alliance.Red)
            .isPresent();
  }
}
