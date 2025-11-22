// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

  private final LEDPattern m_rainbow;
  private static final Distance kLedSpacing = Meters.of(1/60.0);  // 60 LEDs per meter
  private final LEDPattern m_scrollingRainbow;
  
  public LEDSubsystem() {
    m_rainbow = LEDPattern.rainbow(255, 128);
    m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(5), kLedSpacing);  // 5 meters per second

    m_led = new AddressableLED(Constants.LEDConstants.LEDPort);
    m_ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.LED_len);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();
    

    setDefaultCommand(runPattern(m_scrollingRainbow));
  }

  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display
    m_scrollingRainbow.applyTo(m_ledBuffer);    // Forces rainbow pattern to constantly run
    m_led.setData(m_ledBuffer);
  }

  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_ledBuffer));
  }
}
