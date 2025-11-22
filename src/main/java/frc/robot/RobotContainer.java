// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.RollCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  // Subsystems
  private final DrivetrainSubsystem drivetrain;
  private final RollerSubsystem roller;
  private final LEDSubsystem leds;

  // Controller
  public final CommandXboxController logitech;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain = new DrivetrainSubsystem();
    roller = new RollerSubsystem();
    leds = new LEDSubsystem();

    logitech = new CommandXboxController(Constants.OperatorConstants.logitechControllerPort);

    configureBindings();
    defaultCommands();
  }

  private void configureBindings() {
    logitech.rightBumper().whileTrue(new RollCommand(roller, 1.0));
  }

  private void defaultCommands(){
    drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, logitech));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
