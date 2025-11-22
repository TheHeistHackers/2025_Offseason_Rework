// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DrivetrainSubsystem;


public class DefaultDriveCommand extends Command {
  
  private DrivetrainSubsystem DRIVETRAIN_SUBSYSTEM;
  private CommandXboxController logitechController;

  double forward;
  double rotation;
  
  public DefaultDriveCommand(DrivetrainSubsystem drive, CommandXboxController logitech) {
    this.DRIVETRAIN_SUBSYSTEM = drive;
    this.logitechController = logitech;

    addRequirements(DRIVETRAIN_SUBSYSTEM);    // If any other command is using the drivetrain subsystem, stop that command and use this one
  }

  @Override
  public void initialize() {
    forward = 0;
    rotation = 0;
  }

  @Override
  public void execute() {
    forward = -logitechController.getLeftY();
    rotation = logitechController.getRightX();

    DRIVETRAIN_SUBSYSTEM.set(forward, rotation);
  }

  @Override
  public void end(boolean interrupted) {
    DRIVETRAIN_SUBSYSTEM.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
