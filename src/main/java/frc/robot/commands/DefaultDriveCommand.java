// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultDriveCommand extends Command {
  /** Creates a new DefaultDriveCommand. */
  
  private DrivetrainSubsystem DRIVETRAIN_SUBSYSTEM;
  private XboxController logitechController;

  double forward;
  double rotation;
  
  public DefaultDriveCommand(DrivetrainSubsystem drive, XboxController logitech) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DRIVETRAIN_SUBSYSTEM = drive;
    this.logitechController = logitech;

    addRequirements(DRIVETRAIN_SUBSYSTEM);    // If any other command is using the drivetrain subsystem, stop that command and use this one
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    forward = 0;
    rotation = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    forward = logitechController.getLeftY();
    rotation = logitechController.getRightX();

    DRIVETRAIN_SUBSYSTEM.set(forward, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DRIVETRAIN_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
