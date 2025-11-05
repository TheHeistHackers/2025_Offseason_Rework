// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RollCommand extends Command {
  /** Creates a new RollCommand. */

  private RollerSubsystem ROLLER_SUBSYSTEM;

  double speed;

  public RollCommand(RollerSubsystem roll, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ROLLER_SUBSYSTEM = roll;
    this.speed = speed;

    addRequirements(ROLLER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ROLLER_SUBSYSTEM.set(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ROLLER_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
