// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollerSubsystem;

public class RollCommand extends Command {

  private RollerSubsystem ROLLER_SUBSYSTEM;

  double speed;

  public RollCommand(RollerSubsystem roll, double speed) {
    this.ROLLER_SUBSYSTEM = roll;
    this.speed = speed;

    addRequirements(ROLLER_SUBSYSTEM);
  }

  @Override
  public void initialize() {
    speed = 0;
  }

  @Override
  public void execute() {
    ROLLER_SUBSYSTEM.set(speed);
  }

  @Override
  public void end(boolean interrupted) {
    ROLLER_SUBSYSTEM.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
