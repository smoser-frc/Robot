// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class RunClimbWinch extends Command {
  /** Creates a new RunClimbWinch. */
  private Climb climb;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.runWinch();
  }

  public RunClimbWinch(Climb climb) {
    this.climb = climb;
    addRequirements(climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // This code isn't necessary, but we're keeping it to make sure
  // that it doesn't past the end of the winch.
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Calling clime.stopWinch isn't necessary, but we're keeping it to make sure
    // that it doesn't past the end of the winch.
    climb.stopWinch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climb.winchLimitIsReached();
  }
}
