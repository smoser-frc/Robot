// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;

public class PrimeIndex extends Command {
  private final Index index;
  private boolean inUpper;

  /**
   * Creates a new PrimeIndex.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PrimeIndex(Index index) {
    this.index = index;
    inUpper = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Index is initializing");
    index.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!inUpper && index.isInUpper()) {
      inUpper = true;
      index.setLower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    index.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return index.isPrimed();
  }
}
