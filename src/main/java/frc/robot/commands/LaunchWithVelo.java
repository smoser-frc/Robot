// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class LaunchWithVelo extends Command {
  /** Creates a new LaunchWithVelo. */
  private Launcher launcher;

  private double launchVelo;

  public LaunchWithVelo(Launcher launcher, double velocity) {
    this.launcher = launcher;
    launchVelo = velocity;
    addRequirements(launcher);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcher.setLaunchVelocity(launchVelo);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (launcher.readyToLaunch(launchVelo)) {
      // FIXME run index here (no subsystem at time of development)
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcher.setLaunchVelocity(0);
    // FIXME stop index here (see above)
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
