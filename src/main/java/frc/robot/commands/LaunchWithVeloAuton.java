// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Launcher;

public class LaunchWithVeloAuton extends Command {
  /** Creates a new LaunchWithVelo. */
  private Launcher launcher;

  private Index index;

  private double launchVelo;

  private Timer switchTimer;

  public LaunchWithVeloAuton(Launcher launcher, Index index, double velocity) {
    this.launcher = launcher;
    this.index = index;
    launchVelo = velocity;
    addRequirements(launcher);
    switchTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Launcher is initializing");
    launcher.setLaunchVelocity(launchVelo);
    switchTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (launcher.readyToLaunch(launchVelo)) {
      index.start();
      switchTimer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      launcher.setLaunchVelocity(0);
      index.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Only called on button hold and done when button released
    return switchTimer.hasElapsed(.5);
  }
}
