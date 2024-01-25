// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class SwitchLaunchAngle extends Command {
  /** Creates a new SwitchLaunchAngle. */
  private Launcher m_launcher;

  private Timer timer;
  private boolean isForward;

  public SwitchLaunchAngle(Launcher launcher) {
    m_launcher = launcher;

    addRequirements(launcher);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_launcher.getAnglePosition() == Value.kForward) {
      isForward = true;
    } else {
      isForward = false;
    }

    timer = new Timer();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_launcher.setAngle(isForward ? Value.kReverse : Value.kForward);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.hasElapsed(1));
  }
}
