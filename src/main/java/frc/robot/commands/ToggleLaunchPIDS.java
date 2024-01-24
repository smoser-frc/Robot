// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class ToggleLaunchPIDS extends Command {
  /** Creates a new ToggleLaunchPIDS. */
  private Launcher m_launcher;

  public ToggleLaunchPIDS(Launcher launcher) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_launcher = launcher;
    addRequirements(launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_launcher.togglePIDTuning();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
