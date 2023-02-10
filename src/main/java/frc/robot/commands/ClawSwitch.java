// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawSwitch extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Claw m_subsystem;

  private boolean isClosed;
  private boolean isOpen;
  private boolean startClosed;

  public ClawSwitch(Claw subsystem) {
    m_subsystem = subsystem;
    System.out.println("Ignore a warning about not using m_subsystem:" + m_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.checkLimits();
    isClosed = m_subsystem.queryClosed();
    isOpen = m_subsystem.queryOpen();

    if (isClosed) {
      m_subsystem.setMotorReverse();
      startClosed = true;
    } else {
      m_subsystem.setMotorForeward();
      startClosed = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.checkLimits();
    isClosed = m_subsystem.queryClosed();
    isOpen = m_subsystem.queryOpen();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isClosed && !startClosed) {
      return true;
    } else if (isOpen && startClosed) {
      return true;
    } else {
      return false;
    }
  }
}
