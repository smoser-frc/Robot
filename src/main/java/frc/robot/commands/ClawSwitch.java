// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawSwitch extends CommandBase {
  private final Claw m_claw;

  private boolean isClosed;
  private boolean isOpen;
  private boolean startClosed;

  public ClawSwitch(Claw subsystem) {
    m_claw = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isClosed = m_claw.queryClosed();
    isOpen = m_claw.queryOpen();

    if (isClosed) {
      m_claw.setMotorReverse();
      startClosed = true;
    } else {
      // FIXME: m_claw.setMotorForward();
      startClosed = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isClosed = m_claw.queryClosed();
    isOpen = m_claw.queryOpen();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.stopMotor();
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
