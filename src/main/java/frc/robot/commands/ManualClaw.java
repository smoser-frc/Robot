// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import java.util.function.DoubleSupplier;

public class ManualClaw extends CommandBase {
  private final Claw m_claw;
  private DoubleSupplier m_speed;

  public ManualClaw(Claw subsystem, DoubleSupplier speed) {
    m_claw = subsystem;
    m_speed = speed;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double position = m_claw.getPosition();
    double power = m_speed.getAsDouble();
    if (power <= 0.1 && power >= -0.1) {
      m_claw.setMotor(0);
    } else {
      m_claw.setMotor(power);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
