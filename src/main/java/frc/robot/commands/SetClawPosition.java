// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class SetClawPosition extends CommandBase {
  /** Creates a new SetArmPosition. */
  private double m_position;

  private Claw m_claw;
  private PIDController m_Controller;

  public SetClawPosition(Claw claw, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_position = position;
    m_claw = claw;
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double kG = SmartDashboard.getNumber("claw kG", 0);
    double kV = SmartDashboard.getNumber("claw kV", 0);
    double kP = SmartDashboard.getNumber("claw kP", 0);
    double kI = SmartDashboard.getNumber("claw kI", 0);
    double kD = SmartDashboard.getNumber("claw kD", 0);

    m_Controller = new PIDController(kP, kI, kD);

    m_Controller.setTolerance(2, 2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_Controller.calculate(m_claw.getPosition(), m_position);
    m_claw.setMotorVolts(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Controller.atSetpoint();
  }
}
