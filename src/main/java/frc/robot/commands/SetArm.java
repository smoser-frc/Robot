// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RealConstants;
import frc.robot.subsystems.Arm;

/** A command that will move the robot forward */
public class SetArm extends CommandBase {

  private Arm m_arm;
  private PIDController m_PidControl;
  private double m_position;

  /**
   * Moves the robot a specified distance forward.
   *
   * @param distance Distance in inches to drive.
   * @param drive The drive subsystem to use
   */
  public SetArm(double position, Arm arm) {
    m_arm = arm;
    m_position = position;

    SmartDashboard.putNumber("Arm P", 0);
    SmartDashboard.putNumber("Arm I", 0);
    SmartDashboard.putNumber("Arm D", 0);

    addRequirements(arm);
  }

  @Override
  public void initialize() {

    double p = SmartDashboard.getNumber("Arn P", 0);
    double i = SmartDashboard.getNumber("Arm I", 0);
    double d = SmartDashboard.getNumber("Arm D", 0);

    m_PidControl = new PIDController(0.01, 0, 0);
    m_PidControl.setTolerance(RealConstants.kArmPositionToleranceDegrees);

    System.out.println("At " + m_arm.getPosition() + " going to " + m_position);

    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
  }

  @Override
  public void execute() {
    double pidSpeed = m_PidControl.calculate(m_arm.getPosition(), m_position);
    m_arm.setMotor(pidSpeed);
    SmartDashboard.putNumber("ArmPid", pidSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("At " + m_arm.getPosition() + " target " + m_position + " Done!");
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return m_PidControl.atSetpoint();
  }
}
