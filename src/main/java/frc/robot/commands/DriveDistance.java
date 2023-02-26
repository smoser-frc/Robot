// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/** A command that will move the robot forward */
public class DriveDistance extends CommandBase {

  private DriveSubsystem m_drive;
  private PIDController m_PidControl;
  private double m_distance;
  private double setPoint;

  /**
   * Moves the robot a specified distance forward.
   *
   * @param distance Distance in inches to drive.
   * @param drive The drive subsystem to use
   */
  public DriveDistance(double distance, DriveSubsystem drive) {
    m_drive = drive;
    m_distance = distance;

    m_PidControl =
        new PIDController(DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD);
    m_PidControl.setTolerance(
        Constants.DriveConstants.kDriveToleranceMeters,
        Constants.DriveConstants.kDriveRateToleranceMetersPerS);

    addRequirements(drive);
  }

  @Override
  public void initialize() {

    double target = m_distance + m_drive.getAverageEncoderDistance();
    System.out.println("At " + m_drive.getAverageEncoderDistance() + " going to " + target);

    setPoint = m_drive.getAverageEncoderDistance() + m_distance;

    m_drive.setMaxOutput(1.0);

    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
  }

  @Override
  public void execute() {
    m_drive.arcadeDrive(m_PidControl.calculate(m_drive.getAverageEncoderDistance(), setPoint), 0);
    SmartDashboard.putNumber(
        "Wheel Speed via PID",
        m_PidControl.calculate(m_drive.getAverageEncoderDistance(), setPoint));
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.setMaxOutput(1);
    System.out.println(
        "At " + m_drive.getAverageEncoderDistance() + " target " + setPoint + " Done!");
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return m_PidControl.atSetpoint();
  }
}
