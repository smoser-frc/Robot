// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  protected MotorControllerGroup m_leftMotors, m_rightMotors;

  // The robot's drive
  protected DifferentialDrive m_drive;

  // The gyro sensor
  protected Gyro m_gyro;

  protected boolean debug = false;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotors.setInverted(false);
    m_rightMotors.setInverted(true);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  @Override
  public void periodic() {
    if (debug) {
      SmartDashboard.putNumber("Drive Distance", getAverageEncoderDistance());
      SmartDashboard.putNumber("Turn Rate", getTurnRate());
      SmartDashboard.putNumber("Heading", getHeading());
    }
  }

  /**
   * Gets the average distance of the drive encoders.
   *
   * @return the average of the encoder readings
   */
  public double getAverageEncoderDistance() {
    throw new java.lang.UnsupportedOperationException();
  }

  public void resetEncoders() {
    throw new java.lang.UnsupportedOperationException();
  }
}
