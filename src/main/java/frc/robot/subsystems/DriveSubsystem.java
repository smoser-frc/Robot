// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private CANSparkMax leftFront, leftBack, rightFront, rightBack;
  private final MotorControllerGroup m_leftMotors, m_rightMotors;

  // The robot's drive
  private final DifferentialDrive m_drive;

  // The left-side drive encoder
  private final RelativeEncoder m_leftFrontEncoder,
      m_leftBackEncoder,
      m_rightFrontEncoder,
      m_rightBackEncoder;

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SerialPort.Port.kMXP);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    leftFront = new CANSparkMax(1, MotorType.kBrushless);
    leftBack = new CANSparkMax(2, MotorType.kBrushless);
    rightFront = new CANSparkMax(3, MotorType.kBrushless);
    rightBack = new CANSparkMax(4, MotorType.kBrushless);

    m_leftMotors = new MotorControllerGroup(leftFront, leftBack);
    m_rightMotors = new MotorControllerGroup(rightFront, rightBack);

    m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    m_leftFrontEncoder = leftFront.getEncoder();
    m_leftBackEncoder = leftBack.getEncoder();
    m_rightFrontEncoder = leftFront.getEncoder();
    m_rightBackEncoder = leftBack.getEncoder();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotors.setInverted(false);
    m_rightMotors.setInverted(true);

    // Sets the distance per pulse for the encoders
    m_leftFrontEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_leftBackEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_rightFrontEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_rightBackEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
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

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftFrontEncoder.setPosition(0);
    m_leftBackEncoder.setPosition(0);
    m_rightFrontEncoder.setPosition(0);
    m_rightBackEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftFrontEncoder.getPosition()
            + m_leftBackEncoder.getPosition()
            + m_rightFrontEncoder.getPosition()
            + m_rightBackEncoder.getPosition())
        / 4.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public RelativeEncoder getLeftEncoder() {
    return m_leftFrontEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public RelativeEncoder getRightEncoder() {
    return m_rightFrontEncoder;
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
    SmartDashboard.putNumber("Drive Distance", getAverageEncoderDistance());
    SmartDashboard.putNumber("Turn Rate", getTurnRate());
    SmartDashboard.putNumber("Heading", getHeading());
  }
}
