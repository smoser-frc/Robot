// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  protected MotorControllerGroup m_leftMotors, m_rightMotors;

  // The robot's drive
  protected DifferentialDrive m_drive;

  DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(DriveConstants.kTrackwidthMeters);

  // The gyro sensor
  protected Gyro m_gyro;

  protected DifferentialDriveOdometry m_odometry;
  protected Field2d m_fieldSim;

  protected boolean debug = false;

  public void _init() {
    m_leftMotors.setInverted(false);
    m_rightMotors.setInverted(true);
    m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    m_odometry =
        new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(getHeading()), getLeftDistance(), getRightDistance());
    m_fieldSim = new Field2d();

    SmartDashboard.putData("Field", m_fieldSim);
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

  /*
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  @Override
  public void periodic() {
    if (debug) {
      SmartDashboard.putNumber("Drive Distance", getAverageEncoderDistance());
      SmartDashboard.putNumber("Turn Rate", getTurnRate());
      SmartDashboard.putNumber("Heading", getHeading());
    }
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftDistance(), getRightDistance());

    m_fieldSim.setRobotPose(getPose());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetPosition() {
    resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    if (Robot.isSimulation()) {
      setSimPose(pose);
    }
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getHeading()), getLeftDistance(), getRightDistance(), pose);
  }

  public void setSimPose(Pose2d pose) {}

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

  public double getLeftDistance() {
    throw new java.lang.UnsupportedOperationException();
  }

  public double getRightDistance() {
    throw new java.lang.UnsupportedOperationException();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
  }

  public double getLeftSpeed() {
    throw new java.lang.UnsupportedOperationException();
  }

  public double getRightSpeed() {
    throw new java.lang.UnsupportedOperationException();
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    // These are taken from picture at
    // https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/characterizing-drive.html
    // oh how we need sysid.
    double ks = 0.52269, kv = 2.4021, ka = 0.43354;

    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              // Reset odometry for the first path you run during auto
              if (isFirstPath) {
                this.resetOdometry(traj.getInitialPose());
              }
            }),
        new PPRamseteCommand(
            traj,
            this::getPose, // Pose supplier,
            new RamseteController(),
            new SimpleMotorFeedforward(ks, kv, ka),
            this.kinematics, // DifferentialDriveKinematics
            this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            // left and right pid controllers
            new PIDController(
                DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD),
            new PIDController(
                DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD),
            this::tankDriveVolts, // Voltage biconsumer
            true, // Should the path be automatically mirrored depending on alliance color.
            // Optional, defaults to true
            this // Requires this drive subsystem
            ));
  }
}
