// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.Constants.DriveConstants;

public class SimDrive extends DriveSubsystem {
  // The left-side drive encoder
  Encoder m_leftEncoder, m_rightEncoder;

  // These classes help us simulate our drivetrain
  public DifferentialDrivetrainSim m_drivetrainSimulator;
  private final EncoderSim m_leftEncoderSim, m_rightEncoderSim;

  // The Field2d class shows the field in the sim GUI
  private final ADXRS450_GyroSim m_gyroSim;

  /** Creates a new SimDrive. */
  public SimDrive() {
    // The gyro sensor
    m_gyro = new ADXRS450_Gyro();

    // The motors on the left side of the drive.
    m_leftMotors =
        new MotorControllerGroup(
            new PWMSparkMax(SimDriveConstants.kLeftMotor1Port),
            new PWMSparkMax(SimDriveConstants.kLeftMotor2Port));

    // The motors on the right side of the drive.
    m_rightMotors =
        new MotorControllerGroup(
            new PWMSparkMax(SimDriveConstants.kRightMotor1Port),
            new PWMSparkMax(SimDriveConstants.kRightMotor2Port));

    m_leftEncoder =
        new Encoder(SimDriveConstants.kLeftEncoderPorts[0], SimDriveConstants.kLeftEncoderPorts[1]);

    m_rightEncoder =
        new Encoder(
            SimDriveConstants.kRightEncoderPorts[0], SimDriveConstants.kRightEncoderPorts[1]);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(SimDriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(SimDriveConstants.kEncoderDistancePerPulse);

    if (RobotBase.isSimulation()) { // If our robot is simulated
      // This class simulates our drivetrain's motion around the field.
      m_drivetrainSimulator =
          new DifferentialDrivetrainSim(
              SimDriveConstants.kDrivetrainPlant,
              SimDriveConstants.kDriveGearbox,
              DriveConstants.kGearRatioHigh,
              DriveConstants.kTrackwidthMeters,
              DriveConstants.kWheelDiameterMeters / 2.0,
              VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

      // The encoder and gyro angle sims let us set simulated sensor readings
      m_leftEncoderSim = new EncoderSim(m_leftEncoder);
      m_rightEncoderSim = new EncoderSim(m_rightEncoder);
      m_gyroSim = new ADXRS450_GyroSim((ADXRS450_Gyro) m_gyro);
    } else {
      m_leftEncoderSim = null;
      m_rightEncoderSim = null;
      m_gyroSim = null;
    }

    _init();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  @Override
  public void setSimPose(Pose2d pose) {
    m_drivetrainSimulator.setPose(pose);
  }

  @Override
  public double getLeftDistance() {
    return m_leftEncoder.getDistance();
  }

  @Override
  public double getRightDistance() {
    return m_rightEncoder.getDistance();
  }

  public double getLeftSpeed() {
    return m_leftEncoder.getRate();
  }

  public double getRightSpeed() {
    return m_rightEncoder.getRate();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    m_drivetrainSimulator.setInputs(
        m_leftMotors.get() * RobotController.getBatteryVoltage(),
        m_rightMotors.get() * RobotController.getBatteryVoltage());
    m_drivetrainSimulator.update(0.020);

    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  public static final class SimDriveConstants {

    public static final int kLeftMotor1Port = 0;
    public static final int kLeftMotor2Port = 1;
    public static final int kRightMotor1Port = 2;
    public static final int kRightMotor2Port = 3;

    public static final int[] kLeftEncoderPorts = new int[] {0, 1};
    public static final int[] kRightEncoderPorts = new int[] {2, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = false;

    public static final double kEncoderCPR = 1024;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (DriveConstants.kWheelDiameterMeters * Math.PI)
            / DriveConstants.kGearRatioHigh
            / (double) kEncoderCPR;
    // These are taken from StateSpaceDriveSimulation example.

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // These two values are "angular" kV and kA
    public static final double kvVoltSecondsPerRadian = 1.5;
    public static final double kaVoltSecondsSquaredPerRadian = 0.3;

    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
        LinearSystemId.identifyDrivetrainSystem(
            kvVoltSecondsPerMeter,
            kaVoltSecondsSquaredPerMeter,
            kvVoltSecondsPerRadian,
            kaVoltSecondsSquaredPerRadian);

    // Example values only -- use what's on your physical robot!
    public static final DCMotor kDriveGearbox = DCMotor.getCIM(2);

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;
  }
}
