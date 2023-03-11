// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RealConstants;
import java.util.function.DoubleSupplier;

public class RealDrive extends Drive {

  private CANSparkMax leftFront = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax leftMid = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax leftBack = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax rightFront = new CANSparkMax(4, MotorType.kBrushless);
  private CANSparkMax rightMid = new CANSparkMax(5, MotorType.kBrushless);
  private CANSparkMax rightBack = new CANSparkMax(6, MotorType.kBrushless);

  private double leftZero = 0;
  private double rightZero = 0;

  private MotorControllerGroup leftGroup = new MotorControllerGroup(leftBack, leftMid, leftFront);
  private MotorControllerGroup rightGroup =
      new MotorControllerGroup(rightFront, rightMid, rightBack);

  private RelativeEncoder leftEnc = leftFront.getEncoder();
  private RelativeEncoder rightEnc = rightFront.getEncoder();

  private final AHRS m_gyro = new AHRS(SerialPort.Port.kMXP);

  private DifferentialDriveOdometry m_odometry =
      new DifferentialDriveOdometry(
          m_gyro.getRotation2d(), leftEnc.getPosition(), rightEnc.getPosition());

  private DifferentialDrive driveTrain = new DifferentialDrive(leftGroup, rightGroup);

  public RealDrive() {
    leftGroup.setInverted(true);
    rightGroup.setInverted(false);

    setCoastMode();

    leftEnc.setPositionConversionFactor(RealConstants.kMetersPerRev);
    rightEnc.setPositionConversionFactor(RealConstants.kMetersPerRev);
  }

  @Override
  public void setTankDrive(DoubleSupplier lSpeed, DoubleSupplier rSpeed, double pOutput) {

    driveTrain.tankDrive(lSpeed.getAsDouble() * pOutput, rSpeed.getAsDouble() * pOutput);
  }

  @Override
  public void setBrakeMode(){
    leftFront.setIdleMode(IdleMode.kBrake);
    leftMid.setIdleMode(IdleMode.kBrake);
    leftBack.setIdleMode(IdleMode.kBrake);
    rightFront.setIdleMode(IdleMode.kBrake);
    rightMid.setIdleMode(IdleMode.kBrake);
    rightBack.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void setCoastMode(){
    leftFront.setIdleMode(IdleMode.kCoast);
    leftMid.setIdleMode(IdleMode.kCoast);
    leftBack.setIdleMode(IdleMode.kCoast);
    rightFront.setIdleMode(IdleMode.kCoast);
    rightMid.setIdleMode(IdleMode.kCoast);
    rightBack.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void setArcadeDrive(double speed, double rotation) {
    driveTrain.arcadeDrive(speed, rotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(m_gyro.getRotation2d(), leftEnc.getPosition(), rightEnc.getPosition());

    SmartDashboard.putNumber("Left Drive Position", leftEnc.getPosition());
    SmartDashboard.putNumber(" L Conversion", leftEnc.getPositionConversionFactor());
    SmartDashboard.putNumber("Right Drive Position", rightEnc.getPosition());
    SmartDashboard.putNumber("ConversionFactor R", rightEnc.getPositionConversionFactor());
    SmartDashboard.putNumber("Drive Position", getAverageEncoderDistance());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  @Override
  public void resetOdometry(Pose2d pose) {
    zeroEncoders();
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), leftEnc.getPosition(), rightEnc.getPosition(), pose);
  }

  @Override
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftGroup.setVoltage(leftVolts);
    rightGroup.setVoltage(rightVolts);
    driveTrain.feed();
  }

  @Override
  public void zeroEncoders() {
    leftEnc.setPosition(0);
    rightEnc.setPosition(0);
  }

  @Override
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftEnc.getVelocity() * RealConstants.speedConversionFactor,
        rightEnc.getVelocity() * RealConstants.speedConversionFactor);
  }

  @Override
  public void zeroHeading() {
    m_gyro.reset();
  }

  @Override
  public double getHeading() {
    return m_gyro.getAngle();
  }

  @Override
  public double getTurnRate() {
    return m_gyro.getRate();
  }

  @Override
  public double getAverageEncoderDistance() {
    return (-leftEnc.getPosition() + rightEnc.getPosition()) / 2;
  }
}
