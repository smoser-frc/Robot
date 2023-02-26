// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants.DriveConstants;

public class RealDrive extends DriveSubsystem {
  private CANSparkMax leftFront, leftMid, leftBack, rightFront, rightMid, rightBack;
  private final RelativeEncoder m_leftFrontEncoder,
      m_leftBackEncoder,
      m_rightFrontEncoder,
      m_rightBackEncoder;

  protected boolean six = true;

  /** Creates a new RealDrive. */
  public RealDrive() {
    this(false, false);
  }

  public RealDrive(boolean six, boolean debug) {
    this.debug = debug;
    this.six = six;

    if (six) {
      leftFront = new CANSparkMax(1, MotorType.kBrushless);
      leftMid = new CANSparkMax(2, MotorType.kBrushless);
      leftBack = new CANSparkMax(3, MotorType.kBrushless);
      rightFront = new CANSparkMax(4, MotorType.kBrushless);
      rightMid = new CANSparkMax(5, MotorType.kBrushless);
      rightBack = new CANSparkMax(6, MotorType.kBrushless);
      m_leftMotors = new MotorControllerGroup(leftFront, leftMid, leftBack);
      m_rightMotors = new MotorControllerGroup(rightFront, rightMid, rightBack);
    } else {
      leftFront = new CANSparkMax(1, MotorType.kBrushless);
      leftBack = new CANSparkMax(2, MotorType.kBrushless);
      rightFront = new CANSparkMax(3, MotorType.kBrushless);
      rightBack = new CANSparkMax(4, MotorType.kBrushless);
      m_leftMotors = new MotorControllerGroup(leftFront, leftBack);
      m_rightMotors = new MotorControllerGroup(rightFront, rightBack);
    }
    m_gyro = new AHRS(SerialPort.Port.kMXP);
    System.out.println("Yeah, set gyyro to" + m_gyro);

    m_leftFrontEncoder = leftFront.getEncoder();
    m_leftBackEncoder = leftBack.getEncoder();
    m_rightFrontEncoder = leftFront.getEncoder();
    m_rightBackEncoder = leftBack.getEncoder();

    // Sets the distance per pulse for the encoders
    m_leftFrontEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_leftBackEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_rightFrontEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_rightBackEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);

    _init();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  @Override
  public void resetEncoders() {
    m_leftFrontEncoder.setPosition(0);
    m_leftBackEncoder.setPosition(0);
    m_rightFrontEncoder.setPosition(0);
    m_rightBackEncoder.setPosition(0);
  }

  @Override
  public double getAverageEncoderDistance() {
    return (m_leftFrontEncoder.getPosition()
            + m_leftBackEncoder.getPosition()
            + m_rightFrontEncoder.getPosition()
            + m_rightBackEncoder.getPosition())
        / 4.0;
  }

  @Override
  public double getLeftDistance() {
    return (m_leftFrontEncoder.getPosition() + m_leftBackEncoder.getPosition()) / 2;
  }

  @Override
  public double getRightDistance() {
    return (m_rightFrontEncoder.getPosition() + m_rightBackEncoder.getPosition()) / 2;
  }
}
