// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {

  private static CANSparkMax leftFrontDriveMotor;
  private static CANSparkMax leftFrontTurnMotor;
  private static CANSparkMax rightFrontDriveMotor;
  private static CANSparkMax rightFrontTurnMotor;
  private static CANSparkMax leftBackDriveMotor;
  private static CANSparkMax leftBackTurnMotor;
  private static CANSparkMax rightBackDriveMotor;
  private static CANSparkMax rightBackTurnMotor;

  private static SwerveDriveWheel leftFrontWheel;
  private static SwerveDriveWheel rightFrontWheel;
  private static SwerveDriveWheel leftBackWheel;
  private static SwerveDriveWheel rightBackWheel;

  private static SwerveDriveCoordinator swerveCoordinator;

  private static CANcoder leftFrontTurnEncoder;
  private static CANcoder leftBackTurnEncoder;
  private static CANcoder rightFrontTurnEncoder;
  private static CANcoder rightBackTurnEncoder;

  private static RelativeEncoder leftFrontDriveEncoder;
  private static RelativeEncoder leftBackDriveEncoder;
  private static RelativeEncoder rightFrontDriveEncoder;
  private static RelativeEncoder rightBackDriveEncoder;

  private static AHRS gyro;

  public Drive() {
    leftFrontDriveMotor = new CANSparkMax(0, MotorType.kBrushless);
    leftBackDriveMotor = new CANSparkMax(1, MotorType.kBrushless);
    rightFrontDriveMotor = new CANSparkMax(2, MotorType.kBrushless);
    rightBackDriveMotor = new CANSparkMax(3, MotorType.kBrushless);

    leftFrontTurnMotor = new CANSparkMax(4, MotorType.kBrushless);
    leftBackTurnMotor = new CANSparkMax(5, MotorType.kBrushless);
    rightFrontTurnMotor = new CANSparkMax(6, MotorType.kBrushless);
    rightBackTurnMotor = new CANSparkMax(7, MotorType.kBrushless);

    double wheelP = 0.01;
    double wheelI = 0.001;
    double wheelD = 0.0;

    leftFrontWheel =
        new SwerveDriveWheel(
            wheelP,
            wheelI,
            wheelD,
            leftFrontTurnEncoder,
            leftFrontTurnMotor,
            leftFrontDriveMotor,
            Constants.kLeftFrontDriveInverted);
    leftBackWheel =
        new SwerveDriveWheel(
            wheelP,
            wheelI,
            wheelD,
            leftBackTurnEncoder,
            leftBackTurnMotor,
            leftBackDriveMotor,
            Constants.kLeftBackDriveInverted);
    rightFrontWheel =
        new SwerveDriveWheel(
            wheelP,
            wheelI,
            wheelD,
            rightFrontTurnEncoder,
            rightFrontTurnMotor,
            rightFrontDriveMotor,
            Constants.kRightFrontDriveInverted);
    rightBackWheel =
        new SwerveDriveWheel(
            wheelP,
            wheelI,
            wheelD,
            rightBackTurnEncoder,
            rightBackTurnMotor,
            rightBackDriveMotor,
            Constants.kRightBackDriveInverted);
    swerveCoordinator =
        new SwerveDriveCoordinator(leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel);

    leftFrontTurnEncoder = new CANcoder(8);
    leftBackTurnEncoder = new CANcoder(9);
    rightFrontTurnEncoder = new CANcoder(10);
    rightBackTurnEncoder = new CANcoder(11);

    leftFrontDriveEncoder = leftFrontDriveMotor.getEncoder();
    leftBackDriveEncoder = leftBackDriveMotor.getEncoder();
    rightFrontDriveEncoder = rightFrontDriveMotor.getEncoder();
    rightBackDriveEncoder = rightBackDriveMotor.getEncoder();

    gyro = new AHRS(SerialPort.Port.kMXP);
  }

  public double getGyroPosition() {
    return gyro.getAngle();
  }

  public void setCoordinator(double dir, double speed, double twist) {
    swerveCoordinator.setSwerveDrive(dir, twist, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
