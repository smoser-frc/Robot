// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class SimDrive extends Drive {

  private static PWMSparkMax leftFront = new PWMSparkMax(0);
  private static PWMSparkMax leftBack = new PWMSparkMax(1);
  private static PWMSparkMax rightFront = new PWMSparkMax(2);
  private static PWMSparkMax rightBack = new PWMSparkMax(3);
  private static Encoder leftEncoder = new Encoder(1, 2);
  private static Encoder rightEncoder = new Encoder(3, 4);
  private EncoderSim lEncSim = new EncoderSim(leftEncoder);
  private EncoderSim rEncSim = new EncoderSim(rightEncoder);
  private static DifferentialDrivetrainSim dtSim;
  private static ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
  private final ADXRS450_GyroSim gyroSim;
  
  public SimDrive() {
    super();

    leftGroup = new MotorControllerGroup(leftBack, leftFront);
    rightGroup = new MotorControllerGroup(rightFront, rightBack);
    leftGroup.setInverted(true);
    driveTrain = new DifferentialDrive(leftGroup, rightGroup);
    dtSim = new DifferentialDrivetrainSim(DCMotor.getNeo550(2), 8.451, 6, 57, Units.inchesToMeters(3), Units.inchesToMeters(22.5), null);

    odometer = new DifferentialDriveOdometry(dtSim.getHeading(), leftEncoder.getDistance(), rightEncoder.getDistance());
    gyroSim = new ADXRS450_GyroSim(m_gyro);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    odometer.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getDistance(), rightEncoder.getDistance());
    m_field.setRobotPose(odometer.getPoseMeters());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    dtSim.update(0.02);

    dtSim.setInputs(leftGroup.get() * RobotController.getBatteryVoltage() * -1, rightGroup.get() * RobotController.getBatteryVoltage() * -1);

    lEncSim.setDistance(dtSim.getLeftPositionMeters());
    lEncSim.setRate(dtSim.getLeftVelocityMetersPerSecond());
    rEncSim.setDistance(dtSim.getRightPositionMeters());
    rEncSim.setRate(dtSim.getRightVelocityMetersPerSecond());
    gyroSim.setAngle(-dtSim.getHeading().getDegrees());
  }

  public double getHeading(){
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * -1;
  }
}
