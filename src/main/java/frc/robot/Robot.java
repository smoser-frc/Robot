// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final CANSparkMax m_leftMotor1 = new CANSparkMax(1, MotorType.kBrushed);
  private final CANSparkMax m_leftMotor2 = new CANSparkMax(2, MotorType.kBrushed);
  private final CANSparkMax m_rightMotor1 = new CANSparkMax(3, MotorType.kBrushed);
  private final CANSparkMax m_rightMotor2 = new CANSparkMax(4, MotorType.kBrushed);
  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  private final XboxController m_xbox1 = new XboxController(0);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);
  }

  @Override
  public void teleopPeriodic() {
    boolean arcadeDrive = true;

    if (arcadeDrive) {
      // Drive with arcade drive.
      // That means that the Y axis drives forward
      // and backward, and the X turns left and right.
      m_robotDrive.arcadeDrive(-m_xbox1.getLeftY(), -m_xbox1.getLeftX());
    } else {
      // Drive with tank Drive
      // That means that the left Y access controls the left side motors
      // and the right Y access controls the right side motors.
      // the left X and right Y access do not do anything.
      m_robotDrive.tankDrive(-m_xbox1.getLeftY(), -m_xbox1.getRightY());
    }
  }
}
