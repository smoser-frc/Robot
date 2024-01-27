// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax motorLeft = new CANSparkMax(30, CANSparkLowLevel.MotorType.kBrushless);

  private CANSparkMax motorCenter = new CANSparkMax(31, CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax motorRight = new CANSparkMax(32, CANSparkLowLevel.MotorType.kBrushless);

  private DigitalInput sensor = new DigitalInput(Constants.IntakeConstants.sensorID);

  double speed = 0.35;
  double currentSpeed = 0.0;

  public Intake() {
    motorLeft.setInverted(false);
    motorCenter.setInverted(false);
    motorRight.setInverted(true);
    stop();
  }

  private void set(double power) {
    motorLeft.set(power);
    motorCenter.set(power);
    motorRight.set(power);
    currentSpeed = power;
  }

  public void toggle() {
    if (currentSpeed == 0.0) {
      start();
    } else {
      stop();
    }
  }

  public boolean noteAtSquishPoint() {
    return sensor.get();
  }

  public void start() {
    set(speed);
  }

  public void reverse() {
    set(-speed);
  }

  public void stop() {
    set(0.0);
  }

  public boolean isRunning() {
    return !(motorLeft.get() == 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake Running", isRunning());
  }
}
