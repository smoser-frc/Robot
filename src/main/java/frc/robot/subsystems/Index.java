// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {
  private CANSparkMax motorLower =
      new CANSparkMax(Constants.Index.lowerCANID, CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax motorWhooper =
      new CANSparkMax(Constants.Index.whooperCANID, CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax motorUpper =
      new CANSparkMax(Constants.Index.upperCANID, CANSparkLowLevel.MotorType.kBrushless);
  private DigitalInput breakBeam0 = new DigitalInput(Constants.Index.breakBeam);
  // init variables speed and currentSpeed
  double currentSpeed = 0;
  double speed = 3.5;

  public Index() {
    motorLower.setInverted(Constants.Index.lowerInverted);
    motorWhooper.setInverted(Constants.Index.whooperInverted);
    motorUpper.setInverted(Constants.Index.upperInverted);
  }

  private void set(double power) {
    motorLower.set(power);
    motorWhooper.set(power);
    motorUpper.set(power);
    currentSpeed = power;
  }

  public void start() {
    set(speed);
  }

  public void stop() {
    set(0.0);
  }

  public boolean isPrimed() {
    return breakBeam0.get();
  }

  public void toggle() {
    if (currentSpeed == 0.0) {
      start();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    if (isPrimed() == true) {
      stop();
    }
  }
}
