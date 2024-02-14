// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private CANSparkMax winchRight = new CANSparkMax(Constants.Climb.rightCANID, MotorType.kBrushless);
  private CANSparkMax winchLeft = new CANSparkMax(Constants.Climb.leftCANID, MotorType.kBrushless);
  private double speed;
  
  private boolean armExtended = false;
  private DigitalInput winchLimitLeft = new DigitalInput(Constants.Climb.winchLimitLeft);
  private DigitalInput winchLimitRight = new DigitalInput(Constants.Climb.winchLimitRight);

  /** Creates a new Climb. */
  public Climb() {}

  @Override
  public void periodic() {
    if (speed > 0) {
      return;
    }
    // This method will be called once per scheduler run
    if (winchLimitLeft.get()) {
      stopWinchLeft();
    }
    if (winchLimitRight.get()) {
      stopWinchRight();
    }
  }

  // This stops the left and right winches respectively.
  public void stopWinchLeft() {
    winchLeft.set(0);
  }
  public void stopWinchRight() {
    winchRight.set(0);
  }

  public void setWinch(double speed){
    double convertedSpeed = speed * Constants.Climb.motorSpeedFactor;
    this.speed = speed;
    winchLeft.set(convertedSpeed);
    winchRight.set(convertedSpeed);
  }

  public Command setWinchCommand(DoubleSupplier speed) {
    return this.run(() -> setWinch(speed.getAsDouble()));
  }
}
