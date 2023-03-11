// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RealConstants;

public class Arm extends SubsystemBase {
  private CANSparkMax armMotor = new CANSparkMax(7, MotorType.kBrushless);

  private SparkMaxLimitSwitch reverseLimit =
      armMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

  private boolean isFront;
  private boolean isBack;

  private PIDController holdPID =
      new PIDController(RealConstants.kArmP, RealConstants.kArmI, RealConstants.kArmD);

  private RelativeEncoder armEnc = armMotor.getEncoder();

  private DigitalInput armProxSensor = new DigitalInput(0);

  /** Creates a new Arm subsystem. */
  public Arm() {
    reverseLimit.enableLimitSwitch(true);

    isBack = reverseLimit.isPressed();

    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setInverted(true);

    armEnc.setPosition(0);
    armEnc.setPositionConversionFactor(RealConstants.armConversionFactor);
  }

  public boolean queryFront() {
    return isFront;
  }

  public boolean queryBack() {
    return isBack;
  }

  public void setMotor(double speed) {
    double position = armEnc.getPosition();
    if (position <= -RealConstants.armForwardLimit && speed < 0) {
      armMotor.set(0);
    } else if (position >= -RealConstants.armReverseLimit && speed > 0) {
      armMotor.set(0);
    } else {
      armMotor.set(speed * RealConstants.armSpeed);
    }
  }

  public void setMotorReverse(double speed) {
    armMotor.set(speed);
  }

  public void stopMotor() {
    armMotor.set(0);
  }

  public double getVelocityRad() {
    return Units.degreesToRadians(armEnc.getVelocity());
  }

  public double getPosition() {
    return armEnc.getPosition();
  }

  public void setMotorVolts(double speed) {
    double position = getPosition();

    if (position <= -RealConstants.armForwardLimit && speed < 0) {
      armMotor.setVoltage(0);
    } else if (position >= -RealConstants.armReverseLimit && speed > 0) {
      armMotor.setVoltage(0);
    } else {
      armMotor.setVoltage(speed);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Position", armEnc.getPosition());
    SmartDashboard.putNumber("Arm Velocity", armEnc.getVelocity());
    SmartDashboard.putBoolean("Arm Limit", armProxSensor.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
