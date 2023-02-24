// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase {
  private CANSparkMax armMotor = new CANSparkMax(7, MotorType.kBrushless);

  private SparkMaxLimitSwitch forwardLimit =
      armMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  private SparkMaxLimitSwitch reverseLimit =
      armMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

  private boolean isFront;
  private boolean isBack;

  /** Creates a new Arm subsystem. */
  public Arm() {
    forwardLimit.enableLimitSwitch(true);
    reverseLimit.enableLimitSwitch(true);

    isFront = forwardLimit.isPressed();
    isBack = reverseLimit.isPressed();

    armMotor.setIdleMode(IdleMode.kBrake);
  }

  public boolean queryFront() {
    return isFront;
  }

  public boolean queryBack() {
    return isBack;
  }

  public void setMotorForward(DoubleSupplier speed) {
    armMotor.set(speed.getAsDouble());
  }

  public void setMotorReverse(double speed) {
    armMotor.set(speed);
  }

  public void stopMotor() {
    armMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    isFront = forwardLimit.isPressed();
    isBack = reverseLimit.isPressed();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
