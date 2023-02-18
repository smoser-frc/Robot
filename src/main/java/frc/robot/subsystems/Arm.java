// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RealConstants;

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
    forwardLimit.enableLimitSwitch(false);
    reverseLimit.enableLimitSwitch(false);

    isFront = forwardLimit.isPressed();
    isBack = reverseLimit.isPressed();
  }

  public boolean queryFront() {
    return isFront;
  }

  public boolean queryBack() {
    return isBack;
  }

  public void setMotorForward() {
    armMotor.set(RealConstants.armSpeed);
  }

  public void setMotorReverse() {
    armMotor.set(-RealConstants.armSpeed);
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
