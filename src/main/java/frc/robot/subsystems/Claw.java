// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  private CANSparkMax clawMotor = new CANSparkMax(8, MotorType.kBrushless);

  private SparkMaxLimitSwitch forewardLimit =
      clawMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  private SparkMaxLimitSwitch reverseLimit =
      clawMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

  private boolean isClosed;
  private boolean isOpen;

  public Claw() {
    forewardLimit.enableLimitSwitch(false);
    reverseLimit.enableLimitSwitch(false);

    isClosed = forewardLimit.isPressed();
    isOpen = reverseLimit.isPressed();
  }

  public boolean queryClosed() {
    return isClosed;
  }

  public boolean queryOpen() {
    return isOpen;
  }

  public void checkLimits() {
    isClosed = forewardLimit.isPressed();
    isOpen = reverseLimit.isPressed();
  }

  public void setMotorForward() {
    clawMotor.set(Constants.clawSpeed);
  }

  public void setMotorReverse() {
    clawMotor.set(-Constants.clawSpeed);
  }

  public void stopMotor() {
    clawMotor.set(0);
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
