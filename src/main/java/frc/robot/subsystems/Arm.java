// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm subsystem. */
  public Arm() {

    forewardLimit.enableLimitSwitch(false);
    reverseLimit.enableLimitSwitch(false);

    if (forewardLimit.isPressed()) {
      isFront = true;
    } else {
      isFront = false;
    }

    if (reverseLimit.isPressed()) {
      isBack = true;
    } else {
      isBack = false;
    }
  }

  private CANSparkMax armMotor = new CANSparkMax(1, MotorType.kBrushless);

  private SparkMaxLimitSwitch forewardLimit =
      armMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  private SparkMaxLimitSwitch reverseLimit =
      armMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

  private boolean isFront;
  private boolean isBack;

  public boolean queryFront() {
    return isFront;
  }

  public boolean queryBack() {
    return isBack;
  }

  public void checkLimits() {
    if (forewardLimit.isPressed()) {
      isFront = true;
    } else if (reverseLimit.isPressed()) {
      isBack = true;
    } else {
      isFront = false;
      isBack = false;
    }
  }

  public void setMotorForeward() {
    armMotor.set(Constants.armSpeed);
  }

  public void setMotorReverse() {
    armMotor.set(-Constants.armSpeed);
  }

  public void stopMotor() {
    armMotor.set(0);
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
