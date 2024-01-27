// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {
  public Index() {}

  private CANSparkMax motorLower = new CANSparkMax(40, CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax motorUpper = new CANSparkMax(41, CANSparkLowLevel.MotorType.kBrushless);
  private DigitalInput breakBeam0 = new DigitalInput(0);
  // init variables speed and currentSpeed
  double currentSpeed = 0;
  double speed = 3.5;

  private void set(double power) {
    motorLower.set(power);
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

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {

          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
