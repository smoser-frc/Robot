// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Launcher extends SubsystemBase {
  /** Creates a new Launcher. */
  private CANSparkFlex shoot = new CANSparkFlex(Constants.Launch.shootCANID, MotorType.kBrushless);

  private CANSparkMax angleMotor =
      new CANSparkMax(Constants.Launch.angleCANID, MotorType.kBrushless);
  private SparkAbsoluteEncoder angleEncoder;
  private boolean tuningPIDS = false;

  public Launcher() {
    angleEncoder = angleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    angleEncoder.setZeroOffset(Constants.Launch.launchAngleEncoderOffset);
    System.out.println(
        "encodervalue ="
            + angleEncoder.getPosition()
            + ", 0offset ="
            + angleEncoder.getZeroOffset());

    setPIDsDefault();
    showPIDs();
  }

  private void showPIDs() {
    SmartDashboard.putNumber("Launch P", shoot.getPIDController().getP());
    SmartDashboard.putNumber("Launch I", shoot.getPIDController().getI());
    SmartDashboard.putNumber("Launch D", shoot.getPIDController().getD());

    SmartDashboard.putNumber("Angle P", angleMotor.getPIDController().getP());
    SmartDashboard.putNumber("Angle I", angleMotor.getPIDController().getI());
    SmartDashboard.putNumber("Angle D", angleMotor.getPIDController().getD());

    SmartDashboard.putNumber("Angle Pos", angleEncoder.getPosition());
  }

  private void setPIDsDefault() {
    updateShootPIDs(
        Constants.Launch.launcherP, Constants.Launch.launcherI, Constants.Launch.launcherD);
    updateAnglePIDs(Constants.Launch.angleP, Constants.Launch.angleI, Constants.Launch.angleD);
  }

  private void updateShootPIDs(double p, double i, double d) {
    shoot.getPIDController().setP(p);
    shoot.getPIDController().setI(i);
    shoot.getPIDController().setD(d);
  }

  private void updateAnglePIDs(double p, double i, double d) {
    angleMotor.getPIDController().setP(p);
    angleMotor.getPIDController().setI(i);
    angleMotor.getPIDController().setD(d);
  }

  private void updatePIDFromDashboard(String keyWord) {
    double p = SmartDashboard.getNumber(keyWord + " P", 0);
    double i = SmartDashboard.getNumber(keyWord + " I", 0);
    double d = SmartDashboard.getNumber(keyWord + " D", 0);

    updateShootPIDs(p, i, d);
  }

  public void setLaunchVelocity(double velocity) {
    shoot.getPIDController().setReference(velocity, ControlType.kSmartVelocity);
  }

  public void setAngle(double angle) {
    angleMotor.getPIDController().setReference(angle, ControlType.kPosition);
  }

  public void switchAngle() {
    // FIXMEEEE
    setAngle(Constants.Launch.launchAngleHigh);
  }

  public double getCurrentVelocity() {
    return shoot.getEncoder().getVelocity();
  }

  public boolean isWithinVeloPercentage(double percent, double targetVelo) {
    double currentPercent = getCurrentVelocity() / targetVelo;
    return (1 - percent <= currentPercent && currentPercent <= 1 + percent);
  }

  public boolean readyToLaunch(double targetVelo) {
    if (Robot.isSimulation()) {
      return true;
    }
    return isWithinVeloPercentage(Constants.Launch.allowedVeloPercent, targetVelo);
  }

  public void togglePIDTuning() {
    tuningPIDS = !tuningPIDS;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (tuningPIDS) {
      updatePIDFromDashboard("Launch");
    }
  }
}
// Ayyo we don't got a robot yet fr fr.
