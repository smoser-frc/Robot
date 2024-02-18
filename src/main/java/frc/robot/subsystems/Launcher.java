// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
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

  static String labelLaunchPre = "Launch ";
  static String labelAnglePre = "Angle ";

  double goalAngle, curAngle;

  public Launcher() {
    angleEncoder = angleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    angleEncoder.setZeroOffset(Constants.Launch.launchAngleEncoderOffset);

    angleMotor.getPIDController().setFeedbackDevice(angleEncoder);
    angleMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Launch.launchAngleMax);
    angleMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Launch.launchAngleMin);
    angleMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    angleMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    System.out.println(
        "encodervalue ="
            + angleEncoder.getPosition()
            + ", 0offset ="
            + angleEncoder.getZeroOffset());

    setPIDsDefault();
    showPIDs();
  }

  private void showPIDs() {
    String n = labelLaunchPre;
    SmartDashboard.putNumber(n + "P", shoot.getPIDController().getP());
    SmartDashboard.putNumber(n + "I", shoot.getPIDController().getI());
    SmartDashboard.putNumber(n + "D", shoot.getPIDController().getD());

    n = labelAnglePre;
    SmartDashboard.putNumber(n + "P", angleMotor.getPIDController().getP());
    SmartDashboard.putNumber(n + "I", angleMotor.getPIDController().getI());
    SmartDashboard.putNumber(n + "D", angleMotor.getPIDController().getD());
    SmartDashboard.putNumber(n + "Pos", angleEncoder.getPosition());
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

  private void updateShootPIDFromDashboard() {
    String n = labelLaunchPre;
    updateShootPIDs(
        SmartDashboard.getNumber(n + " P", 0),
        SmartDashboard.getNumber(n + " I", 0),
        SmartDashboard.getNumber(n + " D", 0));
  }

  private void updateAnglePIDs(double p, double i, double d) {
    angleMotor.getPIDController().setP(p);
    angleMotor.getPIDController().setI(i);
    angleMotor.getPIDController().setD(d);
  }

  private void updateAnglePIDFromDashboard() {
    String n = labelAnglePre;
    updateAnglePIDs(
        SmartDashboard.getNumber(n + " P", 0),
        SmartDashboard.getNumber(n + " I", 0),
        SmartDashboard.getNumber(n + " D", 0));
  }

  public void setLaunchVelocity(double velocity) {
    shoot.getPIDController().setReference(velocity, ControlType.kSmartVelocity);
  }

  public void setAngle(double angle) {
    goalAngle = angle;
    REVLibError err = angleMotor.getPIDController().setReference(angle, ControlType.kSmartMotion);
    if (err != REVLibError.kOk) {
      System.out.println("YIKES! setAngle(" + angle + ") err is " + err);
    }
  }

  public void switchAngle() {
    double newTarget = Constants.Launch.launchAngleLow;
    if (goalAngle == Constants.Launch.launchAngleLow) {
      newTarget = Constants.Launch.launchAngleHigh;
    }
    setAngle(newTarget);
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
      updateShootPIDFromDashboard();
      updateAnglePIDFromDashboard();
    }
  }
}
// Ayyo we don't got a robot yet fr fr.
