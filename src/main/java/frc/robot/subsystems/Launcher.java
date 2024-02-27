// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Launch.LaunchPosition;
import frc.robot.Robot;

public class Launcher extends SubsystemBase {
  /** Creates a new Launcher. */
  private CANSparkFlex upperLauncher =
      new CANSparkFlex(Constants.Launch.upperLauncherID, MotorType.kBrushless);

  private CANSparkFlex lowerLauncher =
      new CANSparkFlex(Constants.Launch.lowerLauncherID, MotorType.kBrushless);

  private SparkPIDController upperLauncherController;
  private SparkPIDController lowerLauncherController;
  private RelativeEncoder upperLauncherEncoder;
  private RelativeEncoder lowerLauncherEncoder;

  private CANSparkMax angle = new CANSparkMax(Constants.Launch.angleID, MotorType.kBrushless);
  private SparkPIDController angleController;
  private SparkAbsoluteEncoder angleEncoder;

  private boolean tuningPIDS = false;

  private LaunchPosition goalPosition, curPosition;

  public Launcher() {

    angle.setInverted(Constants.Launch.angleMotorInverted);
    upperLauncher.setInverted(Constants.Launch.upperMotorInverted);
    lowerLauncher.setInverted(Constants.Launch.lowerMotorInverted);

    angleController = angle.getPIDController();
    upperLauncherController = upperLauncher.getPIDController();
    lowerLauncherController = lowerLauncher.getPIDController();

    upperLauncherEncoder = upperLauncher.getEncoder();
    lowerLauncherEncoder = lowerLauncher.getEncoder();
    angleEncoder = angle.getAbsoluteEncoder(Type.kDutyCycle);
    angleEncoder.setInverted(Constants.Launch.angleEncoderInverted);

    upperLauncherController.setFeedbackDevice(upperLauncherEncoder);
    lowerLauncherController.setFeedbackDevice(lowerLauncherEncoder);
    angleController.setFeedbackDevice(angleEncoder);

    upperLauncherEncoder.setVelocityConversionFactor(Constants.Launch.launcherConversionFactor);
    lowerLauncherEncoder.setVelocityConversionFactor(Constants.Launch.launcherConversionFactor);
    angleEncoder.setPositionConversionFactor(Constants.Launch.angleConversionFactor);
    angleEncoder.setVelocityConversionFactor(Constants.Launch.angleConversionFactor);

    setPIDsDefault();
    showPIDs();
    SmartDashboard.putNumber("Launch Speed", 0);

    angleController.setFF(0);
    angleController.setOutputRange(-1, 1);

    angleController.setSmartMotionMaxVelocity(2000, 0);
    angleController.setSmartMotionMinOutputVelocity(0, 0);
    angleController.setSmartMotionMaxAccel(1500, 0);
    angleController.setSmartMotionAllowedClosedLoopError(0, 0);
  }

  private void showPIDs() {
    SmartDashboard.putNumber("Launch P", upperLauncherController.getP());
    SmartDashboard.putNumber("Launch I", upperLauncherController.getI());
    SmartDashboard.putNumber("Launch D", upperLauncherController.getD());
    SmartDashboard.putNumber("Launch Velo", getCurrentVelocity());

    SmartDashboard.putNumber("Angle P", angleController.getP());
    SmartDashboard.putNumber("Angle I", angleController.getI());
    SmartDashboard.putNumber("Angle D", angleController.getD());
    SmartDashboard.putNumber("Angle Position", angleEncoder.getPosition());
  }

  private void setPIDsDefault() {
    updateLauncherPIDs(
        Constants.Launch.launcherP,
        Constants.Launch.launcherI,
        Constants.Launch.launcherD,
        Constants.Launch.launcherFF);
    updateAnglePIDs(Constants.Launch.angleP, Constants.Launch.angleI, Constants.Launch.angleD, 0);
  }

  private void updateLauncherPIDs(double p, double i, double d, double ff) {
    upperLauncherController.setP(p);
    upperLauncherController.setI(i);
    upperLauncherController.setD(d);
    upperLauncherController.setFF(ff);

    lowerLauncherController.setP(p);
    lowerLauncherController.setI(i);
    lowerLauncherController.setD(d);
    lowerLauncherController.setFF(ff);
  }

  private void updateAnglePIDs(double p, double i, double d, double ff) {
    angleController.setP(p);
    angleController.setI(i);
    angleController.setD(d);
    angleController.setFF(ff);
  }

  private void updatePIDFromDashboard(String keyWord, MyMethod runnable) {
    double p = SmartDashboard.getNumber(keyWord + " P", 0);
    double i = SmartDashboard.getNumber(keyWord + " I", 0);
    double d = SmartDashboard.getNumber(keyWord + " D", 0);
    double ff = SmartDashboard.getNumber(keyWord + "ff", 0);

    runnable.apply(p, i, d, ff);
  }

  public void setLaunchVelocity(double velocity) {
    upperLauncherController.setReference(velocity, ControlType.kVelocity);
    lowerLauncherController.setReference(velocity, ControlType.kVelocity);
  }

  public double getCurrentVelocity() {
    return (upperLauncherEncoder.getVelocity() + lowerLauncherEncoder.getVelocity()) / 2;
  }

  public void setLauncherSpeed(double speed) {
    upperLauncher.set(speed);
    lowerLauncher.set(speed);
  }

  public boolean isWithinVeloPercentage(double percent, double targetVelo) {
    double currentPercent = getCurrentVelocity() / targetVelo;
    return (1 - (percent * 0.01) <= currentPercent && currentPercent <= 1 + (percent * 0.01));
  }

  public boolean motorDifferenceWithinPercent(double percent) {
    double currentPercent =
        Math.abs(lowerLauncherEncoder.getVelocity() - upperLauncherEncoder.getVelocity())
            / getCurrentVelocity();
    return (1 - (percent * 0.01) <= currentPercent && currentPercent <= 1 + (percent * 0.01));
  }

  public boolean readyToLaunch(double targetVelo) {
    if (Robot.isSimulation()) {
      return true;
    }
    boolean isReady = isWithinVeloPercentage(Constants.Launch.allowedVeloPercent, targetVelo); // &&
    // motorDifferenceWithinPercent(Constants.Launch.allowedDifferencePercent);
    return isReady;
  }

  public void setLaunchPosition(LaunchPosition launchPosition) {
    goalPosition = launchPosition;
  }

  public LaunchPosition getLaunchPosition() {
    return curPosition;
  }

  public void switchAngle() {
    boolean isFar = (goalPosition == LaunchPosition.FAR);
    goalPosition = (isFar ? LaunchPosition.CLOSE : LaunchPosition.FAR);
  }

  public void togglePIDTuning() {
    tuningPIDS = !tuningPIDS;
  }

  @FunctionalInterface
  private interface MyMethod {
    void apply(double p, double i, double d, double ff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (tuningPIDS) {
      updatePIDFromDashboard("Launcher", this::updateLauncherPIDs);
      updatePIDFromDashboard("Angle", this::updateAnglePIDs);
    }
    if (goalPosition == LaunchPosition.FAR) {
      angleController.setReference(Constants.Launch.farLaunchPosition, ControlType.kSmartMotion);
      curPosition = goalPosition;
      SmartDashboard.putNumber("Angle Desired", Constants.Launch.farLaunchPosition);
    } else {
      angleController.setReference(Constants.Launch.closeLaunchPosition, ControlType.kSmartMotion);
      curPosition = goalPosition;
      SmartDashboard.putNumber("Angle Desired", Constants.Launch.closeLaunchPosition);
    }
    SmartDashboard.putNumber("Angle Speed", angle.get());
    SmartDashboard.putNumber("Launch Curr Velo", getCurrentVelocity());
    SmartDashboard.putNumber("Angle Position", angleEncoder.getPosition());
  }

  public Command switchAngleCommand() {
    return this.runOnce(() -> switchAngle());
  }
}
