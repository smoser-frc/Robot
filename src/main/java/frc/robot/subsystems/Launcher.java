// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Launch.LaunchPosition;

public class Launcher extends SubsystemBase {
  /** Creates a new Launcher. */
  private CANSparkMax upper = new CANSparkMax(Constants.Launch.upperCANID, MotorType.kBrushless);

  private CANSparkMax lower = new CANSparkMax(Constants.Launch.lowerCANID, MotorType.kBrushless);

  private boolean tuningPIDS = false;

  private DoubleSolenoid angleSwitcher =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          Constants.Launch.angleSwitchForwardChannel,
          Constants.Launch.angleSwitchReverseChannel);

  private LaunchPosition goalPosition, curPosition;

  private Timer switchTimer;

  public Launcher() {
    switch (angleSwitcher.get()) {
      case kForward:
        curPosition = Constants.Launch.LaunchPosition.CLOSE;
        break;
      case kOff:
        angleSwitcher.set(Value.kForward);
        curPosition = Constants.Launch.LaunchPosition.CLOSE;
        break;
      case kReverse:
        curPosition = Constants.Launch.LaunchPosition.FAR;
        break;
    }

    goalPosition = curPosition;

    setPIDsDefault();
    showPIDs();
  }

  private void showPIDs() {
    SmartDashboard.putNumber("Launch P", upper.getPIDController().getP());
    SmartDashboard.putNumber("Launch I", upper.getPIDController().getI());
    SmartDashboard.putNumber("Launch D", upper.getPIDController().getD());
  }

  private void setPIDsDefault() {
    updatePIDs(Constants.Launch.launcherP, Constants.Launch.launcherI, Constants.Launch.launcherD);
  }

  private void updatePIDs(double p, double i, double d) {
    upper.getPIDController().setP(p);
    upper.getPIDController().setI(i);
    upper.getPIDController().setD(d);

    lower.getPIDController().setP(p);
    lower.getPIDController().setI(i);
    lower.getPIDController().setD(d);
  }

  private void updatePIDFromDashboard(String keyWord) {
    double p = SmartDashboard.getNumber(keyWord + " P", 0);
    double i = SmartDashboard.getNumber(keyWord + " I", 0);
    double d = SmartDashboard.getNumber(keyWord + " D", 0);

    updatePIDs(p, i, d);
  }

  public void setLaunchVelocity(double velocity) {
    upper.getPIDController().setReference(velocity, ControlType.kSmartVelocity);
    lower.getPIDController().setReference(velocity, ControlType.kSmartVelocity);
  }

  public double getCurrentVelocity() {
    double upperVelocity = upper.getEncoder().getVelocity();
    double lowerVelocity = lower.getEncoder().getVelocity();
    return (upperVelocity + lowerVelocity) / 2;
  }

  public double getVelocityDifference() {
    double upperVelocity = upper.getEncoder().getVelocity();
    double lowerVelocity = lower.getEncoder().getVelocity();
    return Math.abs(upperVelocity - lowerVelocity);
  }

  public boolean isWithinVeloPercentage(double percent, double targetVelo) {
    double currentPercent = getCurrentVelocity() / targetVelo;
    return (1 - percent <= currentPercent && currentPercent <= 1 + percent);
  }

  public boolean differenceWithinPercentage(double percent, double targetVelo) {
    double differencePercent = getVelocityDifference() / targetVelo;
    return (differencePercent <= percent);
  }

  public boolean readyToLaunch(double targetVelo) {
    boolean differenceReady =
        differenceWithinPercentage(Constants.Launch.allowedDifferencePercent, targetVelo);
    boolean veloReady = isWithinVeloPercentage(Constants.Launch.allowedVeloPercent, targetVelo);
    return (differenceReady && veloReady);
  }

  public void setLaunchPosition(LaunchPosition launchPosition) {
    if (goalPosition == launchPosition) {
      return;
    }
    switch (launchPosition) {
      case CLOSE:
        angleSwitcher.set(Constants.Launch.closeLaunchPosition);
        break;
      case FAR:
        angleSwitcher.set(Constants.Launch.farLaunchPosition);
        break;
    }
    goalPosition = launchPosition;
    switchTimer.start();
  }

  public LaunchPosition getLaunchPosition() {
    return curPosition;
  }

  public void switchAngle() {
    boolean switcherForward = (angleSwitcher.get() == Value.kForward);
    angleSwitcher.set(switcherForward ? Value.kReverse : Value.kForward);
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
    if (switchTimer.hasElapsed(.5)) {
      curPosition = goalPosition;
      switchTimer.stop();
      switchTimer.reset();
    }
  }
}
