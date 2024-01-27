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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LaunchConstants.LaunchDistance;

public class Launcher extends SubsystemBase {
  /** Creates a new Launcher. */
  private CANSparkMax upper =
      new CANSparkMax(Constants.LaunchConstants.upperCANID, MotorType.kBrushless);

  private CANSparkMax lower =
      new CANSparkMax(Constants.LaunchConstants.lowerCANID, MotorType.kBrushless);

  private boolean tuningPIDS = false;

  private DoubleSolenoid angleSwitcher =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          Constants.LaunchConstants.angleSwitchForwardChannel,
          Constants.LaunchConstants.angleSwitchReverseChannel);

  public Launcher() {
    setPIDsDefault();
    showPIDs();
  }

  private void showPIDs() {
    SmartDashboard.putNumber("Launch P", upper.getPIDController().getP());
    SmartDashboard.putNumber("Launch I", upper.getPIDController().getI());
    SmartDashboard.putNumber("Launch D", upper.getPIDController().getD());
  }

  private void setPIDsDefault() {
    upper.getPIDController().setP(Constants.LaunchConstants.launcherP);
    upper.getPIDController().setI(Constants.LaunchConstants.launcherI);
    upper.getPIDController().setD(Constants.LaunchConstants.launcherD);

    lower.getPIDController().setP(Constants.LaunchConstants.launcherP);
    lower.getPIDController().setI(Constants.LaunchConstants.launcherI);
    lower.getPIDController().setD(Constants.LaunchConstants.launcherD);
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

  public Value getAnglePosition() {
    return angleSwitcher.get();
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
        differenceWithinPercentage(Constants.LaunchConstants.allowedDifferencePercent, targetVelo);
    boolean veloReady =
        isWithinVeloPercentage(Constants.LaunchConstants.allowedVeloPercent, targetVelo);
    return (differenceReady && veloReady);
  }

  public void setSwitcherPosition(LaunchDistance launchDistance) {
    switch (launchDistance) {
      case CLOSE:
        angleSwitcher.set(Constants.LaunchConstants.closeLaunchPosition);
        break;
      case FAR:
        angleSwitcher.set(Constants.LaunchConstants.farLaunchPosition);
        break;
    }
  }

  public void switchAngle() {
    boolean switcherForward = (angleSwitcher.get() == Value.kForward);
    angleSwitcher.set(switcherForward ? Value.kReverse : Value.kForward);
  }

  public void togglePIDTuning() {
    tuningPIDS = !tuningPIDS;
  }

  public void maintainSwitcherState() {
    angleSwitcher.set(angleSwitcher.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (tuningPIDS) {
      updatePIDFromDashboard("Launch");
    }
    maintainSwitcherState();
  }
}
