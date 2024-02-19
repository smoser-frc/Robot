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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Launch.LaunchPosition;
import frc.robot.Robot;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

public class Launcher extends SubsystemBase {
  /** Creates a new Launcher. */
  private CANSparkFlex motor = new CANSparkFlex(Constants.Launch.motorID, MotorType.kBrushless);

  private SparkPIDController motorController;
  private RelativeEncoder motorEncoder;

  private CANSparkMax winch = new CANSparkMax(Constants.Launch.winchID, MotorType.kBrushless);
  private SparkPIDController winchController;
  private SparkAbsoluteEncoder winchEncoder;

  private boolean tuningPIDS = false;

  private LaunchPosition goalPosition, curPosition;

  private Timer switchTimer;

  public Launcher() {

    winchController = winch.getPIDController();
    motorController = motor.getPIDController();

    motorEncoder = motor.getEncoder();
    winchEncoder = winch.getAbsoluteEncoder(Type.kDutyCycle);

    motorController.setFeedbackDevice(motorEncoder);
    winchController.setFeedbackDevice(winchEncoder);

    motorEncoder.setVelocityConversionFactor(Constants.Launch.motorConversionFactor);
    winchEncoder.setPositionConversionFactor(Constants.Launch.winchConversionFactor);

    setPIDsDefault();
    showPIDs();
  }

  private void showPIDs() {
    SmartDashboard.putNumber("Launch P", motorController.getP());
    SmartDashboard.putNumber("Launch I", motorController.getI());
    SmartDashboard.putNumber("Launch D", motorController.getD());

    SmartDashboard.putNumber("Winch P", winchController.getP());
    SmartDashboard.putNumber("Winch I", winchController.getI());
    SmartDashboard.putNumber("Winch D", winchController.getD());
  }

  private void setPIDsDefault() {
    updateLaunchPIDs(
        Constants.Launch.launcherP, Constants.Launch.launcherI, Constants.Launch.launcherD);
    updateWinchPIDs(Constants.Launch.winchP, Constants.Launch.winchI, Constants.Launch.winchD);
  }

  private void updateLaunchPIDs(double p, double i, double d) {
    motorController.setP(p);
    motorController.setI(i);
    motorController.setD(d);
  }

  private void updateWinchPIDs(double p, double i, double d) {
    winchController.setP(p);
    winchController.setI(i);
    winchController.setD(d);
  }

  private void updatePIDFromDashboard(String keyWord) {
    double p = SmartDashboard.getNumber(keyWord + " P", 0);
    double i = SmartDashboard.getNumber(keyWord + " I", 0);
    double d = SmartDashboard.getNumber(keyWord + " D", 0);

    String desiredMethod = "update" + keyWord + "PIDs";
    Class<?>[] paramTypes = {double.class, double.class, double.class};
    Method method;
    try {
      method = this.getClass().getMethod(desiredMethod, paramTypes);
    } catch (SecurityException e) {
      return;
    } catch (NoSuchMethodException e) {
      return;
    }
    try {
      method.invoke(this, p, i, d);
    } catch (IllegalArgumentException e) {
      return;
    } catch (IllegalAccessException e) {
      return;
    } catch (InvocationTargetException e) {
      return;
    }
  }

  public void setLaunchVelocity(double velocity) {
    motorController.setReference(velocity, ControlType.kVelocity);
  }

  public double getCurrentVelocity() {
    return motorEncoder.getVelocity();
  }

  public void setMotorSpeed(double speed) {
    motor.set(speed);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (tuningPIDS) {
      updatePIDFromDashboard("Launch");
      updatePIDFromDashboard("Winch");
    }
    if (goalPosition == LaunchPosition.FAR) {
      winchController.setReference(Constants.Launch.farLaunchPosition, ControlType.kPosition);
    } else {
      winchController.setReference(Constants.Launch.closeLaunchPosition, ControlType.kPosition);
    }
  }
}
