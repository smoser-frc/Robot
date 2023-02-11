// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Drive extends SubsystemBase {

  public void setTankDrive(DoubleSupplier m_lSpeed, DoubleSupplier m_rSpeed, double m_pOutput) {
    throw new java.lang.UnsupportedOperationException();
  }
  public Pose2d getPose() {
    throw new java.lang.UnsupportedOperationException();
  }

  public void resetOdometry(Pose2d pose) {
    throw new java.lang.UnsupportedOperationException();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    throw new java.lang.UnsupportedOperationException();
  }

  public void zeroEncoders() {
    throw new java.lang.UnsupportedOperationException();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    throw new java.lang.UnsupportedOperationException();
  }

  public void zeroHeading(){
    throw new java.lang.UnsupportedOperationException();
  }

  public double getHeading(){
    throw new java.lang.UnsupportedOperationException();
  }

  public double getTurnRate(){
    throw new java.lang.UnsupportedOperationException();
  }
}
