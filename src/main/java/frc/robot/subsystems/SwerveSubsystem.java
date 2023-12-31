// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  private final SwerveDrive swerveDrive;

  public double maxSpeed = Units.feetToMeters(14);

  public SwerveSubsystem(File directory) {
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maxSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  public SwerveSubsystem(
      SwerveDriveConfiguration driveConfig, SwerveControllerConfiguration controlConfig) {
    swerveDrive = new SwerveDrive(driveConfig, controlConfig, maxSpeed);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation, rotation, fieldRelative, false);
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
