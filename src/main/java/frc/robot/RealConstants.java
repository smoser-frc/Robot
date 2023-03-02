// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class RealConstants extends Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final double armSpeed = 0.55;
  public static final double driveSpeed = 0.95;
  public static final double clawSpeed = 0.9;
  public static final int kCPR = 42;
  public static final double speedConversionFactor = 1;
  public static final double gearRatioHigh = 4.77;
  public static final double gearRatioLow = 20.67;
  public static final double wheelDiameter = Units.inchesToMeters(6);
  public static final double kMetersPerRev = (wheelDiameter * Math.PI / gearRatioHigh);

  public static final double kTrackwidthMeters = 0.69;
  public static final DifferentialDriveKinematics kDriveKinematics =
      new DifferentialDriveKinematics(kTrackwidthMeters);

  // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
  // These characterization values MUST be determined either experimentally or theoretically
  // for *your* robot's drive.
  // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
  // values for your robot.
  public static final double ksVolts = 0.22;
  public static final double kvVoltSecondsPerMeter = 1.98;
  public static final double kaVoltSecondsSquaredPerMeter = 0.2;

  // Example value only - as above, this must be tuned for your drive!
  public static final double kPDriveVel = 8.5;

  public static final double kMaxSpeedMetersPerSecond = 1;
  public static final double kMaxAccelerationMetersPerSecondSquared = 0.25;

  // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  public static final double kArmP = 0;
  public static final double kArmI = 0;
  public static final double kArmD = 0;

  public static final double kClawP = 0;
  public static final double kClawI = 0;
  public static final double kClawD = 0;

  public static final float armForwardLimit = 255;
  public static final float armReverseLimit = 5;

  public static final double armGearRatio = 192;
  public static final double armConversionFactor = 360 / armGearRatio;

  public static final double clawGearRatio = 144;
  public static final double clawConversionFactor = 360 / clawGearRatio;

  public static final float clawForwardLimit = 90;
  public static final float clawReverseLimit = -20;

  public static final double kArmPositionToleranceDegrees = 5;
}
