// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final double kEncoderCPR = 1;
    public static final double kWheelDiameterInches = 6;
    public static final double kGearRatioHigh = 4.77;
    public static final double kGearRatioLow = 20.2;
    public static final double kTrackwidthMeters = Units.inchesToMeters(21.75);
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterInches * Math.PI) / kGearRatioHigh / (double) kEncoderCPR;

    public static final boolean kGyroReversed = false;

    public static final double kStabilizationP = 1;
    public static final double kStabilizationI = 0.5;
    public static final double kStabilizationD = 0;

    public static final double kTurnP = 0.05;
    public static final double kTurnI = 0.001;
    public static final double kTurnD = 0;

    public static final double kDriveP = .10;
    public static final double kDriveI = .005;
    public static final double kDriveD = 0;

    public static final double kMaxTurnRateDegPerS = 100;
    public static final double kMaxTurnAccelerationDegPerSSquared = 300;

    public static final double kTurnToleranceDeg = 5;
    public static final double kTurnRateToleranceDegPerS = 10; // degrees per second

    public static final double kDriveToleranceInches = 1;
    public static final double kDriveRateToleranceInchesPerS = 10; // inches per second
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }
}
