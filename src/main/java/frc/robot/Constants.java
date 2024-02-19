// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int pneumaticsControlModuleCANID = 2;

  public static final boolean kLeftFrontDriveInverted = false;
  public static final boolean kLeftBackDriveInverted = false;
  public static final boolean kRightBackDriveInverted = false;
  public static final boolean kRightFrontDriveInverted = false;

  public static final double LOOP_TIME = 0.13;
  public static final double ROBOT_MASS = 150 * 0.453592;
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);

  public static class Index {
    public static final int lowerCANID = 40;
    public static final int whooperCANID = 41;
    public static final int upperCANID = 42;
    public static final boolean lowerInverted = true;
    public static final boolean upperInverted = true;
    public static final boolean whooperInverted = false;
    public static final int breakBeam = 1;
  }

  public static class Intake {
    public static final int leftCANID = 30;
    public static final int centerCANID = 31;
    public static final int rightCANID = 32;
    public static final int breakBeam = 0;
    public static final boolean leftInverted = false;
    public static final boolean centerInverted = true;
    public static final boolean rightInverted = true;
    public static final double speed = 0.6;
  }

  public static class Launch {
    public static final double launcherP = 0;
    public static final double launcherI = 0;
    public static final double launcherD = 0;
    public static final double launcherFF = 0.000156;
    public static final double angleP = 0.004;
    public static final double angleI = 0;
    public static final double angleD = 0;
    public static final double angleFF = 0;
    public static final int launcherID = 45;
    public static final int angleID = 46;
    public static final double allowedVeloPercent = 0.05;
    public static final double allowedDifferencePercent = 0.05;
    public static final double closeLaunchPosition = 0.207;
    public static final double farLaunchPosition = 0.162;
    public static final double angleMin = 0.161;
    public static final double angleMax = 0.234;
    public static final double speedCloseSpeaker = 0.9;
    public static final double angleConversionFactor = 1;
    public static final double launcherConversionFactor = 1; // FIXME

    public enum LaunchPosition {
      CLOSE,
      FAR
    }
  }

  public static class Climb {
    public static final int leftCANID = 50;
    public static final int rightCANID = 51;
    // These are break beam sensor IDS
    public static final int winchLimitLeft = 2;
    public static final int winchLimitRight = 3;
    public static final double motorSpeedFactor = 0.5;
  }

  public static final String limelightName = "limelight";

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double LEFT_X_DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.01;
    public static final double RIGHT_X_DEADBAND = 0.01;
  }
}
