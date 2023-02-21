// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmForward;
import frc.robot.commands.ArmReverse;
import frc.robot.commands.ClawSwitch;
import frc.robot.commands.DriveTank;
import frc.robot.commands.ManualClaw;
import frc.robot.commands.SwitchGears;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GearShifter;
import frc.robot.subsystems.RealDrive;
import frc.robot.subsystems.SimDrive;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Dont remove example until autons are programmed
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private Drive m_drive;
  // private final Arm m_arm = new Arm();
  // private final Claw m_claw = new Claw();
  private final GearShifter m_gearShifter = new GearShifter();
  private Constants m_constants;
  private final Constants m_realConstants = new RealConstants();
  private final Constants m_simConstants = new SimConstants();
  private final Arm m_arm = new Arm();
  private final Claw m_claw = new Claw();

  private final XboxController leftStick = new XboxController(0);
  private final XboxController rightStick = new XboxController(1);
  private final XboxController coDriver = new XboxController(2);
  // private final XboxController xboxController = new XboxController(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    if (RobotBase.isSimulation()) {
      m_drive = new SimDrive();
      m_constants = m_simConstants;
      m_drive.setDefaultCommand(
          new DriveTank(
              m_drive, leftStick::getLeftY, leftStick::getRightY, m_constants.driveSpeed));
    } else {
      m_drive = new RealDrive();
      m_constants = m_realConstants;
      m_drive.setDefaultCommand(
          new DriveTank(
              m_drive, leftStick::getLeftY, leftStick::getRightY, m_constants.driveSpeed));
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    final JoystickButton codriverA = new JoystickButton(coDriver, XboxController.Button.kA.value);
    final JoystickButton codriverB = new JoystickButton(coDriver, XboxController.Button.kB.value);
    final JoystickButton leftStickTrigger = new JoystickButton(leftStick, 1);
    final JoystickButton rightStickTrigger = new JoystickButton(rightStick, 1);

    codriverA.whileTrue(new ArmForward(m_arm));
    codriverB.whileTrue(new ArmReverse(m_arm));
    leftStickTrigger.onTrue(new ClawSwitch(m_claw));
    rightStickTrigger.whileTrue(new SwitchGears(m_gearShifter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                RealConstants.ksVolts,
                RealConstants.kvVoltSecondsPerMeter,
                RealConstants.kaVoltSecondsSquaredPerMeter),
            RealConstants.kDriveKinematics,
            5);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                RealConstants.kMaxSpeedMetersPerSecond,
                RealConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(RealConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_drive::getPose,
            new RamseteController(RealConstants.kRamseteB, RealConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                RealConstants.ksVolts,
                RealConstants.kvVoltSecondsPerMeter,
                RealConstants.kaVoltSecondsSquaredPerMeter),
            RealConstants.kDriveKinematics,
            m_drive::getWheelSpeeds,
            new PIDController(RealConstants.kPDriveVel, 0, 0),
            new PIDController(RealConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_drive::tankDriveVolts,
            m_drive);

    // Reset odometry to the starting pose of the trajectory.
    m_drive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }
}
