// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AbsoluteDrive;
import frc.robot.commands.LaunchWithVelo;
import frc.robot.commands.LaunchWithVeloAuton;
import frc.robot.commands.PrimeIndex;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.ToggleLaunchPIDS;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final SwerveSubsystem m_swerve =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerves/KrakenSwerve"));
  private final Intake m_intake = new Intake();
  private final Index m_index = new Index();
  private final Launcher m_launch = new Launcher();
  private final Climb m_climb = new Climb();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController driver = new XboxController(0);
  private final XboxController coDriver = new XboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Create commands for PathPlanner
    NamedCommands.registerCommand(
        "launch", new LaunchWithVeloAuton(m_launch, m_index, Constants.Launch.speedCloseSpeaker));
    NamedCommands.registerCommand("intake", new ToggleIntake(m_intake));
    NamedCommands.registerCommand("index", new PrimeIndex(m_index));

    // Configure the trigger bindings
    configureBindings();

    AbsoluteDrive closedAbsoluteDrive =
        new AbsoluteDrive(
            m_swerve,
            // Applies deadbands and inverts controls because joysticks
            // are back-right positive while robot
            // controls are front-left positive
            () -> MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () -> driver.getRightX(),
            () -> -driver.getRightY());

    m_swerve.setDefaultCommand(
        !RobotBase.isSimulation() ? closedAbsoluteDrive : closedAbsoluteDrive);

    m_index.setDefaultCommand(m_index.manualIntake(coDriver::getRightY));

    m_climb.setDefaultCommand(m_climb.setWinchCommand(coDriver::getLeftY));

    // add auto options
    m_chooser.setDefaultOption("Test Drive", m_swerve.getAutonomousCommand("Test Drive"));

    m_chooser.addOption(
        "2 Note Midfield Auton Blue", m_swerve.getAutonomousCommand("2 Note Midfield Auton Blue"));
    m_chooser.addOption(
        "2 Note Midfield Auton Red", m_swerve.getAutonomousCommand("2 Note Midfield Auton Red"));
    m_chooser.addOption("4 Note Auton Blue", m_swerve.getAutonomousCommand("4 Note Auton Blue"));
    m_chooser.addOption("4 Note Auton Red", m_swerve.getAutonomousCommand("4 Note Auton Red"));
    m_chooser.addOption("Turn Auto", m_swerve.getAutonomousCommand("Turn Auto"));
    m_chooser.addOption("Drive and Turn", m_swerve.getAutonomousCommand("drive and turn"));
    SmartDashboard.putData(m_chooser);
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
    JoystickButton leftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    leftBumper.onTrue(new ToggleIntake(m_intake));

    JoystickButton rightBumper =
        new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    rightBumper.onTrue(new PrimeIndex(m_index));

    JoystickButton a = new JoystickButton(driver, XboxController.Button.kA.value);
    a.whileTrue(m_swerve.alignCommand());

    JoystickButton b = new JoystickButton(driver, XboxController.Button.kB.value);
    b.whileTrue(new LaunchWithVelo(m_launch, m_index, 0, true));

    JoystickButton y = new JoystickButton(driver, XboxController.Button.kY.value);
    y.onTrue(m_launch.switchAngleCommand());

    JoystickButton coA = new JoystickButton(coDriver, XboxController.Button.kA.value);
    coA.onTrue(new ToggleLaunchPIDS(m_launch));

    JoystickButton x = new JoystickButton(driver, XboxController.Button.kX.value);
    x.onTrue(m_swerve.updatePositionCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  // autonselect
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    m_swerve.setMotorBrake(brake);
  }
}
