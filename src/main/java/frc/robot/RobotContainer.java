// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RealDrive;
import frc.robot.subsystems.SimDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Robot.isReal()) {
      m_robotDrive = new RealDrive();
    } else {
      m_robotDrive = new SimDrive();
    }

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () ->
                m_robotDrive.arcadeDrive(
                    -m_driverController.getLeftY(), -m_driverController.getLeftX()),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link PS4Controller}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    /*new JoystickButton(m_driverController, Button.kR1.value)
    .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
    .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));*/

    // Stabilize robot to drive straight with gyro when left bumper is held
    /*new JoystickButton(m_driverController, Button.kL1.value)
    .whileTrue(
        new PIDCommand(
            new PIDController(
                DriveConstants.kStabilizationP,
                DriveConstants.kStabilizationI,
                DriveConstants.kStabilizationD),
            // Close the loop on the turn rate
            m_robotDrive::getTurnRate,
            // Setpoint is 0
            0,
            // Pipe the output to the turning controls
            output -> m_robotDrive.arcadeDrive(-m_driverController.getLeftY(), output),
            // Require the robot drive
            m_robotDrive));*/

    // Turn to 90 degrees when the 'X' button is pressed, with a 5 second timeout
    // new JoystickButton(m_driverController, Button.kCross.value)
    // .onTrue(new TurnToAngle(90, m_robotDrive).withTimeout(5));

    // Turn to -90 degrees with a profile when the Circle button is pressed, with a 5 second timeout
    // new JoystickButton(m_driverController, Button.kCircle.value)
    // .onTrue(new TurnToAngleProfiled(-90, m_robotDrive).withTimeout(5));

    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(new DriveDistance(6 * Math.PI * 4, m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .onTrue(new DriveDistance(-6 * Math.PI * 4, m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .onTrue(new TurnToAngle(180, m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new TurnToAngle(0, m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // no auto
    return new InstantCommand();
  }

  public void autonomousInit() {
    m_robotDrive.resetPosition();
  }
}
