// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmForward;
import frc.robot.commands.ArmReverse;
import frc.robot.commands.Autos;
import frc.robot.commands.ClawSwitch;
import frc.robot.commands.DriveTank;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.RealDrive;
import frc.robot.subsystems.SimDrive;

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
  private final Drive m_realDrive = new RealDrive();
  private final SimDrive m_simDrive = new SimDrive();
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
      m_simDrive.setDefaultCommand(
          new DriveTank(
              m_simDrive, leftStick::getLeftY, leftStick::getRightY, Constants.driveSpeed));
      m_drive = m_simDrive;
    } else {
      m_realDrive.setDefaultCommand(
          new DriveTank(
              m_realDrive, leftStick::getLeftY, rightStick::getLeftY, Constants.driveSpeed));
      m_drive = m_realDrive;
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

    codriverA.whileTrue(new ArmForward(m_arm));
    codriverB.whileTrue(new ArmReverse(m_arm));
    leftStickTrigger.whileTrue(new ClawSwitch(m_claw));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
