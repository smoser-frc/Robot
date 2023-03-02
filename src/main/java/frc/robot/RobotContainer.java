// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.BasicAuto;
import frc.robot.commands.DriveTank;
import frc.robot.commands.ManualArm;
import frc.robot.commands.ManualClaw;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.SetClawPosition;
import frc.robot.commands.SwitchGears;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GearShifter;
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
  // private final Arm m_arm = new Arm();
  // private final Claw m_claw = new Claw();
  private final GearShifter m_gearShifter = new GearShifter();
  private Constants m_constants;
  private final Constants m_realConstants = new RealConstants();
  private final Constants m_simConstants = new SimConstants();
  private final Arm m_arm = new Arm();
  private final Claw m_claw = new Claw();

  SendableChooser<Command> m_Chooser = new SendableChooser<>();

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
              m_drive, leftStick::getLeftY, rightStick::getLeftY, m_constants.driveSpeed));
      m_arm.setDefaultCommand(new ManualArm(m_arm, coDriver::getLeftY));
      m_claw.setDefaultCommand(new ManualClaw(m_claw, coDriver::getRightY));
    }
    
    Command basicAuto = new BasicAuto(m_arm, m_claw, m_drive);

    m_Chooser.setDefaultOption("Basic Auton", basicAuto);

    SmartDashboard.putData(m_Chooser);
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
    final JoystickButton rightStickTrigger = new JoystickButton(rightStick, 1);
    final JoystickButton coDriverA = new JoystickButton(coDriver, XboxController.Button.kA.value);
    final JoystickButton coDriverB = new JoystickButton(coDriver, XboxController.Button.kB.value);
    final JoystickButton coDriverX = new JoystickButton(coDriver, XboxController.Button.kX.value);
    final JoystickButton coDriverY = new JoystickButton(coDriver, XboxController.Button.kY.value);

    rightStickTrigger.whileTrue(new SwitchGears(m_gearShifter));

    coDriverA.onTrue(new SetArmPosition(m_arm, 190));
    coDriverB.onTrue(new SetArmPosition(m_arm, 15));
    coDriverX.onTrue(new SetClawPosition(m_claw, 90));
    coDriverY.onTrue(new SetClawPosition(m_claw, 10));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    Command autoCommand = m_Chooser.getSelected();

    return autoCommand;
  }
}
