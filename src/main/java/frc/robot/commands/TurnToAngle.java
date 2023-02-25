// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/** A command that will move the robot forward */
public class TurnToAngle extends CommandBase {

  private DriveSubsystem m_drive;
  private PIDController m_PidControl;
  private double m_angle;
  private double setPoint;

  /**
   * Moves the robot a specified distance forward.
   *
   * @param angle Angle to go to.
   * @param drive The drive subsystem to use
   */
  public TurnToAngle(double angle, DriveSubsystem drive) {
    m_drive = drive;
    m_angle = angle;

    m_PidControl =
        new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD);
    m_PidControl.setTolerance(
        Constants.DriveConstants.kTurnToleranceDeg,
        Constants.DriveConstants.kTurnRateToleranceDegPerS);

    addRequirements(drive);
  }

  @Override
  public void initialize() {

    double heading = m_drive.getHeading();
    double target = m_angle + heading;
    System.out.println("At " + heading + " going to " + target);

    setPoint = heading + m_angle;

    m_drive.setMaxOutput(1.0);

    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
  }

  @Override
  public void execute() {
    double heading = m_drive.getHeading();
    double output = m_PidControl.calculate(heading, setPoint);
    m_drive.arcadeDrive(0, output);
    SmartDashboard.putNumber("Heading", heading);
    SmartDashboard.putNumber("TurnOutput", output);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.setMaxOutput(1);
    System.out.println(
        "At " + m_drive.getHeading() + " target " + setPoint + " Done!");
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return m_PidControl.atSetpoint();
  }
}
