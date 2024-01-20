// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;

public class SwerveCommand extends Command {
  /** Creates a new SwerveCommand. */
  private final SwerveSubsystem swerve;

  private final DoubleSupplier dX;
  private final DoubleSupplier dY;
  private final DoubleSupplier omega;
  private final BooleanSupplier driveMode;
  private final SwerveController controller;

  public SwerveCommand(
      SwerveSubsystem swerve,
      DoubleSupplier dX,
      DoubleSupplier dY,
      DoubleSupplier omega,
      BooleanSupplier driveMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.dX = dX;
    this.dY = dY;
    this.omega = omega;
    this.driveMode = driveMode;
    this.controller = swerve.getSwerveController();
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVelocity = Math.pow(dX.getAsDouble(), 3);
    double yVelocity = Math.pow(dY.getAsDouble(), 3);
    double angleVelocity = Math.pow(omega.getAsDouble(), 3);

    swerve.drive(
        new Translation2d(xVelocity * swerve.maxSpeed, yVelocity * swerve.maxSpeed),
        angleVelocity * controller.config.maxAngularVelocity,
        driveMode.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
