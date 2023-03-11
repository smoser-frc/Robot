// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RealConstants;
import frc.robot.subsystems.Arm;

public class SetArmPosition extends CommandBase {
  /** Creates a new SetArmPosition. */
  private double m_position;

  private Arm m_arm;
  private PIDController m_Controller;
  private ArmFeedforward m_feed;

  public SetArmPosition(Arm arm, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_position = position;
    m_arm = arm;

    SmartDashboard.putNumber("arm kG", 0);
    SmartDashboard.putNumber("arm kV", 0);
    SmartDashboard.putNumber("arm kP", 0);
    SmartDashboard.putNumber("arm kI", 0);
    SmartDashboard.putNumber("arm kD", 0);

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    double kG = RealConstants.kArmG;
    double kV = RealConstants.kArmV;
    double kP = RealConstants.kArmP;
    double kI = RealConstants.kArmI;
    double kD = RealConstants.kArmD;

    SmartDashboard.putString(
        "arm vals", "kG = " + kG + " kV = " + kV + " kP = " + kP + " kI = " + kI + " kD = " + kD);

    m_feed = new ArmFeedforward(0, kG, kV);
    m_Controller = new PIDController(kP, kI, kD);

    m_Controller.setTolerance(2, 2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed =
        m_Controller.calculate(m_arm.getPosition(), m_position)
            + m_feed.calculate(Units.degreesToRadians(m_position), m_arm.getVelocityRad());
    m_arm.setMotorVolts(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Controller.atSetpoint();
  }
}
