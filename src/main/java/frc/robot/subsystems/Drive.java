// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Drive extends SubsystemBase {
  
  protected final Field2d m_field = new Field2d();
  protected DifferentialDrive driveTrain;
  protected MotorControllerGroup leftGroup;
  protected MotorControllerGroup rightGroup;
  protected DifferentialDriveOdometry odometer;

  public Drive() {
    SmartDashboard.putData("field", m_field);
  }

  public void setTankDrive(DoubleSupplier lSpeed, DoubleSupplier rSpeed, Double pOutput){
    driveTrain.tankDrive(lSpeed.getAsDouble() * pOutput, rSpeed.getAsDouble() * pOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
