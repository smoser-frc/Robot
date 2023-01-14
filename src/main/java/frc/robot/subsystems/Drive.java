// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  
  public Drive() {}

  private CANSparkMax leftFront = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax leftBack = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax rightFront = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax rightBack = new CANSparkMax(3, MotorType.kBrushless);

  private RelativeEncoder leftEncoder = leftFront.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 42);
  private RelativeEncoder rightEncoder = rightFront.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 42);

   MotorControllerGroup leftGroup = new MotorControllerGroup(leftBack, leftFront);
   MotorControllerGroup rightGroup = new MotorControllerGroup(rightFront, rightBack);

  private DifferentialDrive driveTrain = new DifferentialDrive(leftGroup, rightGroup);

  private DifferentialDrivetrainSim dtSim = new DifferentialDrivetrainSim(DCMotor.getNEO(2), 8.451, 6, 57, Units.inchesToMeters(3), Units.inchesToMeters(22.5), null);


  public void setTankDrive(DoubleSupplier lSpeed, DoubleSupplier rSpeed, Double pOutput){

    driveTrain.tankDrive(lSpeed.getAsDouble() * pOutput, rSpeed.getAsDouble() * pOutput);

  }

  public CANSparkMax getLeft(){
    return leftFront;
  }

public CANSparkMax getRight(){
  return rightFront;
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    REVPhysicsSim.getInstance().run();
  }
}
