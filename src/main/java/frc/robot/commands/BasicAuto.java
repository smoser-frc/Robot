// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BasicAuto extends SequentialCommandGroup {
  /** Creates a new BasicAuto. */
  public BasicAuto(Arm arm, Claw claw, Drive drive) {
    addCommands(
        new SetArmPosition(arm, -175),
        new SetClawPosition(claw, -25),
        new SetArmPosition(arm, -20),
        new DriveDistance(Units.inchesToMeters(6 * 12), drive));
  }
}
