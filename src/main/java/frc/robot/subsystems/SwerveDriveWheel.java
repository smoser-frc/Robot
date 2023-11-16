package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class SwerveDriveWheel {
  // create a class for swervedrive wheels
  public PIDController turnController;
  public MotorController turnMotor;
  public CANcoder turnSensor;
  public MotorController speedMotor;
  public boolean driveInverted;

  public SwerveDriveWheel(
      double P,
      double I,
      double D,
      CANcoder directionSensor,
      MotorController directionMotor,
      MotorController speedMotor,
      boolean speedInverted) {
    turnSensor = directionSensor;
    turnMotor = directionMotor;
    turnController = new PIDController(P, I, D);
    this.speedMotor = speedMotor;
    driveInverted = speedInverted;
  }

  public void setDirection(double setpoint) {
    turnController.reset();

    // calculate the closest angle to the setpoint, as well as make use
    // of the wheel's ability to turn 180 degrees and go backward with a flipped angle
    double currentAngle = turnSensor.getAbsolutePosition().getValue();
    double setPointAngle = closestAngle(currentAngle, setpoint);
    double flippedSetPointAngle = closestAngle(currentAngle, setpoint + 180);

    // if it is easier to go to the setpoint angle, then do that
    // otherwise go to the flipped angle and reverse the drive motor
    if (Math.abs(setPointAngle) <= Math.abs(flippedSetPointAngle)) {
      speedMotor.setInverted(driveInverted);
      turnController.setSetpoint(currentAngle + setPointAngle);
    } else {
      speedMotor.setInverted(!driveInverted);
      turnController.setSetpoint(currentAngle + flippedSetPointAngle);
    }

    double speed = turnController.calculate(currentAngle);
    turnMotor.set(speed);
  }

  public void setSpeed(double speed) {
    // set the speed of the wheels
    speedMotor.set(speed);
  }

  private static double closestAngle(double a, double b) {
    // find the closest angle differnece between two angles
    double dir = (b % 360) - (b % 360);
    if (Math.abs(dir) > 180.0) {
      dir = -(Math.signum(dir) * 360.0) + dir;
    }
    return dir;
  }
}
