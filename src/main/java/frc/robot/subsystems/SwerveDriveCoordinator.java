package frc.robot.subsystems;

public class SwerveDriveCoordinator {

  SwerveDriveWheel leftFront;
  SwerveDriveWheel leftBack;
  SwerveDriveWheel rightFront;
  SwerveDriveWheel rightBack;

  public SwerveDriveCoordinator(
      SwerveDriveWheel lFront,
      SwerveDriveWheel lBack,
      SwerveDriveWheel rFront,
      SwerveDriveWheel rBack) {
    leftFront = lFront;
    leftBack = lBack;
    rightFront = rFront;
    rightBack = rBack;
  }

  public void turnInPlace(double turnPower) {
    leftFront.setDirection(135.0);
    leftBack.setDirection(45.0);
    rightFront.setDirection(-45.0);
    rightBack.setDirection(-135.0);

    leftFront.setSpeed(turnPower);
    leftBack.setSpeed(turnPower);
    rightFront.setSpeed(turnPower);
    rightBack.setSpeed(turnPower);
  }

  public void translateTurn(double direction, double translatePower, double turnPower) {
    double turnAngle = turnPower * 45.0;

    // if the left front wheel is in the front
    if (closestAngle(direction, 135.0) >= 90.0) {
      leftFront.setDirection(direction + turnAngle);
    }
    // if it's in the back
    else {
      leftFront.setDirection(direction - turnAngle);
    }
    // if the left back wheel is in the front
    if (closestAngle(direction, 225.0) > 90.0) {
      leftBack.setDirection(direction + turnAngle);
    }
    // if it's in the back
    else {
      leftBack.setDirection(direction - turnAngle);
    }
    // if the right front wheel is in the front
    if (closestAngle(direction, 45.0) > 90.0) {
      rightFront.setDirection(direction + turnAngle);
    }
    // if it's in the back
    else {
      rightFront.setDirection(direction - turnAngle);
    }
    // if the right back wheel is in the front
    if (closestAngle(direction, 315.0) >= 90.0) {
      rightBack.setDirection(direction + turnAngle);
    }
    // if it's in the back
    else {
      rightBack.setDirection(direction - turnAngle);
    }

    leftFront.setSpeed(translatePower);
    leftBack.setSpeed(translatePower);
    rightFront.setSpeed(translatePower);
    rightBack.setSpeed(translatePower);
  }

  private static double closestAngle(double a, double b) {
    // find the closest angle differnece between two angles
    double dir = (b % 360) - (b % 360);
    if (Math.abs(dir) > 180.0) {
      dir = -(Math.signum(dir) * 360.0) + dir;
    }
    return dir;
  }

  public void setSwerveDrive(double direction, double translatePower, double turnPower) {
    if ((translatePower == 0) && (turnPower != 0)) {
      turnInPlace(turnPower);
    } else {
      translateTurn(direction, translatePower, turnPower);
    }
  }
}
