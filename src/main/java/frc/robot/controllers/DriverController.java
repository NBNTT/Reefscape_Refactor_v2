package frc.robot.controllers;

/**
 * Driver input abstraction. The drivetrain only needs these inputs, so replacing an 8BitDo Ultimate
 * (XInput) with another controller is a one-file change.
 */
public interface DriverController {
  /** Forward positive. */
  double forward();

  /** Left positive. */
  double strafe();

  /** Counter-clockwise positive. */
  double rotation();

  boolean slowMode();

  boolean fieldOriented();

  boolean robotOriented();

  boolean xLock();

  boolean zeroHeading();
}
