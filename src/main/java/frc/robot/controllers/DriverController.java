package frc.robot.controllers;

/**
 * Driver input abstraction. The drivetrain only needs these six signals, so replacing an 8BitDo
 * Ultimate (XInput) with another controller is a one-file change.
 */
public interface DriverController {
  /** Forward positive. */
  double forward();

  /** Left positive. */
  double strafe();

  /** Counter-clockwise positive. */
  double rotation();

  boolean slowMode();

  boolean robotOrientedToggle();

  boolean xLock();

  boolean zeroHeading();
}
