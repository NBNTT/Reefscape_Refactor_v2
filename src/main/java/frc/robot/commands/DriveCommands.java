package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** Driver-facing drivetrain commands. */
public final class DriveCommands {
  private DriveCommands() {}

  /**
   * Field-relative drive with deadband, cubic response, slow mode, and always-on heading hold.
   *
   * <p>When the rotation stick is centered, the lock holds the last heading while the robot
   * strafes. Rotation input always wins; when it stops, the lock holds the new heading.
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier omega,
      BooleanSupplier slowMode) {
    CardinalLock lock = new CardinalLock();
    return drive
        .run(
            () -> {
              double multiplier =
                  slowMode.getAsBoolean() ? OperatorConstants.SLOW_MODE_MULTIPLIER : 1.0;
              double xSpeed =
                  cubic(applyDeadband(x.getAsDouble(), OperatorConstants.DEADBAND))
                      * drive.getMaxLinearSpeedMetersPerSec()
                      * multiplier;
              double ySpeed =
                  cubic(applyDeadband(y.getAsDouble(), OperatorConstants.DEADBAND))
                      * drive.getMaxLinearSpeedMetersPerSec()
                      * multiplier;
              double omegaInput =
                  cubic(applyDeadband(omega.getAsDouble(), OperatorConstants.DEADBAND))
                      * drive.getMaxAngularSpeedRadPerSec()
                      * multiplier;
              if (DriverStation.isDisabled()) {
                lock.reset();
              }
              double omegaSpeed =
                  lock.apply(
                      omegaInput,
                      drive.getRotation(),
                      drive.getMaxAngularSpeedRadPerSec() * multiplier,
                      drive.getHeadingResetCounter());
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      xSpeed, ySpeed, omegaSpeed, drive.getRotation()));
            })
        .withName("Joystick Drive")
        .finallyDo(drive::stop);
  }

  static double applyDeadband(double value, double deadband) {
    if (!Double.isFinite(value)) {
      return 0.0;
    }
    if (Math.abs(value) < deadband) {
      return 0.0;
    }
    return Math.copySign((Math.abs(value) - deadband) / (1.0 - deadband), value);
  }

  private static double cubic(double value) {
    return MathUtil.clamp(value * value * value, -1.0, 1.0);
  }

  /** Holds the heading captured when the rotation stick was last centered. */
  static final class CardinalLock {
    private final PIDController pid =
        new PIDController(DriveConstants.cardinalKp, 0.0, DriveConstants.cardinalKd);
    private Rotation2d target;
    private int lastResetCounter = -1;

    CardinalLock() {
      pid.enableContinuousInput(-Math.PI, Math.PI);
    }

    void reset() {
      target = null;
      pid.reset();
    }

    double apply(double omegaInput, Rotation2d current, double maxOmega, int resetCounter) {
      if (target == null || resetCounter != lastResetCounter) {
        lastResetCounter = resetCounter;
        target = current;
        pid.reset();
      }
      if (Math.abs(omegaInput) > 1e-3) {
        target = current;
        pid.reset();
        return omegaInput;
      }
      return MathUtil.clamp(
          pid.calculate(current.getRadians(), target.getRadians()), -maxOmega, maxOmega);
    }
  }
}
