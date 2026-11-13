package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
   * Field-relative drive with deadband, cubic response, slow mode, and cardinal lock.
   *
   * <p>Cardinal lock holds the nearest 90-degree heading while the driver strafes. Rotation input
   * always wins; when it stops, the lock holds the new heading.
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier omega,
      BooleanSupplier slowMode,
      BooleanSupplier cardinalLock) {
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
              double omegaSpeed =
                  lock.apply(
                      cardinalLock.getAsBoolean(),
                      omegaInput,
                      drive.getRotation(),
                      drive.getMaxAngularSpeedRadPerSec() * multiplier);
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

  static Rotation2d nearestCardinal(Rotation2d current) {
    return Rotation2d.fromDegrees(Math.round(current.getDegrees() / 90.0) * 90.0);
  }

  private static double cubic(double value) {
    return MathUtil.clamp(value * value * value, -1.0, 1.0);
  }

  /** One lock per command instance; it keeps its target while the button is held. */
  static final class CardinalLock {
    private final PIDController pid =
        new PIDController(DriveConstants.cardinalKp, 0.0, DriveConstants.cardinalKd);
    private Rotation2d target;

    CardinalLock() {
      pid.enableContinuousInput(-Math.PI, Math.PI);
    }

    double apply(boolean active, double omegaInput, Rotation2d current, double maxOmega) {
      if (!active) {
        target = null;
        pid.reset();
        return omegaInput;
      }
      if (target == null) {
        target = nearestCardinal(current);
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
