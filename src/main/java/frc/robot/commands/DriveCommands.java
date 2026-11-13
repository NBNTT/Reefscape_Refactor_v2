package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
   * Field- or robot-oriented drive with deadband, cubic response, smooth start, quick stop, slow
   * mode, and always-on heading hold.
   *
   * <p>When the rotation stick is centered, the lock holds the last heading even at zero
   * translation. Rotation input always wins; when it stops, the lock holds the new heading.
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier omega,
      BooleanSupplier slowMode,
      BooleanSupplier robotOriented) {
    CardinalLock lock = new CardinalLock();
    FastStopSlew xSlew = new FastStopSlew();
    FastStopSlew ySlew = new FastStopSlew();
    FastStopSlew omegaSlew = new FastStopSlew();

    return drive
        .run(
            () -> {
              double multiplier =
                  slowMode.getAsBoolean() ? OperatorConstants.SLOW_MODE_MULTIPLIER : 1.0;
              double now = Timer.getFPGATimestamp();
              if (DriverStation.isDisabled()) {
                xSlew.reset();
                ySlew.reset();
                omegaSlew.reset();
                lock.reset();
              }
              double xSpeed =
                  xSlew.calculate(
                      cubic(applyDeadband(x.getAsDouble(), OperatorConstants.DEADBAND))
                          * drive.getMaxLinearSpeedMetersPerSec()
                          * multiplier,
                      now,
                      DriveConstants.maxLinearAccelMetersPerSec2,
                      DriveConstants.maxLinearDecelMetersPerSec2);
              double ySpeed =
                  ySlew.calculate(
                      cubic(applyDeadband(y.getAsDouble(), OperatorConstants.DEADBAND))
                          * drive.getMaxLinearSpeedMetersPerSec()
                          * multiplier,
                      now,
                      DriveConstants.maxLinearAccelMetersPerSec2,
                      DriveConstants.maxLinearDecelMetersPerSec2);
              double omegaInput =
                  omegaSlew.calculate(
                      cubic(applyDeadband(omega.getAsDouble(), OperatorConstants.DEADBAND))
                          * drive.getMaxAngularSpeedRadPerSec()
                          * multiplier,
                      now,
                      DriveConstants.maxAngularAccelRadPerSec2,
                      DriveConstants.maxAngularDecelRadPerSec2);

              double omegaSpeed =
                  lock.apply(
                      omegaInput,
                      drive.getRotation(),
                      drive.getMaxAngularSpeedRadPerSec() * multiplier,
                      drive.getHeadingResetCounter());

              drive.runVelocity(
                  robotOriented.getAsBoolean()
                      ? new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed)
                      : ChassisSpeeds.fromFieldRelativeSpeeds(
                          xSpeed, ySpeed, omegaSpeed, drive.getRotation()));
            })
        .beforeStarting(
            () -> {
              xSlew.reset();
              ySlew.reset();
              omegaSlew.reset();
            })
        .withName("Joystick Drive")
        .finallyDo(drive::stop);
  }

  /** Holds the drivetrain in an X pattern to resist pushing. */
  public static Command lockWheels(Drive drive) {
    return drive.run(drive::stopWithX).withName("X-Lock");
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

  /** Asymmetric rate limiter: slow accel, fast decel, exact target at full speed. */
  static final class FastStopSlew {
    private double value = 0.0;
    private double lastTime = Double.NaN;

    void reset() {
      value = 0.0;
      lastTime = Double.NaN;
    }

    double calculate(double target, double now, double accel, double decel) {
      double dt = Double.isNaN(lastTime) ? 0.02 : MathUtil.clamp(now - lastTime, 0.005, 0.1);
      lastTime = now;
      boolean slowing = Math.abs(target) < Math.abs(value) || value * target < 0.0;
      double maxDelta = (slowing ? decel : accel) * dt;
      value += MathUtil.clamp(target - value, -maxDelta, maxDelta);
      return value;
    }
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
