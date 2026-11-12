// Copyright (c) 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD license that can be found in
// the LICENSE file at the root of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** Driver-facing drivetrain commands. */
public final class DriveCommands {
  private DriveCommands() {}

  /**
   * Field-relative joystick drive with deadband, cubic response, and an optional slow mode.
   *
   * @param x forward positive
   * @param y left positive
   * @param omega counter-clockwise positive
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier omega,
      BooleanSupplier slowMode) {
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
              double omegaSpeed =
                  cubic(applyDeadband(omega.getAsDouble(), OperatorConstants.DEADBAND))
                      * drive.getMaxAngularSpeedRadPerSec()
                      * multiplier;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      xSpeed, ySpeed, omegaSpeed, drive.getRotation()));
            })
        .withName("Joystick Drive")
        .finallyDo(drive::stop);
  }

  /** Holds the modules in an X pattern. */
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
}
