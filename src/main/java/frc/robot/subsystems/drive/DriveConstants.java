// Copyright (c) 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD license that can be found in
// the LICENSE file at the root of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/**
 * Drivetrain geometry, device IDs, gear ratios, and control gains.
 *
 * <p>These values come from the 2024/2025 robot: four SDS MK4n modules with REV NEO motors, Spark
 * MAX controllers, analog absolute encoders on the roboRIO, a Pigeon 2, and a REV PDH. Verify every
 * value against the physical robot before the first drive.
 */
public final class DriveConstants {
  private DriveConstants() {}

  /** Free speed is about 4.8 m/s. 4.5 is near the top; lower it if acceleration is poor. */
  public static final double maxSpeedMetersPerSec = 4.5;

  public static final double maxAngularSpeedRadPerSec = 3.0 * Math.PI;

  public static final double trackWidth = Units.inchesToMeters(20.5);
  public static final double wheelBase = Units.inchesToMeters(20.5);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

  // Order is FL, FR, BL, BR everywhere in this package.
  public static final Translation2d[] moduleTranslations = {
    new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
    new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
    new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
    new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
  };

  // SDS MK4n with NEO motors.
  public static final double wheelRadiusMeters = Units.inchesToMeters(3.75) / 2.0;

  /** Motor rotations per wheel rotation. MK4n L2+. */
  public static final double driveMotorReduction = 5.9;

  /** Motor rotations per module rotation. MK4n uses 16T steering pinions (18.75:1). */
  public static final double turnMotorReduction = 18.75;

  public static final double driveEncoderPositionFactor =
      2.0 * Math.PI * wheelRadiusMeters / driveMotorReduction;
  public static final double driveEncoderVelocityFactor = driveEncoderPositionFactor / 60.0;
  public static final double turnEncoderPositionFactor = 2.0 * Math.PI / turnMotorReduction;
  public static final double turnEncoderVelocityFactor = turnEncoderPositionFactor / 60.0;

  public static final int driveMotorCurrentLimit = 40;
  public static final int turnMotorCurrentLimit = 40;

  // Start here, then tune with SysId or a simple velocity sweep.
  public static final double driveKp = 0.5;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.1;
  public static final double driveKv =
      12.0 / (DCMotor.getNEO(1).freeSpeedRadPerSec * wheelRadiusMeters / driveMotorReduction);

  public static final double turnKp = 0.25;
  public static final double turnKd = 0.0;

  // Simulation gains only; they do not affect the real robot.
  public static final double driveSimKp = 1.0;
  public static final double driveSimKv = 2.5;
  public static final double turnSimKp = 5.0;

  public static final int pigeonCanId = Constants.CAN.PIGEON;

  /**
   * Per-module hardware configuration.
   *
   * <p>The analog absolute encoder channels are the roboRIO analog inputs 0-3. Offsets are the
   * measured module zero offsets in radians. Re-measure after replacing an encoder or module.
   */
  public record ModuleHardware(
      int driveId,
      int turnId,
      int absoluteEncoderChannel,
      double absoluteEncoderOffsetRad,
      boolean driveMotorInverted,
      boolean turnMotorInverted,
      boolean absoluteEncoderReversed) {}

  public static final ModuleHardware[] modules = {
    // Front left
    new ModuleHardware(
        Constants.CAN.FRONT_LEFT_DRIVE,
        Constants.CAN.FRONT_LEFT_TURN,
        0,
        -6.903,
        true,
        true,
        false),
    // Front right
    new ModuleHardware(
        Constants.CAN.FRONT_RIGHT_DRIVE,
        Constants.CAN.FRONT_RIGHT_TURN,
        1,
        -1.909,
        true,
        true,
        false),
    // Back left
    new ModuleHardware(
        Constants.CAN.BACK_LEFT_DRIVE, Constants.CAN.BACK_LEFT_TURN, 2, 3.035, true, true, false),
    // Back right
    new ModuleHardware(
        Constants.CAN.BACK_RIGHT_DRIVE, Constants.CAN.BACK_RIGHT_TURN, 3, 0.526, true, true, false)
  };
}
