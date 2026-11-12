// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Robot-wide constants that are not specific to the drivetrain.
 *
 * <p>Everything in this file is intentionally small. Drivetrain geometry, gains, and device IDs
 * live in {@link frc.robot.subsystems.drive.DriveConstants}.
 */
public final class Constants {
  private Constants() {}

  /** Change to {@link Mode#REPLAY} to replay an AdvantageKit log. */
  public static final Mode simMode = Mode.SIM;

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulation. */
    SIM,
    /** Replaying a log file with AdvantageKit. */
    REPLAY
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final double DEADBAND = 0.1;
    public static final double SLOW_MODE_MULTIPLIER = 0.3;
  }

  public static final class CAN {
    // Drivetrain IDs match the 2024/2025 robot. Verify these before deploying.
    public static final int FRONT_LEFT_DRIVE = 10;
    public static final int FRONT_LEFT_TURN = 20;
    public static final int FRONT_RIGHT_DRIVE = 11;
    public static final int FRONT_RIGHT_TURN = 21;
    public static final int BACK_LEFT_DRIVE = 12;
    public static final int BACK_LEFT_TURN = 22;
    public static final int BACK_RIGHT_DRIVE = 13;
    public static final int BACK_RIGHT_TURN = 23;

    public static final int PIGEON = 31;
    public static final int PDH = 1;
  }

  public static final class FieldConstants {
    // 2025 Reefscape field. Used only by AllianceFlipUtil.
    public static final double FIELD_LENGTH_METERS = 16.54;
    public static final double FIELD_WIDTH_METERS = 8.07;
  }
}
