// Copyright (c) 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD license that can be found in
// the LICENSE file at the root of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for one swerve module. */
public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public boolean turnConnected = false;
    public Rotation2d turnPosition = Rotation2d.kZero;
    public double turnAbsolutePositionRad = 0.0;
    public double turnAbsoluteVoltage = 0.0;
    public boolean turnAbsoluteEncoderValid = false;
    public boolean turnEncoderSeeded = false;
    public int turnReseedCount = 0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Runs the drive motor at the requested wheel speed. */
  public default void setDriveVelocity(double velocityMetersPerSec) {}

  /** Runs the turn motor to the requested module angle. */
  public default void setTurnPosition(Rotation2d rotation) {}

  /** Runs the drive motor at a fixed voltage. Used for characterization and stopping. */
  public default void setDriveOpenLoop(double volts) {}

  /** Runs the turn motor at a fixed voltage. Used for stopping. */
  public default void setTurnOpenLoop(double volts) {}

  /** Re-seeds the turn motor's relative encoder from the absolute encoder. */
  public default void resetTurnEncoder() {}
}
