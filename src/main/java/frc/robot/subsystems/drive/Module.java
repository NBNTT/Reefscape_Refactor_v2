// Copyright (c) 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD license that can be found in
// the LICENSE file at the root of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

/**
 * One swerve module.
 *
 * <p>If a module loses a motor or its turn encoder, it coasts both motors and freezes its last
 * valid position/state. Drive keeps using the frozen values so the other three modules continue to
 * steer and estimate pose. When the module is healthy again it re-brakes and re-seeds its encoder.
 */
public class Module {
  private static final int FAILURE_CYCLES = 6;

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderAlert;
  private final Alert absoluteEncoderAlert;
  private final Alert turnReseedAlert;
  private final Alert failedAlert;

  private boolean failed = false;
  private int failureCounter = 0;
  private boolean odometryReset = false;
  private SwerveModulePosition lastPosition = new SwerveModulePosition();
  private SwerveModuleState lastState = new SwerveModuleState();

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    driveDisconnectedAlert =
        new Alert("Disconnected drive motor on module " + index + ".", AlertType.kError);
    turnDisconnectedAlert =
        new Alert("Disconnected turn motor on module " + index + ".", AlertType.kError);
    turnEncoderAlert =
        new Alert("Turn encoder not seeded on module " + index + ".", AlertType.kError);
    absoluteEncoderAlert =
        new Alert("Absolute encoder invalid on module " + index + ".", AlertType.kWarning);
    turnReseedAlert =
        new Alert("Turn encoder repeatedly re-seeding on module " + index + ".", AlertType.kError);
    failedAlert =
        new Alert(
            "Module " + index + " failed; coasting and frozen in odometry.", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);

    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderAlert.set(!inputs.turnEncoderSeeded);
    absoluteEncoderAlert.set(!inputs.turnAbsoluteEncoderValid);
    turnReseedAlert.set(inputs.turnReseedCount > 3);

    if (!isReady()) {
      if (++failureCounter >= FAILURE_CYCLES && !failed) {
        failed = true;
        io.setCoast(true);
        lastState = new SwerveModuleState(0.0, lastPosition.angle);
      }
    } else {
      failureCounter = 0;
      if (failed) {
        failed = false;
        odometryReset = true;
        io.setCoast(false);
        io.resetTurnEncoder();
      }
    }

    if (isReady()) {
      lastPosition = new SwerveModulePosition(inputs.drivePositionMeters, inputs.turnPosition);
      lastState = new SwerveModuleState(inputs.driveVelocityMetersPerSec, inputs.turnPosition);
    }
    failedAlert.set(failed);
  }

  /** Runs the module at the requested state. The state is optimized and cosine-scaled. */
  public void runSetpoint(SwerveModuleState state) {
    if (failed) {
      return;
    }
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPosition);
    io.setDriveVelocity(state.speedMetersPerSecond);
    io.setTurnPosition(state.angle);
  }

  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  public void resetTurnEncoder() {
    io.resetTurnEncoder();
  }

  /** True when the module can safely accept drive and turn setpoints. */
  public boolean isReady() {
    return inputs.driveConnected && inputs.turnConnected && inputs.turnEncoderSeeded;
  }

  public boolean isFailed() {
    return failed;
  }

  /** True once after recovery, so Drive can drop the position jump from the frozen encoder. */
  public boolean consumeOdometryReset() {
    boolean reset = odometryReset;
    odometryReset = false;
    return reset;
  }

  public Rotation2d getAngle() {
    return failed ? lastPosition.angle : inputs.turnPosition;
  }

  public double getPositionMeters() {
    return failed ? lastPosition.distanceMeters : inputs.drivePositionMeters;
  }

  public SwerveModulePosition getPosition() {
    return failed
        ? lastPosition
        : new SwerveModulePosition(inputs.drivePositionMeters, inputs.turnPosition);
  }

  public SwerveModuleState getState() {
    return failed
        ? lastState
        : new SwerveModuleState(inputs.driveVelocityMetersPerSec, inputs.turnPosition);
  }
}
