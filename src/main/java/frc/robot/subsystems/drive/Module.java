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
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/**
 * One swerve module.
 *
 * <p>If a module loses a motor or its turn encoder, it coasts both motors and freezes its last
 * valid position/state. Drive keeps using the frozen values so the other three modules continue to
 * steer and estimate pose. When the module is healthy again it re-brakes and re-seeds its encoder.
 */
public class Module {
  private static final int FAILURE_CYCLES = 3;

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final Alert moduleReadyAlert;
  private final Alert absoluteEncoderAlert;
  private final Alert turnReseedAlert;

  private boolean failed = false;
  private boolean coastSet = false;
  private double lastCoastAttempt = Double.NaN;
  private int failureCounter = 0;
  private boolean odometryReset = false;
  private SwerveModulePosition lastPosition = new SwerveModulePosition();

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    moduleReadyAlert =
        new Alert(
            "Module " + index + " not ready; check drive, turn, and encoder.", AlertType.kError);
    absoluteEncoderAlert =
        new Alert("Absolute encoder invalid on module " + index + ".", AlertType.kWarning);
    turnReseedAlert =
        new Alert("Turn encoder repeatedly re-seeding on module " + index + ".", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);

    boolean inputsOk = inputsReady();
    moduleReadyAlert.set(!isReady());
    absoluteEncoderAlert.set(!inputs.turnAbsoluteEncoderValid);
    turnReseedAlert.set(inputs.turnReseedCount > 3);

    if (!inputsOk) {
      if (++failureCounter >= FAILURE_CYCLES && !failed) {
        failed = true;
        coastSet = io.setCoast(true);
        lastCoastAttempt = Double.NaN;
      } else if (failed && !coastSet && coastAttemptDue()) {
        coastSet = io.setCoast(true);
      }
    } else {
      failureCounter = 0;
      if (failed && coastAttemptDue() && io.setCoast(false)) {
        failed = false;
        coastSet = false;
        odometryReset = true;
        io.resetTurnEncoder();
      }
    }

    if (!failed && inputsOk) {
      lastPosition = new SwerveModulePosition(inputs.drivePositionMeters, inputs.turnPosition);
    }
  }

  /** Runs the module at the requested state. The state is optimized and cosine-scaled. */
  public void runSetpoint(SwerveModuleState state) {
    if (!isReady()) {
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

  /** True when the module can safely accept drive and turn setpoints. */
  public boolean isReady() {
    return !failed && inputsReady();
  }

  private boolean inputsReady() {
    return inputs.driveConnected && inputs.turnConnected && inputs.turnEncoderSeeded;
  }

  private boolean coastAttemptDue() {
    double now = Timer.getFPGATimestamp();
    if (Double.isNaN(lastCoastAttempt) || now - lastCoastAttempt >= 1.0) {
      lastCoastAttempt = now;
      return true;
    }
    return false;
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
    return isReady() ? inputs.turnPosition : lastPosition.angle;
  }

  public double getPositionMeters() {
    return isReady() ? inputs.drivePositionMeters : lastPosition.distanceMeters;
  }

  public SwerveModulePosition getPosition() {
    return isReady()
        ? new SwerveModulePosition(inputs.drivePositionMeters, inputs.turnPosition)
        : lastPosition;
  }

  public SwerveModuleState getState() {
    return isReady()
        ? new SwerveModuleState(inputs.driveVelocityMetersPerSec, inputs.turnPosition)
        : new SwerveModuleState(0.0, lastPosition.angle);
  }
}
