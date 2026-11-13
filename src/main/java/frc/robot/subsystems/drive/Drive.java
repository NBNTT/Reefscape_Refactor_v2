// Copyright (c) 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD license that can be found in
// the LICENSE file at the root of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/** Four-module swerve drivetrain with odometry, failure compensation, and AdvantageKit logging. */
public class Drive extends SubsystemBase {
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4];
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private boolean gyroWasConnected = false;
  private SwerveModulePosition[] lastModulePositions = {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (Module module : modules) {
      module.periodic();
    }

    if (DriverStation.isDisabled()) {
      for (Module module : modules) {
        module.stop();
      }
    }

    SwerveModulePosition[] modulePositions = getModulePositions();
    for (int i = 0; i < 4; i++) {
      if (modules[i].consumeOdometryReset()) {
        lastModulePositions[i] = modulePositions[i];
      }
    }
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      moduleDeltas[i] =
          new SwerveModulePosition(
              modulePositions[i].distanceMeters - lastModulePositions[i].distanceMeters,
              modulePositions[i].angle);
      lastModulePositions[i] = modulePositions[i];
    }

    if (gyroInputs.connected && !gyroWasConnected) {
      // The heading was integrated from module deltas while the gyro was gone. Re-anchor the
      // estimator to the gyro so reconnecting does not feed it a large jump.
      rawGyroRotation = gyroInputs.yawPosition;
      poseEstimator.resetPosition(
          rawGyroRotation,
          getModulePositions(),
          new Pose2d(getPose().getTranslation(), rawGyroRotation));
    }
    gyroWasConnected = gyroInputs.connected;

    if (gyroInputs.connected) {
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), rawGyroRotation, modulePositions);

    Logger.recordOutput("Odometry/Robot", getPose());
    Logger.recordOutput("SwerveChassisSpeeds/Measured", getChassisSpeeds());
    Logger.recordOutput("SwerveStates/Measured", getModuleStates());
    Logger.recordOutput("Drive/ModulesFailed", failedModuleCount());
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Constants.Mode.SIM);
  }

  /** Runs the drivetrain at the requested robot-relative velocity. Failed modules are skipped. */
  public void runVelocity(ChassisSpeeds speeds) {
    if (DriverStation.isDisabled()) {
      stop();
      return;
    }
    if (!Double.isFinite(speeds.vxMetersPerSecond)
        || !Double.isFinite(speeds.vyMetersPerSecond)
        || !Double.isFinite(speeds.omegaRadiansPerSecond)) {
      stop();
      return;
    }

    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    for (int i = 0; i < 4; i++) {
      if (modules[i].isFailed()) {
        modules[i].stop();
      } else {
        modules[i].runSetpoint(setpointStates[i]);
      }
    }

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
  }

  public void stop() {
    for (Module module : modules) {
      module.stop();
    }
  }

  public boolean isModuleFailed(int index) {
    return modules[index].isFailed();
  }

  private int failedModuleCount() {
    int count = 0;
    for (Module module : modules) {
      if (module.isFailed()) {
        count++;
      }
    }
    return count;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the pose and the gyro heading to the requested pose. */
  public void setPose(Pose2d pose) {
    gyroIO.setYaw(pose.getRotation());
    rawGyroRotation = pose.getRotation();
    lastModulePositions = getModulePositions();
    poseEstimator.resetPosition(rawGyroRotation, lastModulePositions, pose);
  }

  /** Resets the current heading to the alliance-forward direction, keeping the translation. */
  public void zeroHeading() {
    setPose(
        new Pose2d(
            getPose().getTranslation(),
            AllianceFlipUtil.shouldFlip() ? Rotation2d.kPi : Rotation2d.kZero));
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeedMetersPerSec;
  }

  public double getMaxAngularSpeedRadPerSec() {
    return maxAngularSpeedRadPerSec;
  }
}
