// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file at the root of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

/** Flips poses between the blue and red sides of the field. */
public final class AllianceFlipUtil {
  private AllianceFlipUtil() {}

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip() ? flip(pose) : pose;
  }

  /** Mirrors a blue-side pose across the field center. */
  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(
        FieldConstants.FIELD_LENGTH_METERS - pose.getX(),
        FieldConstants.FIELD_WIDTH_METERS - pose.getY(),
        pose.getRotation().plus(Rotation2d.kPi));
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }
}
