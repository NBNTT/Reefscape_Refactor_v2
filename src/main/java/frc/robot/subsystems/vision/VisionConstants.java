// Copyright (c) 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD license that can be found in
// the LICENSE file at the root of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

/** PhotonVision camera configuration. */
public final class VisionConstants {
  private VisionConstants() {}

  /**
   * The stripped drivetrain has no cameras. Set this to true and update the camera transforms below
   * when cameras are mounted on the next robot.
   */
  public static final boolean enabled = false;

  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  public record CameraConfig(String name, Transform3d robotToCamera) {}

  public static final CameraConfig[] cameras = {
    // Front camera, old superstructure position. Re-measure after remounting.
    new CameraConfig(
        "OV9281_Front",
        new Transform3d(
            Units.inchesToMeters(4.0),
            Units.inchesToMeters(9.956),
            Units.inchesToMeters(10.563),
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(90.0)))),
    // Back camera, old superstructure position. Re-measure after remounting.
    new CameraConfig(
        "OV9281_Back",
        new Transform3d(
            Units.inchesToMeters(-9.105),
            Units.inchesToMeters(9.294),
            Units.inchesToMeters(8.0),
            new Rotation3d(
                Units.degreesToRadians(10.0),
                Units.degreesToRadians(-2.154),
                Units.degreesToRadians(77.5))))
  };
}
