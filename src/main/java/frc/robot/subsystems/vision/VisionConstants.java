package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

/** PhotonVision configuration. Add cameras when they are mounted. */
public final class VisionConstants {
  private VisionConstants() {}

  // Update to next season's field layout when the game changes.
  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  public record CameraConfig(String name, Transform3d robotToCamera) {}

  /** Measured camera transforms; empty until the next robot's cameras are mounted. */
  public static final CameraConfig[] CAMERAS = {};
}
