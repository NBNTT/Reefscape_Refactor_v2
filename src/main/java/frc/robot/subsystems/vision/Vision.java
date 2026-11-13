package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * PhotonVision pose estimation feeding {@link Drive#addVisionMeasurement}. It is a no-op until
 * cameras are configured in {@link VisionConstants}.
 */
public class Vision extends SubsystemBase {
  private record Camera(PhotonCamera camera, PhotonPoseEstimator estimator) {}

  private final Drive drive;
  private final List<Camera> cameras = new ArrayList<>();

  public Vision(Drive drive) {
    this.drive = drive;
    if (VisionConstants.CAMERAS.length == 0 || Constants.currentMode != Constants.Mode.REAL) {
      return;
    }

    for (VisionConstants.CameraConfig config : VisionConstants.CAMERAS) {
      PhotonCamera camera = new PhotonCamera(config.name());
      PhotonPoseEstimator estimator =
          new PhotonPoseEstimator(
              VisionConstants.FIELD_LAYOUT,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              config.robotToCamera());
      estimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
      cameras.add(new Camera(camera, estimator));
    }
  }

  @Override
  public void periodic() {
    for (Camera camera : cameras) {
      camera.estimator().setReferencePose(drive.getPose());
      for (PhotonPipelineResult result : camera.camera().getAllUnreadResults()) {
        Optional<EstimatedRobotPose> estimated = camera.estimator().update(result);
        Logger.recordOutput(
            "Vision/" + camera.camera().getName() + "/Accepted", estimated.isPresent());
        if (estimated.isEmpty()) {
          continue;
        }
        drive.addVisionMeasurement(
            estimated.get().estimatedPose.toPose2d(),
            estimated.get().timestampSeconds,
            stdDevs(estimated.get().targetsUsed.size()));
        Logger.recordOutput(
            "Vision/" + camera.camera().getName() + "/Pose",
            estimated.get().estimatedPose.toPose2d());
      }
    }
  }

  private static Matrix<N3, N1> stdDevs(int tagCount) {
    double xy = tagCount >= 2 ? 0.15 : 0.5;
    double theta = tagCount >= 2 ? 0.3 : 9.9e4;
    return VecBuilder.fill(xy, xy, theta);
  }
}
