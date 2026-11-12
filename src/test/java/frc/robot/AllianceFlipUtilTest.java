package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.AllianceFlipUtil;
import org.junit.jupiter.api.Test;

class AllianceFlipUtilTest {
  @Test
  void flipMirrorsBothAxesAndRotates180() {
    Pose2d blue = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));
    Pose2d red = AllianceFlipUtil.flip(blue);

    assertEquals(Constants.FieldConstants.FIELD_LENGTH_METERS - 1.0, red.getX(), 1e-9);
    assertEquals(Constants.FieldConstants.FIELD_WIDTH_METERS - 2.0, red.getY(), 1e-9);
    assertEquals(
        MathUtil.angleModulus(Rotation2d.fromDegrees(210.0).getRadians()),
        MathUtil.angleModulus(red.getRotation().getRadians()),
        1e-9);
  }
}
