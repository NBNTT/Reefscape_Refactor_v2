package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants;
import org.junit.jupiter.api.Test;

class DriveConstantsTest {
  @Test
  void moduleTranslationsMatchRobotDimensions() {
    double halfTrack = Units.inchesToMeters(20.5) / 2.0;
    assertEquals(4, DriveConstants.moduleTranslations.length);
    assertEquals(halfTrack, DriveConstants.moduleTranslations[0].getX(), 1e-9);
    assertEquals(halfTrack, DriveConstants.moduleTranslations[0].getY(), 1e-9);
    assertEquals(-halfTrack, DriveConstants.moduleTranslations[1].getY(), 1e-9);
    assertEquals(halfTrack, DriveConstants.moduleTranslations[2].getY(), 1e-9);
    assertEquals(-halfTrack, DriveConstants.moduleTranslations[3].getX(), 1e-9);
    assertEquals(-halfTrack, DriveConstants.moduleTranslations[3].getY(), 1e-9);
  }

  @Test
  void moduleIdsMatchCanConstants() {
    assertEquals(Constants.CAN.FRONT_LEFT_DRIVE, DriveConstants.modules[0].driveId());
    assertEquals(Constants.CAN.FRONT_RIGHT_DRIVE, DriveConstants.modules[1].driveId());
    assertEquals(Constants.CAN.BACK_LEFT_DRIVE, DriveConstants.modules[2].driveId());
    assertEquals(Constants.CAN.BACK_RIGHT_DRIVE, DriveConstants.modules[3].driveId());
    assertEquals(Constants.CAN.FRONT_LEFT_TURN, DriveConstants.modules[0].turnId());
    assertEquals(Constants.CAN.FRONT_RIGHT_TURN, DriveConstants.modules[1].turnId());
    assertEquals(Constants.CAN.BACK_LEFT_TURN, DriveConstants.modules[2].turnId());
    assertEquals(Constants.CAN.BACK_RIGHT_TURN, DriveConstants.modules[3].turnId());
  }

  @Test
  void conversionFactorsAreExact() {
    assertEquals(
        2.0 * Math.PI * DriveConstants.wheelRadiusMeters / DriveConstants.driveMotorReduction,
        DriveConstants.driveEncoderPositionFactor,
        1e-12);
    assertEquals(
        2.0 * Math.PI / DriveConstants.turnMotorReduction,
        DriveConstants.turnEncoderPositionFactor,
        1e-12);
    assertTrue(DriveConstants.driveKv > 0.0);
  }
}
