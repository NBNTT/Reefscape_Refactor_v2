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
    assertEquals(-halfTrack, DriveConstants.moduleTranslations[3].getX(), 1e-9);
    assertEquals(-halfTrack, DriveConstants.moduleTranslations[3].getY(), 1e-9);
  }

  @Test
  void conversionFactorsArePositive() {
    assertTrue(DriveConstants.driveEncoderPositionFactor > 0.0);
    assertTrue(DriveConstants.turnEncoderPositionFactor > 0.0);
    assertTrue(DriveConstants.driveKv > 0.0);
  }
}
