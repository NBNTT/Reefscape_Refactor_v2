package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class DriveCommandsTest {
  @Test
  void deadbandZerosSmallInputs() {
    assertEquals(0.0, DriveCommands.applyDeadband(0.05, 0.1), 1e-9);
    assertEquals(0.0, DriveCommands.applyDeadband(-0.05, 0.1), 1e-9);
    assertEquals(0.0, DriveCommands.applyDeadband(Double.NaN, 0.1), 1e-9);
  }

  @Test
  void deadbandRescalesRemainingRange() {
    assertEquals(0.5, DriveCommands.applyDeadband(0.55, 0.1), 1e-9);
    assertEquals(-0.5, DriveCommands.applyDeadband(-0.55, 0.1), 1e-9);
    assertEquals(1.0, DriveCommands.applyDeadband(1.0, 0.1), 1e-9);
  }

  @Test
  void nearestCardinalSnapsToNinetyDegrees() {
    assertCardinal(10.0, 0.0);
    assertCardinal(50.0, 90.0);
    assertCardinal(170.0, 180.0);
    assertCardinal(250.0, 270.0);
  }

  @Test
  void cardinalLockCorrectsDriftButYieldsToRotation() {
    DriveCommands.CardinalLock lock = new DriveCommands.CardinalLock();
    double correction = lock.apply(true, 0.0, Rotation2d.fromDegrees(10.0), 9.42);
    assertTrue(correction < 0.0, "lock should rotate clockwise toward zero");
    assertEquals(1.0, lock.apply(true, 1.0, Rotation2d.fromDegrees(45.0), 9.42), 1e-9);
    assertEquals(1.0, lock.apply(false, 1.0, Rotation2d.fromDegrees(45.0), 9.42), 1e-9);
  }

  private static void assertCardinal(double currentDegrees, double expectedDegrees) {
    assertEquals(
        MathUtil.angleModulus(Rotation2d.fromDegrees(expectedDegrees).getRadians()),
        MathUtil.angleModulus(
            DriveCommands.nearestCardinal(Rotation2d.fromDegrees(currentDegrees)).getRadians()),
        1e-9);
  }
}
