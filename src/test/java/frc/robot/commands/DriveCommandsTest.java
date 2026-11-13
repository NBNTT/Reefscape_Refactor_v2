package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
  void headingHoldCorrectsDriftButYieldsToRotation() {
    DriveCommands.CardinalLock lock = new DriveCommands.CardinalLock();
    assertEquals(0.0, lock.apply(0.0, Rotation2d.fromDegrees(10.0), 9.42, 0), 1e-9);
    assertTrue(
        lock.apply(0.0, Rotation2d.fromDegrees(20.0), 9.42, 0) < 0.0,
        "lock should rotate clockwise back to the held heading");
    assertEquals(1.0, lock.apply(1.0, Rotation2d.fromDegrees(20.0), 9.42, 0), 1e-9);
    assertEquals(0.0, lock.apply(0.0, Rotation2d.fromDegrees(20.0), 9.42, 1), 1e-9);
  }

  @Test
  void fastStopSlewAcceleratesSmoothlyAndStopsQuickly() {
    DriveCommands.FastStopSlew slew = new DriveCommands.FastStopSlew();
    slew.reset();
    assertEquals(0.04, slew.calculate(1.0, 0.0, 2.0, 10.0), 1e-9);
    assertEquals(0.24, slew.calculate(1.0, 0.1, 2.0, 10.0), 1e-9);
    assertEquals(0.0, slew.calculate(0.0, 0.2, 2.0, 10.0), 1e-9);
  }

  @Test
  void cardinalHeadingFollowsStickAngle() {
    assertEquals(0.0, DriveCommands.cardinalHeading(1.0, 0.0).getRadians(), 1e-9);
    assertEquals(-Math.PI / 2, DriveCommands.cardinalHeading(0.0, 1.0).getRadians(), 1e-9);
    assertEquals(Math.PI / 2, DriveCommands.cardinalHeading(0.0, -1.0).getRadians(), 1e-9);
    assertEquals(Math.PI, Math.abs(DriveCommands.cardinalHeading(-1.0, 0.0).getRadians()), 1e-9);
  }

  @Test
  void cardinalDriveTakesShortestPath() {
    DriveCommands.CardinalLock lock = new DriveCommands.CardinalLock();
    assertTrue(
        lock.driveTo(Rotation2d.fromDegrees(-90.0), Rotation2d.fromDegrees(0.0), 9.42, 0) < 0.0,
        "right stick should turn clockwise");
    assertTrue(
        lock.driveTo(Rotation2d.fromDegrees(-170.0), Rotation2d.fromDegrees(170.0), 9.42, 0) > 0.0,
        "shortest path from 170 to -170 is counter-clockwise");
  }
}
