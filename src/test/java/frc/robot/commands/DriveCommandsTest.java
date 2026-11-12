package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

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
}
