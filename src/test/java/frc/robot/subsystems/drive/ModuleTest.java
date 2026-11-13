package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class ModuleTest {
  @BeforeAll
  static void initHal() {
    HAL.initialize(500, 0);
  }

  private static final class FakeModuleIO implements ModuleIO {
    boolean connected = true;
    boolean seeded = true;
    boolean coast = false;
    int resets = 0;
    double drivePositionMeters = 1.0;
    Rotation2d turnPosition = Rotation2d.fromDegrees(30.0);

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
      inputs.driveConnected = connected;
      inputs.turnConnected = connected;
      inputs.turnEncoderSeeded = seeded;
      inputs.turnAbsoluteEncoderValid = connected;
      inputs.drivePositionMeters = drivePositionMeters;
      inputs.turnPosition = turnPosition;
      inputs.turnAbsolutePositionRad = turnPosition.getRadians();
    }

    @Override
    public void setCoast(boolean coast) {
      this.coast = coast;
    }

    @Override
    public void resetTurnEncoder() {
      resets++;
    }
  }

  @Test
  void failedModuleCoastsFreezesAndRecovers() {
    FakeModuleIO io = new FakeModuleIO();
    Module module = new Module(io, 0);

    module.periodic();
    assertFalse(module.isFailed());
    assertEquals(1.0, module.getPosition().distanceMeters, 1e-9);

    io.connected = false;
    io.seeded = false;
    for (int i = 0; i < 6; i++) {
      module.periodic();
    }

    assertTrue(module.isFailed());
    assertTrue(io.coast, "failed module motors should coast");
    io.drivePositionMeters = 5.0;
    module.periodic();
    assertEquals(1.0, module.getPosition().distanceMeters, 1e-9, "failed encoder is ignored");

    io.connected = true;
    io.seeded = true;
    module.periodic();
    assertFalse(module.isFailed());
    assertFalse(io.coast, "recovered module should brake again");
    assertEquals(1, io.resets, "recovered module should re-seed its turn encoder");
    assertTrue(module.consumeOdometryReset(), "recovery should reset the odometry delta");
    assertFalse(module.consumeOdometryReset());
    assertEquals(5.0, module.getPosition().distanceMeters, 1e-9);
  }
}
