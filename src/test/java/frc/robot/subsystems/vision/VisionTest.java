package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class VisionTest {
  @BeforeAll
  static void initHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void disabledVisionIsNoOp() {
    Drive drive =
        new Drive(
            new GyroIO() {},
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());
    Vision vision = new Vision(drive);
    vision.periodic();
    assertEquals(new Pose2d(), drive.getPose());
  }
}
