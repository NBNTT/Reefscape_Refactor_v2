package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class DriveTest {
  @BeforeAll
  static void initHal() {
    HAL.initialize(500, 0);
  }

  private static Drive newSimDrive() {
    return new Drive(
        new GyroIO() {},
        new ModuleIOSim(),
        new ModuleIOSim(),
        new ModuleIOSim(),
        new ModuleIOSim());
  }

  @Test
  void drivesForwardInSimulation() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    Drive drive = newSimDrive();
    drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0));
    for (int i = 0; i < 50; i++) {
      drive.periodic();
    }

    assertTrue(drive.getPose().getX() > 0.1, "robot should move forward");
    assertEquals(0.0, drive.getPose().getY(), 0.05);
    assertEquals(0.0, drive.getRotation().getRadians(), 0.05);

    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @Test
  void poseAndHeadingCanBeReset() {
    Drive drive = newSimDrive();
    drive.setPose(new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90.0)));

    assertEquals(1.0, drive.getPose().getX(), 1e-6);
    assertEquals(2.0, drive.getPose().getY(), 1e-6);
    assertEquals(90.0, drive.getRotation().getDegrees(), 1e-6);

    drive.zeroHeading();
    assertEquals(0.0, drive.getRotation().getRadians(), 1e-6);
    assertEquals(1.0, drive.getPose().getX(), 1e-6);
  }
}
