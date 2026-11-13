package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;
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

  private static final class FixedGyro implements GyroIO {
    private final Rotation2d yaw = Rotation2d.fromDegrees(90.0);

    @Override
    public void updateInputs(GyroIOInputs inputs) {
      inputs.connected = true;
      inputs.yawPosition = yaw;
    }
  }

  private static final class FailableModuleIO implements ModuleIO {
    private final ModuleIOSim delegate = new ModuleIOSim();
    boolean healthy = true;
    boolean coast = false;
    int resets = 0;

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
      delegate.updateInputs(inputs);
      if (!healthy) {
        inputs.driveConnected = false;
        inputs.turnConnected = false;
        inputs.turnEncoderSeeded = false;
      }
    }

    @Override
    public void setDriveVelocity(double velocityMetersPerSec) {
      delegate.setDriveVelocity(velocityMetersPerSec);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
      delegate.setTurnPosition(rotation);
    }

    @Override
    public void setDriveOpenLoop(double volts) {
      delegate.setDriveOpenLoop(volts);
    }

    @Override
    public void setTurnOpenLoop(double volts) {
      delegate.setTurnOpenLoop(volts);
    }

    @Override
    public void resetTurnEncoder() {
      resets++;
    }

    @Override
    public boolean setCoast(boolean coast) {
      this.coast = coast;
      return true;
    }
  }

  @Test
  void drivesForwardInSimulation() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    Drive drive = newSimDrive();
    drive.periodic();
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
  void failedModuleCoastsAndStillDrives() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    FailableModuleIO fl = new FailableModuleIO();
    Drive drive =
        new Drive(
            new GyroIO() {},
            fl,
            new FailableModuleIO(),
            new FailableModuleIO(),
            new FailableModuleIO());

    for (int i = 0; i < 5; i++) {
      drive.periodic();
    }
    fl.healthy = false;
    for (int i = 0; i < 3; i++) {
      drive.periodic();
    }
    assertTrue(drive.isModuleFailed(0));
    assertTrue(fl.coast, "failed module should coast");

    double startX = drive.getPose().getX();
    drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0));
    for (int i = 0; i < 50; i++) {
      drive.periodic();
    }
    assertTrue(drive.getPose().getX() > startX + 0.1, "three healthy modules should still drive");

    fl.healthy = true;
    drive.periodic();
    assertFalse(drive.isModuleFailed(0));
    assertFalse(fl.coast, "recovered module should brake again");

    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @Test
  void rejectsInvalidVisionMeasurements() {
    Drive drive = newSimDrive();
    drive.setPose(new Pose2d(1.0, 2.0, Rotation2d.kZero));

    double now = Timer.getFPGATimestamp();
    drive.addVisionMeasurement(
        new Pose2d(Double.NaN, 2.0, Rotation2d.kZero), now, VecBuilder.fill(0.1, 0.1, 0.1));
    drive.addVisionMeasurement(
        new Pose2d(100.0, 2.0, Rotation2d.kZero), now, VecBuilder.fill(0.1, 0.1, 0.1));

    assertEquals(1.0, drive.getPose().getX(), 1e-9);
  }

  @Test
  void joystickCommandDrivesForward() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    Drive drive = newSimDrive();
    Command command =
        DriveCommands.joystickDrive(
            drive,
            () -> 1.0,
            () -> 0.0,
            () -> 0.0,
            () -> 0.0,
            () -> 0.0,
            () -> false,
            () -> DriveCommands.DriveMode.FIELD);
    command.initialize();
    for (int i = 0; i < 50; i++) {
      command.execute();
      drive.periodic();
    }

    assertTrue(drive.getPose().getX() > 0.1, "joystick command should move forward");
    command.end(false);

    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @Test
  void cardinalCommandTurnsToStickAngle() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    Drive drive = newSimDrive();
    Command command =
        DriveCommands.joystickDrive(
            drive,
            () -> 0.0,
            () -> 0.0,
            () -> 0.0,
            () -> 0.0,
            () -> 1.0,
            () -> false,
            () -> DriveCommands.DriveMode.CARDINAL);
    command.initialize();
    for (int i = 0; i < 100; i++) {
      command.execute();
      drive.periodic();
    }

    assertEquals(
        -90.0,
        drive.getRotation().getDegrees(),
        10.0,
        "right stick should face the robot to the driver's right");
    command.end(false);

    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @Test
  void xLockTurnsModulesToCross() {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    Drive drive = newSimDrive();
    drive.periodic();
    drive.stopWithX();
    for (int i = 0; i < 100; i++) {
      drive.periodic();
    }

    double[] expectedDegrees = {45.0, -45.0, 135.0, -135.0};
    for (int i = 0; i < 4; i++) {
      assertEquals(
          Math.IEEEremainder(expectedDegrees[i], 180.0),
          Math.IEEEremainder(drive.getModuleStates()[i].angle.getDegrees(), 180.0),
          8.0,
          "module " + i + " should be at its X angle");
    }

    double startX = drive.getPose().getX();
    drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0));
    for (int i = 0; i < 50; i++) {
      drive.periodic();
    }
    assertTrue(drive.getPose().getX() > startX + 0.1, "drive should recover after X-lock");

    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @Test
  void usesConnectedGyroHeading() {
    Drive drive =
        new Drive(
            new FixedGyro(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());
    drive.periodic();
    assertEquals(90.0, drive.getRotation().getDegrees(), 1e-6);
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
