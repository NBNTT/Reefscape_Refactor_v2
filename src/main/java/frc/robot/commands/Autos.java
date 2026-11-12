// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

/** Simple autonomous routines that only need the drivetrain. */
public final class Autos {
  private Autos() {}

  /** Drives robot-relative forward for the given time. */
  public static Command driveForward(Drive drive, double seconds, double metersPerSecond) {
    return Commands.run(
            () -> drive.runVelocity(new ChassisSpeeds(metersPerSecond, 0.0, 0.0)), drive)
        .withTimeout(seconds)
        .andThen(Commands.runOnce(drive::stop, drive))
        .withName("Drive Forward");
  }

  /** Leaves the starting line. */
  public static Command leave(Drive drive) {
    return driveForward(drive, 2.0, 2.0).withName("Leave");
  }
}
