// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/** Wires the drivetrain, controller, autonomous chooser, and telemetry together. */
public class RobotContainer {
  private final Drive drive;
  private final Vision vision;
  private final PowerDistribution pdh;
  private final CommandXboxController driver =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL ->
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOSpark(0),
                  new ModuleIOSpark(1),
                  new ModuleIOSpark(2),
                  new ModuleIOSpark(3));
      case SIM ->
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
      case REPLAY ->
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {});
      default ->
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {});
    }

    vision = new Vision(drive);
    pdh =
        Constants.currentMode == Constants.Mode.REAL
            ? new PowerDistribution(Constants.CAN.PDH, ModuleType.kRev)
            : null;

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () ->
                DriverStation.isJoystickConnected(OperatorConstants.DRIVER_CONTROLLER_PORT)
                    && driver.rightBumper().getAsBoolean()));

    configureBindings();
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", Commands.none());
    autoChooser.addOption("Drive Forward", Autos.driveForward(drive, 2.0, 1.5));
    autoChooser.addOption("Leave", Autos.leave(drive));
  }

  private void configureBindings() {
    driver.a().whileTrue(DriveCommands.lockWheels(drive));
    driver.back().onTrue(Commands.runOnce(drive::zeroHeading).withName("Zero Heading"));
    driver
        .start()
        .onTrue(Commands.runOnce(drive::resetTurnEncoders).withName("Reset Turn Encoders"));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void periodic() {
    if (pdh != null) {
      Logger.recordOutput("Power/TotalCurrentAmps", pdh.getTotalCurrent());
      Logger.recordOutput("Power/Voltage", pdh.getVoltage());
      Logger.recordOutput("Power/TotalPowerWatts", pdh.getTotalPower());
    }
  }
}
