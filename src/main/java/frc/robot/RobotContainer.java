// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.XboxDriverController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/** Wires the drivetrain, controller, autonomous chooser, and telemetry together. */
public class RobotContainer {
  private final Drive drive;
  private final Vision vision;
  private final DriverController driver =
      new XboxDriverController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final PowerDistribution pdh;
  private final SendableChooser<Command> autoChooser;
  private boolean robotOriented = false;

  public RobotContainer() {
    drive =
        switch (Constants.currentMode) {
          case REAL ->
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOSparkMax(0),
                  new ModuleIOSparkMax(1),
                  new ModuleIOSparkMax(2),
                  new ModuleIOSparkMax(3));
          case SIM ->
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
          case REPLAY ->
              new Drive(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {});
        };

    vision = new Vision(drive);
    pdh =
        Constants.currentMode == Constants.Mode.REAL
            ? new PowerDistribution(Constants.CAN.PDH, ModuleType.kRev)
            : null;

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            driver::forward,
            driver::strafe,
            driver::rotation,
            driver::slowMode,
            () -> robotOriented));

    configureBindings();
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", Commands.none());
    autoChooser.addOption("Drive Forward", Autos.driveForward(drive, 2.0, 1.5));
    autoChooser.addOption("Leave", Autos.leave(drive));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Only D-pad, A, and Back are bound; the sticks and right bumper are read by the drive command.
   */
  private void configureBindings() {
    Trigger teleop = new Trigger(DriverStation::isTeleopEnabled);
    teleop.and(new Trigger(driver::xLock)).whileTrue(DriveCommands.lockWheels(drive));
    teleop
        .and(new Trigger(driver::fieldOriented))
        .onTrue(Commands.runOnce(() -> robotOriented = false));
    teleop
        .and(new Trigger(driver::robotOriented))
        .onTrue(Commands.runOnce(() -> robotOriented = true));
    teleop
        .and(new Trigger(driver::zeroHeading))
        .onTrue(Commands.runOnce(drive::zeroHeading).withName("Set Forward"));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void periodic() {
    Logger.recordOutput("Drive/RobotOriented", robotOriented);
    if (pdh != null) {
      Logger.recordOutput("Power/TotalCurrentAmps", pdh.getTotalCurrent());
      Logger.recordOutput("Power/Voltage", pdh.getVoltage());
      Logger.recordOutput("Power/TotalPowerWatts", pdh.getTotalPower());
    }
  }
}
