// Copyright (c) 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD license that can be found in
// the LICENSE file at the root of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics simulation for one swerve module. */
public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private final PIDController driveController = new PIDController(driveSimKp, 0.0, 0.0);
  private final PIDController turnController = new PIDController(turnSimKp, 0.0, 0.0);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim() {
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.025, driveMotorReduction),
            DCMotor.getNEO(1));
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, turnMotorReduction),
            DCMotor.getNEO(1));
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (driveClosedLoop) {
      double currentMetersPerSec = driveSim.getAngularVelocityRadPerSec() * wheelRadiusMeters;
      driveAppliedVolts = driveFFVolts + driveController.calculate(currentMetersPerSec);
    } else {
      driveController.reset();
    }

    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
    } else {
      turnController.reset();
    }

    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    inputs.driveConnected = true;
    inputs.drivePositionMeters = driveSim.getAngularPositionRad() * wheelRadiusMeters;
    inputs.driveVelocityMetersPerSec = driveSim.getAngularVelocityRadPerSec() * wheelRadiusMeters;
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.turnConnected = true;
    inputs.turnPosition = new Rotation2d(MathUtil.angleModulus(turnSim.getAngularPositionRad()));
    inputs.turnAbsolutePositionRad = inputs.turnPosition.getRadians();
    inputs.turnAbsoluteEncoderValid = true;
    inputs.turnEncoderSeeded = true;
    inputs.turnReseedCount = 0;
    inputs.turnAbsoluteVoltage =
        MathUtil.inputModulus(inputs.turnAbsolutePositionRad, 0.0, 2.0 * Math.PI)
            / (2.0 * Math.PI)
            * 5.0;
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
  }

  @Override
  public void setDriveVelocity(double velocityMetersPerSec) {
    driveClosedLoop = true;
    driveFFVolts = driveKs * Math.signum(velocityMetersPerSec) + driveSimKv * velocityMetersPerSec;
    driveController.setSetpoint(velocityMetersPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }

  @Override
  public void setDriveOpenLoop(double volts) {
    driveClosedLoop = false;
    driveAppliedVolts = volts;
  }

  @Override
  public void setTurnOpenLoop(double volts) {
    turnClosedLoop = false;
    turnAppliedVolts = volts;
  }

  @Override
  public void resetTurnEncoder() {
    // Simulation has no absolute encoder to re-seed from.
  }
}
