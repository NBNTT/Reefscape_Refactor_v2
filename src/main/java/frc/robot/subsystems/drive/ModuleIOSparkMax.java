// Copyright (c) 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD license that can be found in
// the LICENSE file at the root of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import java.util.function.DoubleSupplier;

/**
 * Real module IO: two Spark MAX controllers and an analog absolute encoder wired to the roboRIO.
 *
 * <p>The Spark MAX turn motor uses its own relative encoder for closed-loop control. It is seeded
 * from the analog absolute encoder once at startup (and again on request), so belt slip or a
 * swapped module can be corrected without rewiring.
 */
public class ModuleIOSparkMax implements ModuleIO {
  private final ModuleHardware config;

  private final SparkMax driveSpark;
  private final SparkMax turnSpark;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;
  private final AnalogInput absoluteEncoder;

  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  private boolean driveConfigured = false;
  private boolean turnConfigured = false;
  private boolean turnSeeded = false;
  private int turnDriftCounter = 0;
  private int turnReseedCount = 0;

  public ModuleIOSparkMax(int module) {
    config = modules[module];

    driveSpark = new SparkMax(config.driveId(), MotorType.kBrushless);
    turnSpark = new SparkMax(config.turnId(), MotorType.kBrushless);
    driveEncoder = driveSpark.getEncoder();
    turnEncoder = turnSpark.getEncoder();
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();
    absoluteEncoder = new AnalogInput(config.absoluteEncoderChannel());
    absoluteEncoder.setAverageBits(4);

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig
        .inverted(config.driveMotorInverted())
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(driveMotorCurrentLimit)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(driveEncoderPositionFactor)
        .velocityConversionFactor(driveEncoderVelocityFactor);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(driveKp, 0.0, driveKd);
    driveConfigured =
        tryUntilOk(
            5,
            () ->
                driveSpark.configure(
                    driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    driveConfigured &= tryUntilOk(5, () -> driveEncoder.setPosition(0.0));

    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(config.turnMotorInverted())
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .encoder
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-Math.PI, Math.PI)
        .pid(turnKp, 0.0, turnKd);
    turnConfigured =
        tryUntilOk(
            5,
            () ->
                turnSpark.configure(
                    turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    resetTurnEncoder();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(driveSpark, driveEncoder::getPosition, value -> inputs.drivePositionMeters = value);
    ifOk(driveSpark, driveEncoder::getVelocity, value -> inputs.driveVelocityMetersPerSec = value);
    boolean driveControlOk = !sparkStickyFault;

    // Telemetry-only reads: a failure here must not fault the module.
    ifOk(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        values -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(driveSpark, driveSpark::getOutputCurrent, value -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConfigured && driveConnectedDebounce.calculate(driveControlOk);

    sparkStickyFault = false;
    ifOk(turnSpark, turnEncoder::getPosition, value -> inputs.turnPosition = new Rotation2d(value));
    boolean turnControlOk = !sparkStickyFault;

    // Turn velocity, applied output, and current are telemetry only.
    ifOk(turnSpark, turnEncoder::getVelocity, value -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        values -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, value -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConfigured && turnConnectedDebounce.calculate(turnControlOk);

    inputs.turnAbsoluteVoltage = absoluteEncoder.getAverageVoltage();
    inputs.turnAbsolutePositionRad = getAbsoluteAngle().getRadians();
    inputs.turnAbsoluteEncoderValid = absoluteEncoderValid();

    if (!turnSeeded && inputs.turnAbsoluteEncoderValid) {
      turnSeeded = resetTurnEncoderInternal();
    }
    if (turnSeeded && inputs.turnAbsoluteEncoderValid) {
      double error =
          MathUtil.angleModulus(inputs.turnPosition.getRadians() - inputs.turnAbsolutePositionRad);
      if (Math.abs(error) > 0.5) {
        if (++turnDriftCounter > 10) {
          turnSeeded = resetTurnEncoderInternal();
          turnDriftCounter = 0;
          turnReseedCount++;
        }
      } else {
        turnDriftCounter = 0;
      }
    }
    inputs.turnEncoderSeeded = turnSeeded;
    inputs.turnReseedCount = turnReseedCount;
  }

  @Override
  public void setDriveVelocity(double velocityMetersPerSec) {
    if (!driveConfigured || !Double.isFinite(velocityMetersPerSec)) {
      return;
    }
    double ffVolts = driveKs * Math.signum(velocityMetersPerSec) + driveKv * velocityMetersPerSec;
    driveController.setReference(
        velocityMetersPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    if (!turnConfigured
        || !turnSeeded
        || rotation == null
        || !Double.isFinite(rotation.getRadians())) {
      return;
    }
    double setpoint = MathUtil.inputModulus(rotation.getRadians(), -Math.PI, Math.PI);
    turnController.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public void setDriveOpenLoop(double volts) {
    driveSpark.setVoltage(volts);
  }

  @Override
  public void setTurnOpenLoop(double volts) {
    turnSpark.setVoltage(volts);
  }

  @Override
  public void setCoast(boolean coast) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(coast ? IdleMode.kCoast : IdleMode.kBrake);
    tryUntilOk(
        5,
        () ->
            driveSpark.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    tryUntilOk(
        5,
        () ->
            turnSpark.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  @Override
  public void resetTurnEncoder() {
    resetTurnEncoderInternal();
  }

  private boolean resetTurnEncoderInternal() {
    if (!turnConfigured || !absoluteEncoderValid()) {
      return false;
    }
    boolean ok = tryUntilOk(5, () -> turnEncoder.setPosition(getAbsoluteAngle().getRadians()));
    if (ok) {
      turnSeeded = true;
    }
    return ok;
  }

  private boolean absoluteEncoderValid() {
    double voltage = absoluteEncoder.getAverageVoltage();
    return voltage > 0.01 && voltage < 4.99;
  }

  private Rotation2d getAbsoluteAngle() {
    double voltage5v = RobotController.getVoltage5V();
    if (voltage5v < 1.0) {
      voltage5v = 5.0;
    }
    double angle = absoluteEncoder.getAverageVoltage() / voltage5v * 2.0 * Math.PI;
    angle += config.absoluteEncoderOffsetRad();
    angle = MathUtil.inputModulus(angle, -Math.PI, Math.PI);
    if (config.absoluteEncoderReversed()) {
      angle = -angle;
    }
    return new Rotation2d(MathUtil.angleModulus(angle));
  }
}
