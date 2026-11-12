// Copyright (c) 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD license that can be found in
// the LICENSE file at the root of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;

/** Pigeon 2 gyro implementation. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(DriveConstants.pigeonCanId);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    resetYaw();
    yaw.setUpdateFrequency(50.0);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
  }

  @Override
  public void resetYaw() {
    applyYaw(0.0);
  }

  @Override
  public void setYaw(Rotation2d yaw) {
    applyYaw(yaw.getDegrees());
  }

  private void applyYaw(double degrees) {
    StatusCode status = pigeon.setYaw(degrees);
    for (int i = 0; i < 5 && status != StatusCode.OK; i++) {
      status = pigeon.setYaw(degrees);
    }
    if (status != StatusCode.OK) {
      DriverStation.reportWarning("Pigeon 2 setYaw failed; heading may be stale.", false);
    }
  }
}
