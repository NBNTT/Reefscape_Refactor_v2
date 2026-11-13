# Drivetrain Commissioning

Do this with the robot on blocks, wheels off the ground, and the battery connected. Keep one hand
near the disable button.

## 1. Check devices

- REV Hardware Client: all eight Spark MAXes appear with the expected IDs and firmware.
- Phoenix Tuner X: Pigeon 2 appears on CAN 31 and reports a stable yaw.
- REV Hardware Client: PDH appears on CAN 1.
- Glass or AdvantageScope: connect to NetworkTables and confirm `Drive/Module0` through
  `Drive/Module3` are publishing.

## 2. Check absolute encoders

For each module, turn the wheel by hand and watch
`Drive/ModuleN/turnAbsolutePositionRad` and `Drive/ModuleN/turnAbsoluteVoltage` in AdvantageScope.

- The value must change smoothly and stay within -pi to pi.
- `Drive/ModuleN/turnAbsoluteEncoderValid` must be true. If it is false, the analog encoder is
  unplugged or reading outside 0.01-4.99 V.
- Point the wheel forward. `turnAbsolutePositionRad` should be near 0 after the offset in
  `DriveConstants` is correct. If it is not, adjust that module's `absoluteEncoderOffsetRad` by the
  error and rebuild.
- `Drive/ModuleN/turnEncoderSeeded` must become true. If it does not, rotate the wheel slightly
  off the 0 V point; it re-seeds automatically, otherwise check the analog wiring.
- Compare `turnPosition` (Spark MAX relative encoder) with `turnAbsolutePositionRad` over a full
  revolution. They must move in the same direction and agree within about 0.5 rad. If they do not,
  fix the inversion/offset before driving.

## 3. Check motor direction

- Disable the robot. Spin one drive wheel forward by hand and confirm
  `Drive/ModuleN/driveVelocityMetersPerSec` is positive.
- Enable test mode or use REV Hardware Client to run one drive motor slowly forward. The wheel must
  move the robot forward, not backward.
- Run one turn motor slowly positive. The module must rotate counter-clockwise when viewed from
  above. If not, flip that module's `turnMotorInverted` (and re-check the turn encoder sign).

Repeat for all four modules. Do not run all four until each module is correct.

If any module alert appears while driving (disconnected motor, unseeded encoder, invalid absolute
encoder), stop and fix it before continuing. A module with a bad turn encoder will not steer; it coasts and freezes its odometry until it recovers.

## 3b. Check geometry and Pigeon mount

- Measure the wheelbase and track width center-to-center. Update `wheelBase` and `trackWidth` in
  `DriveConstants` if they are not 20.5 in.
- Place the robot on a flat floor. The Pigeon 2 must be flat, with its x-axis pointing forward.
- Rotate the robot counter-clockwise by hand. `Drive/Gyro/yawPosition` and `Odometry/Robot` heading
  must increase. If the heading decreases, the Pigeon is mounted upside down or rotated.

## 4. Check odometry

- Put the robot on the floor in a clear area.
- Teleop: push forward. `Odometry/Robot` x must increase; y must stay near zero.
- Rotate in place counter-clockwise. `Odometry/Robot` heading must increase.
- If x decreases when driving forward, the drive motors are inverted.
- If the heading decreases when rotating counter-clockwise, the Pigeon yaw is reversed or the
  Pigeon is mounted upside down.

## 5. Tune

- Drive gains: start with `driveKp = 0.5`, `driveKs = 0.1`, `driveKv` as computed. Increase `driveKp`
  if the robot lags; lower it if it oscillates.
- Turn gains: start with `turnKp = 0.25`. Increase if modules are sluggish; lower if they oscillate.
- Heading hold: strafe with the left stick and no rotation; the heading must hold. Move the right
  stick; the hold must yield to driver rotation and then hold the new heading.
- D-pad up: field-oriented drive, the default.
- D-pad down: robot-oriented drive. Forward must follow the robot.
- A: hold X-lock; all four wheels should turn to an X and resist pushing.
- Re-measure the wheel diameter and update `wheelRadiusMeters` if the odometry drifts over a long
  push.

## 6. Before a match

- Confirm the battery is secure and the PDH breaker is on.
- Confirm the radio and roboRIO are connected.
- Confirm the driver controller is on port 0.
- Press Back while the robot faces the driver's forward direction; that direction stays forward for
  the match.

