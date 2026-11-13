# reefscape-refactor-v2

Drivetrain-only refactor of team 8567's Reefscape robot. The superstructure was removed, so this
project keeps only what the 2024/2025 robot still has:

- 4x SDS MK4n swerve modules
- 8x REV NEO motors on Spark MAX controllers
- 4x analog absolute encoders on roboRIO analog inputs 0-3
- CTRE Pigeon 2
- REV Power Distribution Hub
- roboRIO and radio

AdvantageKit logging is kept light so the same drivetrain can move to the next robot. PhotonVision
is integrated with the drive pose estimator but disabled until cameras are mounted.

## Build and test

```powershell
$env:JAVA_HOME = "C:\Program Files\Eclipse Adoptium\jdk-17.0.21.9-hotspot"
.\gradlew.bat build
```

Unit tests run on the desktop and do not need a roboRIO:

```powershell
.\gradlew.bat test
```

Headless simulation:

```powershell
.\gradlew.bat simulateJava
```

The simulation uses `ModuleIOSim`, so you can drive it with a controller or run a code auto
(select "Drive Forward" in the dashboard chooser) without any hardware. Set
`wpi.sim.addGui().defaultEnabled = true` in `build.gradle` if you want the Sim GUI window.

Logs are written to the USB drive (`/U/logs`) on the robot and to `logs/` on the desktop. Open the
`.wpilog` directly in AdvantageScope. The swerve tab uses the standard keys
`SwerveStates/Measured`, `SwerveStates/Setpoints`, `SwerveChassisSpeeds/Measured`, and
`Odometry/Robot`; module inputs are under `Drive/Module0` through `Drive/Module3`.

## Controls

Only these inputs do anything. Every other button is unbound.

| Input | Action |
| --- | --- |
| Left stick | Field- or robot-oriented translation |
| Right stick X | Rotation |
| Right stick (cardinal mode) | Angle selects heading; deflection sets speed |
| Right bumper (hold) | Slow mode (30%) |
| D-pad up | Field-oriented drive (default) |
| D-pad right | Cardinal field-oriented drive |
| D-pad down | Robot-oriented drive |
| A (hold) | X-lock wheels |
| Back | Set the current direction as forward |

The sticks use a deadband and cubic response. A short asymmetric slew limiter smooths starts while
keeping stops quick and leaving full speed unchanged. Heading hold is always on: with the rotation
stick centered it holds the last heading, even at zero translation, and rotation input always wins.
Field-oriented drive is the default. Point the robot where the driver wants "forward", press Back,
and that direction stays forward for the match regardless of alliance.

Cardinal mode keeps field-oriented translation. The right-stick angle is the target heading: up is
forward, right is the driver's right, down is backward, always turning the shortest way. Deflection
scales rotation speed through the same cubic response, so a light push turns slowly. Sweeping the
stick around its edge spins the robot that way. Y, B, X, Start, and the triggers are unbound.

## Before the first drive: commissioning checklist

See [`docs/COMMISSIONING.md`](docs/COMMISSIONING.md) for the full step-by-step procedure.

Every value below is in `DriveConstants` or `Constants.CAN`.

- [ ] CAN IDs: FL 10/20, FR 11/21, BL 12/22, BR 13/23, Pigeon 31, PDH 1.
- [ ] Analog absolute encoder channels: FL 0, FR 1, BL 2, BR 3.
- [ ] Gear ratio: 5.9:1 drive, 18.75:1 steering (MK4n L2+ with 16T pinions). If the modules are a
      different MK4n level, change `driveMotorReduction`.
- [ ] Wheel diameter: 3.75 in. Measure the actual wheels.
- [ ] Geometry: wheelbase and track width are 20.5 in. Measure center-to-center and update
      `wheelBase`/`trackWidth` if they differ.
- [ ] Module zero offsets: currently `-6.903, -1.909, 3.035, 0.526` rad. Re-measure if a module was
      rebuilt. Point all four wheels forward and adjust each offset until
      `Drive/ModuleN/turnAbsolutePositionRad` is near 0.
- [ ] Motor inversion: all drive and turn motors are currently inverted. If a module drives the
      wrong way, flip that module's `driveMotorInverted` or `turnMotorInverted`.
- [ ] Turn encoder: turn each wheel by hand. `Drive/ModuleN/turnPosition` and
      `turnAbsolutePositionRad` must both increase together and agree within about 0.5 rad. If they
      diverge, check `turnMotorInverted`/`absoluteEncoderReversed`; the module re-seeds automatically
      when the absolute encoder is valid.
- [ ] Alerts: `turnEncoderSeeded` and `turnAbsoluteEncoderValid` must both be true on all four
      modules before driving. A module that loses a motor or encoder coasts both motors, freezes its
      last valid odometry position, and re-joins automatically when the connection returns.
- [ ] Pigeon 2 orientation: the robot must be flat and still at startup. The Pigeon x-axis must
      point forward; verify the heading increases when the robot turns counter-clockwise.
- [ ] Drive gains: `driveKp`, `driveKs`, and `driveKv` are starting values. Tune them, or run a
      velocity sweep, before trusting high-speed tracking.
- [ ] Turn gains: `turnKp` starts at 0.25. Increase it if the modules are sluggish; lower it if they
      oscillate.

## What was removed

Elevator, coral intake, algae intake, climber, lights, reef alignment commands, and the old
`Odometry`/`Swerve`/`SwerveModule` classes. The old `dev` branch remains the reference for the
superstructure; this project is the clean drivetrain base for the next robot.

## Vision

`Vision` reads PhotonVision camera results and feeds `Drive.addVisionMeasurement`, so camera poses
go through the same `SwerveDrivePoseEstimator` as odometry. It is a no-op while `CAMERAS` is empty or the code is not running on the real robot. When cameras are mounted:

1. Update `FIELD_LAYOUT` and `Constants.FieldConstants` to next season's field.
2. Fill `CAMERAS` with measured camera names and `robotToCamera` transforms.

Vision measurements are rejected if they are non-finite, outside the field, more than 0.1 s in the
future, or more than 2 s old. Single-tag measurements get high rotation uncertainty; multi-tag
measurements get tighter standard deviations.

## Failure handling

Each module watches its drive connection, turn connection, and turn-encoder seed. If any of those
is bad for three loops, the module:

- sets both Spark MAXes to coast so the wheel can be dragged,
- freezes its last valid position and reports zero speed, so its encoders are ignored by odometry,
- keeps the other three modules driving and steering.

Telemetry-only read failures do not count as module failures. A failed module's setpoint is zeroed
before wheel-speed desaturation, so it does not limit the healthy modules.

When all three signals return, it re-brakes, re-seeds the turn encoder, and resumes normal odometry.
`Drive/ModulesFailed` and the per-module alerts show the current state.

## Hardware abstraction

`ModuleIO` is the hardware seam. `ModuleIOSparkMax` implements the current REV MK4n hardware and
`ModuleIOSim` implements the physics model. A CTRE MK5n drivetrain only needs a new `ModuleIO`
implementation (TalonFX + CANcoder); `Module`, `Drive`, kinematics, logging, and controls stay
unchanged. `GyroIO` does the same for the Pigeon.

Runtime dependencies are WPILib, AdvantageKit, Phoenix 6 (Pigeon), REVLib (Spark MAX), and
PhotonLib (vision). JUnit is test-only. There is no PathPlanner or other unused vendor library.

## Development environment on this machine

- JDK 17: `C:\Program Files\Eclipse Adoptium\jdk-17.0.21.9-hotspot`
- Gradle 8.11 via `gradlew.bat` (cached locally)
- AdvantageScope, Elastic, Glass, SysId, DataLogTool, OutlineViewer: shortcuts in
  `Desktop\2025 WPILib Tools`
- REV Hardware Client and Phoenix Tuner X are installed for device configuration
- The full WPILib ISO was not installed. Gradle downloads the WPILib Java libraries, and the
  individual desktop tools above were installed from the WPILib Maven repository.
- FRC Game Tools / Driver Station are not installed. Install them from NI when you need to deploy to
  the roboRIO or use the real Driver Station.
- PathPlanner is not included: its 2025 Maven artifacts are no longer published. The drivetrain is
  ready for code-based autos; add the 2026 PathPlanner library if you upgrade the project to 2026.
