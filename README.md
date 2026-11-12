# reefscape-refactor-v2

Drivetrain-only refactor of team 8567's Reefscape robot. The superstructure was removed, so this
project keeps only what the 2024/2025 robot still has:

- 4x SDS MK4n swerve modules
- 8x REV NEO motors on Spark MAX controllers
- 4x analog absolute encoders on roboRIO analog inputs 0-3
- CTRE Pigeon 2
- REV Power Distribution Hub
- roboRIO and radio

Logging (AdvantageKit) and optional PhotonVision pose estimation are kept so the same drivetrain
can be moved onto the next robot.

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
without any hardware. Set `wpi.sim.addGui().defaultEnabled = true` in `build.gradle` if you want the
Sim GUI window.

Logs are written under `logs/`. Replay them with AdvantageScope, or set
`Constants.simMode = Mode.REPLAY` and run `simulateJava`.

## Controls

| Input | Action |
| --- | --- |
| Left stick | Field-relative translation |
| Right stick X | Rotation |
| Right bumper (hold) | Slow mode (30%) |
| A (hold) | X-lock wheels |
| Back | Zero heading (keep position) |
| Start | Re-seed turn encoders from the analog absolute encoders |

## Before the first drive: commissioning checklist

Every value below is in `DriveConstants` or `Constants.CAN`.

- [ ] CAN IDs: FL 10/20, FR 11/21, BL 12/22, BR 13/23, Pigeon 31, PDH 1.
- [ ] Analog absolute encoder channels: FL 0, FR 1, BL 2, BR 3.
- [ ] Gear ratio: 5.9:1 drive, 18.75:1 steering (MK4n L2+ with 16T pinions). If the modules are a
      different MK4n level, change `driveMotorReduction`.
- [ ] Wheel diameter: 3.75 in. Measure the actual wheels.
- [ ] Module zero offsets: currently `-6.903, -1.909, 3.035, 0.526` rad. Re-measure if a module was
      rebuilt. Point all four wheels forward and adjust each offset until the reported angle is 0.
- [ ] Motor inversion: all drive and turn motors are currently inverted. If a module drives the
      wrong way, flip that module's `driveMotorInverted` or `turnMotorInverted`.
- [ ] Turn encoder direction: if a module's reported angle decreases when it should increase, flip
      that module's `absoluteEncoderReversed`.
- [ ] Pigeon 2 orientation: the robot must be flat and still at startup. The yaw is zeroed in the
      constructor.
- [ ] Drive gains: `driveKp`, `driveKs`, and `driveKv` are starting values. Tune them, or run a
      velocity sweep, before trusting high-speed tracking.
- [ ] Turn gains: `turnKp` starts at 0.25. Increase it if the modules are sluggish; lower it if they
      oscillate.

## What was removed

Elevator, coral intake, algae intake, climber, lights, reef alignment commands, and the old
`Odometry`/`Swerve`/`SwerveModule` classes. The old `dev` branch remains the reference for the
superstructure; this project is the clean drivetrain base for the next robot.

## Vision

Vision is disabled by default because the stripped robot has no cameras. Set
`VisionConstants.enabled = true` and update the camera names and `robotToCamera` transforms when
cameras are mounted. The existing transforms are from the old superstructure and must be
re-measured.
