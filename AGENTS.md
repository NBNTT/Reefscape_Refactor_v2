# AGENTS.md

## Project

FRC Java 2025 drivetrain-only robot. Four SDS MK4n modules with REV NEO motors on Spark MAX, analog
absolute encoders on roboRIO analog 0-3, Pigeon 2 on CAN 31, REV PDH on CAN 1. AdvantageKit logging
is kept light. PhotonVision is integrated but disabled until cameras are mounted.

## Commands

```powershell
$env:JAVA_HOME = "C:\Program Files\Eclipse Adoptium\jdk-17.0.21.9-hotspot"
.\gradlew.bat build
.\gradlew.bat test
.\gradlew.bat simulateJava
```

The CI workflow runs `./gradlew build` on Ubuntu.

## Architecture

- `ModuleIO` is the hardware seam. `ModuleIOSparkMax` is REV MK4n; a CTRE MK5n would add a
  `ModuleIOTalonFX` and leave `Drive`/`Module`/controls unchanged.
- `DriverController` is the input seam. `XboxDriverController` targets the 8BitDo Ultimate in
  XInput mode.
- `Module` detects failures, coasts both motors, freezes its odometry contribution, and recovers
  automatically.
- `Vision` feeds PhotonVision poses into `Drive.addVisionMeasurement`; it is disabled until
  `VisionConstants.CAMERAS` is configured.
- AdvantageScope keys are standard: `SwerveStates/*`, `SwerveChassisSpeeds/*`, `Odometry/Robot`.

## Constraints

- Keep the project buildable with WPILib 2025.3.2. Do not add PathPlanner 2025: those Maven
  artifacts are no longer published.
- Drivetrain hardware values live in `DriveConstants`; CAN IDs live in `Constants.CAN`.
- Only left/right sticks, right bumper, D-pad up/down/right, A, and Back are bound. Do not add
  other button bindings.
- No superstructure or unused vendor libraries.
- PhotonVision stays disabled until cameras and measured transforms are available.
- Tests must not require a roboRIO. Use `ModuleIOSim` and `GyroIO` no-op implementations.
