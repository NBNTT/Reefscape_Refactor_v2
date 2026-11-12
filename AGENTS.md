# AGENTS.md

## Project

FRC Java 2025 drivetrain-only robot. Four SDS MK4n modules with REV NEO motors on Spark MAX, analog
absolute encoders on roboRIO analog 0-3, Pigeon 2 on CAN 31, REV PDH on CAN 1. AdvantageKit logging
and optional PhotonVision.

## Commands

```powershell
$env:JAVA_HOME = "C:\Program Files\Eclipse Adoptium\jdk-17.0.21.9-hotspot"
.\gradlew.bat build
.\gradlew.bat test
.\gradlew.bat simulateJava
```

The CI workflow runs `./gradlew build` on Ubuntu.

## Constraints

- Keep the project buildable with WPILib 2025.3.2. Do not add PathPlanner 2025: those Maven
  artifacts are no longer published.
- Drivetrain hardware values live in `DriveConstants`; CAN IDs live in `Constants.CAN`.
- The stripped robot has no cameras. Vision is disabled by default; do not enable it without
  measured camera transforms.
- Do not reintroduce superstructure code. It belongs on the next robot's branch.
- Tests must not require a roboRIO. Use `ModuleIOSim` and `GyroIO` no-op implementations.