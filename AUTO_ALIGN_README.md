# Auto Align (Current Behavior)

This document describes the current AutoAlign code behavior on `main` at commit `e5b94e6`.

## Files Involved

- `src/main/java/frc/robot/commands/AutoAllign.java`
- `src/main/java/frc/robot/RobotContainer.java`
- `src/main/java/frc/robot/subsystems/LimelightSubsystem.java`
- `src/main/java/frc/robot/subsystems/SwerveSubsystem.java`
- `src/main/java/frc/robot/Constants.java`

## Trigger / Cancel

- AutoAlign starts when driver or operator presses `A`.
- AutoAlign cancels when driver or operator presses left trigger.

## Inputs Used By AutoAlign

AutoAlign uses these values each loop:

1. `tv` (has target) from Limelight via `limelight.hasValidTarget()`
2. `tid` (tag id) from Limelight via `limelight.getTargetID()`
3. `ty` (vertical offset) from Limelight via `limelight.getY()`
4. Tag height from AprilTag field layout via `limelight.getTagHeightMeters(tagId)`
5. Driver lateral command from left stick X, passed in as `lateralInputMetersPerSecond`

## Step-by-Step Sequence (When AutoAlign Command Runs)

1. `initialize()` resets distance PID and clears `noTagFound`.
2. In each `execute()` cycle, it reads current `tagId`.
3. It checks:
   - Limelight has valid target (`tv > 0.5`), and
   - tag is allowed by `isAllowedTag(tagId)`.
4. If either check fails:
   - drivetrain stops,
   - logs `AutoAlign: no tag found` once,
   - returns from `execute()`.
5. If checks pass:
   - reads `ty`,
   - looks up tag height from field layout,
   - falls back to default tag height if needed,
   - computes distance using trigonometry:
     - `distance = abs((tagHeight - cameraHeight) / tan(ty))`
6. If computed distance is invalid (NaN/Inf):
   - drivetrain stops,
   - logs invalid distance,
   - returns.
7. If distance is valid:
   - computes forward/backward command `vx` from distance PID toward `2.2m`,
   - computes lateral command `vy` from user left-stick input (clamped),
   - commands drivetrain with `drive(vx, vy, 0.0)`.
8. `isFinished()` behavior:
   - returns `true` immediately if `noTagFound` was set,
   - otherwise returns `true` only when distance PID is at setpoint.
9. `end()` always stops drivetrain.

## Important: Rotation Behavior Right Now

- Rotation is NOT auto-controlled in current code.
- AutoAlign always commands `omega = 0.0`.

## Important: Allowed Tag IDs Right Now

- `AutoAllign.isAllowedTag()` currently allows only `10` and `26`.
- `Constants.AutoAlignConstants.REEF_TAG_IDS` now also contains `10` and `26`.

## Control Frame

AutoAlign uses `SwerveSubsystem.drive(vx, vy, omega)` which calls:

- `swerveDrive.drive(..., false, false)`

That is robot-relative command mode for AutoAlign.

## Mermaid Flowchart

```mermaid
flowchart TD
    A[Press A] --> B[initialize: reset distance PID]
    B --> C[execute loop]

    C --> D[Read tv + tid]
    D --> E{hasValidTarget and allowed tag?}
    E -- No --> F[drive.stop]
    F --> G[log no tag found once]
    G --> H[noTagFound=true]
    H --> I[isFinished true]

    E -- Yes --> J[Read ty + tag height]
    J --> K[Compute distance]
    K --> L{distance finite?}
    L -- No --> M[drive.stop + log invalid distance]
    M --> C

    L -- Yes --> N[vx from distance PID to 2.2m]
    N --> O[vy from user left-X input]
    O --> P[omega = 0]
    P --> Q[drive(vx, vy, 0)]
    Q --> R{distance PID at setpoint?}
    R -- No --> C
    R -- Yes --> S[end -> drive.stop]
```

## Quick Summary

Current AutoAlign is a shared-control command:
- Auto controls forward/backward distance to a target distance of `2.2m`.
- Driver controls left/right (lateral) while command is active.
- Robot does not rotate automatically during AutoAlign in this version.
