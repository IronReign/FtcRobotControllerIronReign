# Vision-Pinpoint Pose Fusion

This document describes how we integrate Limelight 3A AprilTag localization with Pinpoint odometry to correct accumulated drift during matches.

## Overview

| System | Type | Frequency | Accuracy | Use Case |
|--------|------|-----------|----------|----------|
| **Pinpoint** | Relative (odometry) | 1500 Hz | Drifts over time | Continuous tracking |
| **Limelight** | Absolute (AprilTags) | ~90 Hz | Fixed accuracy when visible | Periodic correction |

The Pinpoint tracks movement at high frequency but accumulates small errors that compound over time. The Limelight provides absolute field position when AprilTags are visible, which we use to periodically reset the Pinpoint and eliminate drift.

## Design Principles

1. **Triggered updates only** - No automatic corrections. Driver button in teleop, explicit calls in auton.
2. **MT2 preferred** - MegaTag2 uses Pinpoint heading for better single-tag accuracy.
3. **MT1 fallback** - When heading is unknown (robot restart), MT1 provides full pose from tag geometry.
4. **Simple rejection** - Only reject when no valid botpose. No complex confidence scoring.
5. **Starting position selection** - Initial pose set during init based on alliance and field position.

## Starting Position Selection

During `init_loop`, the driver selects alliance color and starting position:

### Button Mapping (Init Phase)

| Button | Action |
|--------|--------|
| X | Blue Alliance |
| B | Red Alliance |
| A | Audience Wall Position |
| Y | Goal Wall Position |
| Back | Unknown Position (teleop testing) |
| Guide | Calibration Position (MT1/MT2 comparison) |

### Starting Positions

| Position | Description | Vision Notes |
|----------|-------------|--------------|
| **AUDIENCE** | Touching audience wall | Can see goal AprilTag (long range, may have glare issues) |
| **GOAL** | Touching goal wall | Too close for vision, must back up before fix |
| **UNKNOWN** | Position unknown | Forces MT1 mode until first vision fix seeds Pinpoint |
| **CALIBRATION** | Field center, facing red goal | For MT1/MT2 comparison testing - seeds known heading |

### Starting Poses (RoadRunner coordinates, inches)

```java
// Red alliance poses
RED_AUDIENCE_POSE = new Pose2d(-36, -60, Math.toRadians(90));   // Facing toward goal
RED_GOAL_POSE = new Pose2d(-36, 60, Math.toRadians(-90));       // Facing away from goal

// Blue poses are reflections across X axis (Y and heading negated)
BLUE_AUDIENCE_POSE = new Pose2d(-36, 60, Math.toRadians(-90));
BLUE_GOAL_POSE = new Pose2d(-36, -60, Math.toRadians(90));

// Calibration pose (alliance-independent)
// In DECODE coords: X+ toward audience, facing red goal ≈ 135°
CALIBRATION_POSE = new Pose2d(0, 0, Math.toRadians(135));  // Field center, facing red goal
```

**Note**: These values are placeholders. Measure and calibrate for actual field setup.

### MT1 Forced Mode

When starting position is UNKNOWN:
1. `initialPositionSet` flag is false
2. Vision is set to force MT1 mode (ignores IMU heading)
3. First successful vision correction sets `initialPositionSet = true`
4. Subsequent corrections use MT2 (better single-tag accuracy)

## MegaTag1 vs MegaTag2

| Feature | MegaTag1 (MT1) | MegaTag2 (MT2) |
|---------|----------------|----------------|
| Heading source | Derived from tag geometry | Provided by robot IMU |
| Single-tag accuracy | Good | Better (IMU constrains solution) |
| Multi-tag benefit | Significant | Moderate |
| Requirements | Just camera | Camera + calibrated IMU |
| Use when | Heading unknown | Heading known |

### MT2 Flow
```
Pinpoint heading (degrees)
        │
        ▼
  limelight.updateRobotOrientation(heading)
        │
        ▼
  result.getBotpose_MT2()
        │
        ▼
  Enhanced pose with IMU fusion
```

### Automatic Fallback
The Vision subsystem automatically falls back to MT1 if MT2 returns null:
```java
Pose3D mt2Pose = result.getBotpose_MT2();
if (mt2Pose != null) {
    usingMT2 = true;
    botPose = mt2Pose;
} else {
    usingMT2 = false;
    botPose = result.getBotpose();  // MT1 fallback
}
```

## When to Correct

### Teleop - Driver Triggered
**Button**: Back button on gamepad1

**When to press**:
- Robot is stationary (not moving)
- AprilTag is clearly visible (check "Has BotPose: YES" in telemetry)
- Before critical scoring actions
- After collisions or suspected drift

**Feedback**: Controller rumbles briefly when correction is applied.

### Autonomous - Checkpoint Triggered
Corrections are automatically applied at these phases:
- `WAITING_PRELOADS` - After launching preloaded balls
- `WAITING_LAUNCH` - After launching collected balls

These are ideal times because:
- Robot is stationary
- Robot is facing the goal (AprilTag visible)
- About to navigate to next location (fresh pose helps accuracy)

## Coordinate Systems

**Critical**: Limelight and RoadRunner must use compatible coordinate systems.

### DECODE Field Layout (2024-2025 Season)

The DECODE field has an **inverted configuration** compared to standard FTC square fields:

| Feature | Standard Square Field | DECODE Field |
|---------|----------------------|--------------|
| Red Wall | Right side (from audience) | **Left side** (from audience) |
| Blue Wall | Left side (from audience) | **Right side** (from audience) |
| X+ Direction | Toward rear of field | **Toward audience** (front) |

This inversion affects coordinate transforms. Always verify against actual field setup.

```
                    AUDIENCE (Front)
                         ↑
                         X+
              ┌──────────┬──────────┐
              │          │          │
              │   RED    │   BLUE   │
  Blue Wall → │  ZONE    │   ZONE   │ ← Red Wall
      Y-      │          │          │      Y+
              │          │          │
              └──────────┴──────────┘
                    (Field Rear)
```

### FTC Global Coordinate System (DECODE)
- Origin: Field center
- X+: Toward **audience** (front of field)
- Y+: From Red Wall toward Blue Wall (robot's left when facing audience)
- Z+: Upward
- Heading 0°: Facing toward audience (X+ direction)
- Units: **Inches** (FTC standard) or **Meters** (Limelight)

### Limelight Field Coordinates
The Limelight uses the FTC global coordinate system when loaded with the DECODE field map:
- Origin: Field center
- X+: Toward audience
- Y+: Toward Blue Wall
- Heading 0°: Facing audience
- Units: **Meters**

Example AprilTag positions (DECODE field map):
- Red Goal (Tag 24): approximately (-1.48, 1.41) meters
- Blue Goal (Tag 20): approximately (-1.48, -1.41) meters

### RoadRunner Coordinates
- Origin: Configurable (often robot start position)
- X+: Robot forward direction at heading 0
- Y+: Robot left direction
- Heading 0°: Configurable
- Units: **Inches**

### Conversion in Code
```java
// Limelight meters → RoadRunner inches
double xInches = xMeters * 39.3701;
double yInches = yMeters * 39.3701;

// Heading is already in radians from both systems
Pose2d visionPose = new Pose2d(xInches, yInches, headingRad);
```

### If Axes Are Misaligned
If testing reveals axes are swapped or negated, modify `applyVisionPoseCorrection()`:
```java
// Example: If Limelight Y maps to RoadRunner X
Pose2d visionPose = new Pose2d(yInches, xInches, headingRad);

// Example: If Limelight X is negated
Pose2d visionPose = new Pose2d(-xInches, yInches, headingRad);
```

**DECODE Field Note**: The DECODE field has inverted X direction (toward audience instead of toward rear). If your RoadRunner is configured for a standard field, you may need to negate X:
```java
// DECODE inversion if RoadRunner expects standard orientation
Pose2d visionPose = new Pose2d(-xInches, yInches, headingRad);
```

## Code Architecture

### Files Modified

| File | Changes |
|------|---------|
| `Vision.java` | MT2 support, heading extraction, telemetry |
| `Robot.java` | `applyVisionPoseCorrection()`, feed heading to Vision |
| `DriverControls.java` | Back button handler |
| `Autonomous.java` | Checkpoint corrections |

### Data Flow
```
┌─────────────────────────────────────────────────────────────┐
│                     Robot.update()                          │
├─────────────────────────────────────────────────────────────┤
│  1. vision.updateRobotOrientation(driveTrain.getHeading())  │
│                          │                                  │
│                          ▼                                  │
│  2. subsystem.readSensors()  ←── Vision.calc() extracts    │
│                                   MT2 pose (or MT1 fallback)│
│                          │                                  │
│                          ▼                                  │
│  3. [Driver presses Back] or [Auton checkpoint reached]     │
│                          │                                  │
│                          ▼                                  │
│  4. robot.applyVisionPoseCorrection()                       │
│         │                                                   │
│         ▼                                                   │
│     driveTrain.setPose(visionPose)                         │
│         │                                                   │
│         ▼                                                   │
│     PinpointLocalizer.setPose() ←── Pinpoint resets        │
└─────────────────────────────────────────────────────────────┘
```

## Telemetry

### Robot Telemetry
```
Alliance: RED
Start Pos: AUDIENCE
Balls: 2/3
```

If starting position is UNKNOWN and not yet corrected:
```
Start Pos: UNKNOWN (unset)
```

### Vision Telemetry (always visible)
```
Has BotPose: YES
MT Mode: MT2 (active)
Distance to Goal: 2.34 m (92.1 in)
MT1 Pos: (-1.23, 0.45) m
MT1 Heading: 43.8°
MT2 Pos: (-1.24, 0.44) m
MT2 Heading: 45.2°
MT Diff: 0.015 m, 1.4°
```

The telemetry now shows **both** MT1 and MT2 poses simultaneously for comparison, along with the difference between them. This is useful for:
- Verifying MT2 is receiving correct heading data
- Diagnosing coordinate system mismatches
- Understanding how much MT2 improves accuracy over MT1

### What to Look For
- **"Start Pos: ... (unset)"** - Initial position not known, MT1 forced
- **"Has BotPose: YES"** - Safe to apply correction
- **"MT Mode: MT2 (active)"** - Using IMU-fused localization (preferred)
- **"MT Mode: MT1 (active)"** - Fallback mode (either forced or MT2 unavailable)
- **"MT Diff"** - Should be small (< 0.1m, < 5°) when heading is correct
- **Large MT Diff heading** - Indicates heading fed to MT2 doesn't match reality

## Testing Procedures

### Test 0: Starting Position Selection
**Goal**: Verify init position selection works

1. Start opmode, don't press Start yet (init_loop running)
2. Press B (Red alliance) - verify telemetry shows "Alliance: RED"
3. Press X (Blue alliance) - verify telemetry shows "Alliance: BLUE"
4. Press A (Audience) - verify telemetry shows "Start Pos: AUDIENCE"
5. Press Y (Goal) - verify telemetry shows "Start Pos: GOAL"
6. Press Back (Unknown) - verify telemetry shows "Start Pos: UNKNOWN (unset)"
7. Press Guide (Calibration) - verify telemetry shows "Start Pos: CALIBRATION"
8. With AUDIENCE selected, press Start
9. Verify Pinpoint pose matches expected starting pose for that alliance

### Test 0.5: MT1/MT2 Calibration Comparison
**Goal**: Verify MT2 is receiving correct heading and compare accuracy

**Setup**:
1. Place robot at exact field center (tile intersection)
2. Rotate robot to face red goal AprilTag (approximately -45° from audience direction)
3. Measure the actual heading with a protractor or field reference

**Procedure**:
1. Start opmode, don't press Start yet
2. Press Guide button to select CALIBRATION position
3. Verify telemetry shows "Start Pos: CALIBRATION"
4. Press Start to begin opmode
5. Robot should initialize with pose (0, 0) and heading -45°
6. Look at Vision telemetry - both MT1 and MT2 poses should be visible
7. Compare:
   - **MT1 Pos** vs expected (0, 0): Should be close if coordinate system is correct
   - **MT1 Heading** vs actual: MT1 derives heading from tag geometry alone
   - **MT2 Heading** vs actual: Should match the -45° we fed from Pinpoint
   - **MT Diff**: Shows how different MT1 and MT2 are

**Interpretation**:
- If MT2 heading matches fed heading (-45°) but MT1 is different → MT2 is working, MT1 shows pure vision heading
- If both MT1 and MT2 have same heading → Either heading fed is matching reality, or MT2 isn't getting heading
- Large positional difference → Coordinate system mismatch between Limelight and RoadRunner
- MT1 position should be trusted for field position verification (it doesn't depend on external heading)

### Test 1: Heading Verification
**Goal**: Confirm Vision heading matches Pinpoint heading

1. Run teleop opmode
2. Open FTC Dashboard, find Vision telemetry
3. Manually rotate robot in place
4. Compare "Robot Heading" (Vision) with Pinpoint heading
5. They should track together within a few degrees

**If off by 90° or 180°**: Coordinate transform needed in `applyVisionPoseCorrection()`

### Test 2: Position Verification
**Goal**: Confirm Vision position is accurate

1. Place robot at a known field position (e.g., corner tile intersection)
2. Calculate expected position in Limelight coordinates (meters from field center)
3. Compare with "Robot Pos" in Vision telemetry
4. Should match within ~5cm

**If axes are swapped**: X and Y need to be exchanged in code

### Test 3: Pose Correction
**Goal**: Confirm correction actually works

1. Start robot at known position
2. Drive around for 30-60 seconds to accumulate drift
3. Return to starting position (visual estimate)
4. Note Pinpoint pose - it will show drift from actual position
5. Face AprilTag, confirm "Has BotPose: YES"
6. Press Back button
7. Pinpoint pose should jump to match Vision pose
8. Drive to a known target - robot should arrive accurately

### Test 4: MT2 vs MT1 Comparison
**Goal**: Verify MT2 is being used and compare accuracy

1. Select CALIBRATION position (Guide during init) - this sets a known heading
2. After Start, should show "MT Mode: MT2 (active)"
3. Look at both MT1 and MT2 poses in telemetry - they should both be visible
4. Compare "MT Diff" - shows position and heading difference between the two
5. A small MT Diff (< 0.05m, < 3°) indicates MT2 agrees with MT1
6. A large heading diff indicates the heading fed to MT2 doesn't match MT1's derived heading
7. Select UNKNOWN position (Back during init) and restart
8. After Start, should show "MT Mode: MT1 (active)" (forced mode)
9. Both MT1 and MT2 poses still shown, but only MT1 is used
10. Press Back button to apply vision correction
11. Telemetry should change to "Start Pos: UNKNOWN" (no longer showing "unset")
12. Should now show "MT Mode: MT2 (active)" (initial position now known)

### Test 5: Autonomous Corrections
**Goal**: Verify corrections happen at checkpoints

1. Add logging to `applyVisionPoseCorrection()`:
   ```java
   public boolean applyVisionPoseCorrection() {
       if (!vision.hasBotPose()) {
           System.out.println("Vision correction SKIPPED - no botpose");
           return false;
       }
       System.out.println("Vision correction APPLIED at " +
           String.format("(%.2f, %.2f)", vision.getRobotX(), vision.getRobotY()));
       // ... rest of method
   }
   ```
2. Run autonomous
3. Check logcat for correction messages at expected checkpoints

## Troubleshooting

### "Has BotPose: no" when facing tag
- Check Limelight pipeline is set correctly (red vs blue alliance)
- Verify camera can see the AprilTag (check Limelight web interface)
- Ensure "Full 3D" is enabled in pipeline settings
- Check camera is not blocked or dirty

### Position jumps wildly after correction
- Coordinate system mismatch - see "If Axes Are Misaligned" section
- **DECODE field inversion** - X axis points toward audience, not rear. May need to negate X in transform.
- Camera offset not configured in Limelight web interface
- Wrong AprilTag map loaded on Limelight (must use DECODE field map)

### MT2 always falling back to MT1
- `updateRobotOrientation()` not being called before `calc()`
- Heading value is invalid (NaN or extreme values)
- Check Limelight firmware is up to date

### Correction doesn't seem to help accuracy
- Verify correction is actually being applied (add logging)
- Check that Pinpoint `setPose()` is working (log before/after)
- Vision pose itself may be inaccurate - verify with Test 2

## Future Improvements

1. **Automatic correction when stationary**: Detect when robot velocity is near zero and vision is stable, auto-apply correction without driver input.

2. **Pose smoothing**: Instead of hard reset, blend vision and odometry poses using a Kalman filter.

3. **Multi-frame validation**: Require N consecutive similar vision poses before applying correction.

4. **Vibration detection**: Automatically disable corrections when launcher/intake vibration is detected.

## Related Documentation

- `PinpointConfiguration.md` - Pinpoint odometry setup
- `ActionsGuide.md` - RoadRunner action patterns
- `LauncherRampDesign.md` - Launcher subsystem (vibration source)

## References

- [FTC Field Coordinate System (DECODE)](https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html) - Official FTC coordinate system docs, including DECODE-specific layout
- [Limelight MegaTag2 Documentation](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2)
- [Limelight FTC Programming Guide](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming)
- [GoBilda Pinpoint User Guide](https://www.gobilda.com/content/user_manuals/3110-0002-0001%20User%20Guide.pdf)
