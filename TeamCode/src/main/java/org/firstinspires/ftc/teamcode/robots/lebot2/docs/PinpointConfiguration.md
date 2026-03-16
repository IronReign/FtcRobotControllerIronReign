# Pinpoint Odometry Configuration

This document describes the configuration and tuning procedure for the GoBilda Pinpoint odometry computer on lebot2.

## Current Configuration

**PinpointLocalizer.java** settings (as of January 2026):

| Parameter | Value | Description |
|-----------|-------|-------------|
| Encoder Resolution | goBILDA_4_BAR_POD (19.89 ticks/mm) | 4-Bar odometry pod encoder |
| X Offset (Parallel) | -180.5 mm | Pod is 180.5mm RIGHT of robot center |
| Y Offset (Perpendicular) | +131.9 mm | Pod is 131.9mm FORWARD of robot center |
| Par Direction | FORWARD | X encoder counts positive when moving forward |
| Perp Direction | FORWARD | Y encoder counts positive when strafing left |

## Pod Naming Convention

The Pinpoint uses X/Y naming for the two encoder inputs:

| Pinpoint Name | Motion Measured | Common Name |
|---------------|-----------------|-------------|
| X encoder | Forward/backward | Parallel pod |
| Y encoder | Left/right strafe | Perpendicular pod |

## Offset Measurement Procedure

The Pinpoint needs to know where each odometry pod is located relative to the robot's center of rotation.

### Step 1: Identify Robot Center

The center of rotation for a tank drive is typically the geometric center of the drivetrain (midpoint between left and right wheel axles, measured along the robot's forward axis).

### Step 2: Measure Pod Positions

For each pod, measure:
1. **Lateral distance** from robot center (left is positive, right is negative)
2. **Forward distance** from robot center (forward is positive, backward is negative)

### Step 3: Apply Offsets

In `PinpointLocalizer.java`:

```java
// Parallel (X) pod offset: lateral position (left+, right-)
// Perpendicular (Y) pod offset: forward position (forward+, backward-)
driver.setOffsets(xOffsetMM, yOffsetMM, DistanceUnit.MM);
```

**lebot2 current offsets:**
- Parallel pod is 180.5mm to the RIGHT → xOffset = -180.5
- Perpendicular pod is 131.9mm FORWARD → yOffset = +131.9

## Encoder Direction Verification

### Test Procedure

1. Run **LocalizationTest** opmode
2. Push the robot forward by hand (no motors running)
3. Check telemetry: X position should increase
4. Push the robot left (strafe)
5. Check telemetry: Y position should increase
6. Rotate robot counter-clockwise
7. Check telemetry: heading should increase

### If Directions Are Wrong

Flip the encoder direction in `PinpointLocalizer.java`:

```java
initialParDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;  // or FORWARD
initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED; // or FORWARD
```

## Three-Phase Update Pattern

The Pinpoint localizer implements a caching mechanism to minimize I2C communication:

```
Phase 1: refresh()         → Performs I2C bulk read (40 bytes)
Phase 2: update()          → Returns cached data
Phase 3: markCycleComplete() → Resets for next cycle
```

### For Competition Opmodes

`Robot.update()` manages the refresh cycle:
```java
// In Robot.update():
pinpointLocalizer.refresh();        // Start of cycle
// ... do other updates ...
// RoadRunner calls updatePoseEstimate() → uses cached data
pinpointLocalizer.markCycleComplete(); // End of cycle
```

This ensures only **one** I2C read per cycle regardless of how many times `update()` is called.

### For Standalone Tuning Opmodes

Tuning opmodes (LocalizationTest, SplineTest, ManualFeedbackTuner) explicitly manage the refresh cycle since they don't use `Robot.update()`.

## Encoder Resolution Reference

| Pod Type | Ticks/mm |
|----------|----------|
| goBILDA Swingarm | 13.26291192 |
| goBILDA 4-Bar | 19.89436789 |

lebot2 uses **4-Bar pods**.

## Troubleshooting

### Pose Drifts Over Time

- Check wheel/encoder slippage
- Verify pod offsets are accurate
- Ensure encoders are securely mounted

### Pose Jumps or Spikes

- Check for I2C communication issues
- Verify Pinpoint device status is READY
- Check for loose encoder connections

### Wrong Direction

- Flip encoder direction (FORWARD ↔ REVERSED)
- Verify hardware wiring matches expected orientation

### Incorrect Scale

- Verify encoder resolution matches actual pod type
- Double-check offset units (mm, not inches)

## Related Files

- `PinpointLocalizer.java` - Localizer implementation
- `TankDrivePinpoint.java` - Drive class using Pinpoint
- `tuning/LocalizationTest.java` - Manual drive test
- `tuning/SplineTest.java` - Spline trajectory test
- `tuning/ManualFeedbackTuner.java` - Feedback gain tuner
