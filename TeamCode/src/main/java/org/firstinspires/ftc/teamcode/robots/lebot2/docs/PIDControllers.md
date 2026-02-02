# Lebot2 PID Controllers Reference

This document catalogs every PID controller and PID-like feedback loop in the Lebot2 codebase. It exists to help the team understand what we have, where the overlap is, and where consolidation makes sense.

---

## Overview

There are **5 distinct PID loops** across 3 files:

| # | Name | Type | File | Controls | Used By |
|---|---|---|---|---|---|
| 1 | Legacy Heading PID | PIDController class | TankDrivePinpoint | Robot heading (turns) | `turnToHeadingPID()`, `turnToTarget()`, `centerOnTarget()` |
| 2 | RR Turn PID | Hand-rolled P+I+D | TankDrivePinpoint (TurnAction) | Robot heading (turns) | `LazyTurnAction`, `turnToHeading()`, health check turns |
| 3 | Position Drive Distance PID | PIDController class | TankDriveActions (PositionDriveAction) | Forward/reverse drive power | `driveTo()`, `driveToReversed()`, `driveThrough()` |
| 4 | Position Drive Heading PID | PIDController class | TankDriveActions (PositionDriveAction) | Steering correction while driving | Same as #3 |
| 5 | Flywheel Velocity PIDF | DcMotorEx built-in | Launcher | Flywheel motor speed | Launcher SPINNING/FIRING states |

**Key redundancy:** #1 and #2 both control heading during turns but use completely different implementations. #1 is the legacy PID turn, #2 is the RoadRunner-based turn with motion profiling.

---

## 1. Legacy Heading PID

**File:** `TankDrivePinpoint.java`
**Dashboard:** `Lebot2_TankDrivePinpoint`

### Configuration

| Parameter | Value | Location | Description |
|---|---|---|---|
| `HEADING_PID.p` | 0.03 | TankDrivePinpoint:127 | Proportional gain |
| `HEADING_PID.i` | 0.04 | TankDrivePinpoint:127 | Integral gain |
| `HEADING_PID.d` | 0.0 | TankDrivePinpoint:127 | Derivative gain |
| `HEADING_TOLERANCE` | 1.5 deg | TankDrivePinpoint:128 | `onTarget()` threshold |
| Integral cut-in | 4.0 deg | TankDrivePinpoint:232 | Hardcoded in constructor |
| Input range | -180 to 180 | TankDrivePinpoint:230 | Continuous (wraps) |
| Output range | -1 to 1 | TankDrivePinpoint:231 | Motor power |

### How It Works

Standard PIDController from our util package. Input is current heading in degrees, setpoint is target heading. Output is motor power applied as differential (left = +correction, right = -correction). Uses `setContinuous(true)` to handle 180/-180 wraparound.

### Where It's Used

- **`executeTurnToHeading()`** (line 329) — Legacy absolute heading turn. Called by `turnToHeadingPID()` (line 379).
- **`executeTurnToTarget()`** (line 347) — Vision-based turn where input is limelight `tx` offset and setpoint is 0. Called by `turnToTarget()` (line 388).
- **`executeCenteringOnTarget()`** (line 434) — Continuous vision tracking. Called by `centerOnTarget()` (line 420), triggered by Y button in teleop.

### Notes

- Code comment at line 123 says this PID is "too weak for small errors"
- `turnToHeadingPID()` is explicitly marked as poorly tuned, with a comment recommending `turnToHeading()` (RR-based) instead
- The vision centering methods (`turnToTarget`, `centerOnTarget`) still actively use this PID — no RR equivalent exists for continuous vision tracking

---

## 2. RoadRunner Turn PID (Hand-Rolled)

**File:** `TankDrivePinpoint.java` (inner class `TurnAction`, line 814)
**Dashboard:** `TankDrivePinpoint_Params`

### Configuration

| Parameter | Value | Location | Description |
|---|---|---|---|
| `turnGain` | 25.0 | TankDrivePinpoint:108 | P term — applied to heading error in **radians** |
| `turnVelGain` | 3.0 | TankDrivePinpoint:109 | D term — applied to angular velocity error |
| `turnIGain` | 0.0 | TankDrivePinpoint:110 | I term — applied to accumulated heading error (code default; may be tuned on Dashboard) |
| `turnICutIn` | 5.0 deg | TankDrivePinpoint:111 | Only integrate when heading error < this |
| `turnFeedforwardScale` | 1.0 | TankDrivePinpoint:112 | Scales motion profile feedforward (0=pure feedback) |
| `turnCompleteTolerance` | 2.0 deg | TankDrivePinpoint:115 | Heading must be within this to declare settled |
| `turnCompleteVelTolerance` | 0.1 rad/s | TankDrivePinpoint:116 | Angular velocity must be below this to declare settled |
| `turnCompleteTimeout` | 2.0 s | TankDrivePinpoint:117 | Hard cap on settling time after motion profile ends |

### How It Works

Not a PID class — raw math at lines 900-903:

```
ffTerm = feedforwardVelocity * turnFeedforwardScale
pTerm  = turnGain * headingError              (radians)
iTerm  = turnIGain * accumulatedHeadingError   (radians * seconds)
dTerm  = turnVelGain * velocityError           (rad/s)
output = ffTerm + pTerm + iTerm + dTerm
```

The output goes through RR's tank kinematics and motor feedforward model (kS/kV/kA) with voltage compensation before reaching the motors.

**Two phases:**
1. **Profile phase** — follows a time-based motion profile (trapezoidal velocity). Feedforward provides most of the power, PID corrects tracking error.
2. **Settling phase** — motion profile is done, feedforward is 0, PID drives error to zero. Turn is complete when BOTH heading tolerance AND velocity tolerance are met simultaneously, OR the timeout expires.

### Where It's Used

- **`LazyTurnAction`** (TankDriveActions:421) — builds an RR `turnTo()` action, which creates a TurnAction. Used in every `driveTo()` / `driveToReversed()` / `driveThrough()` call (initial and final turns).
- **`LazyBearingTurnAction`** (TankDriveActions:468) — same mechanism but computes target heading from bearing to a position.
- **`turnToHeading()`** (TankDrivePinpoint:367) — preferred turn method, builds an RR turnTo action directly. Used by health check turns.

### Notes

- The settling requirement that BOTH heading AND velocity must be within tolerance simultaneously is the primary source of oscillation time waste. The robot bounces between satisfying one but not the other.
- `turnCompleteTimeout` (2.0s) caps worst-case settling, but this is 2 seconds of wasted time per turn.
- The `turnGain` of 25.0 operates on **radians** — this is equivalent to ~0.44 per degree, much stronger than the legacy PID's 0.03 per degree.
- Integral anti-windup: resets to 0 if error exceeds `turnICutIn` (line 893).

---

## 3. Position Drive Distance PID

**File:** `TankDriveActions.java` (inner class `PositionDriveAction`, line 213)
**Dashboard:** `Lebot2_TankDriveActions`

### Configuration

| Parameter | Value | Location | Description |
|---|---|---|---|
| `DISTANCE_PID.p` | 0.04 | TankDriveActions:50 | Proportional gain |
| `DISTANCE_PID.i` | 0.04 | TankDriveActions:50 | Integral gain |
| `DISTANCE_PID.d` | 2.0 | TankDriveActions:50 | Derivative gain |
| `DIST_PID_I_CUTIN` | 8.0 in | TankDriveActions:51 | Only integrate when within 8 inches |
| `POSITION_TOLERANCE` | 1.5 in | TankDriveActions:55 | Along-track completion threshold |
| `MAX_DRIVE_POWER` | 0.8 | TankDriveActions:56 | Power cap |
| `SETTLING_TIMEOUT_MS` | 500 ms | TankDriveActions:63 | Safety timeout after overshoot only |
| Input range | -50 to 50 | TankDriveActions:234 | Inches |
| Output range | -1 to 1 | TankDriveActions:235 | Motor power |

### How It Works

Uses our PIDController class. Input is always 0, setpoint is along-track distance (projection of robot-to-target vector onto drive direction). This means positive setpoint = target ahead, negative = overshot.

Output is drive power, scaled by `cos(headingError)` — if heading is 90 degrees off, drive power goes to zero (robot only steers). Then slew-rate limited for smooth acceleration. In reverse mode, drive power is negated.

### Completion Conditions

1. `|alongTrack| < POSITION_TOLERANCE` (1.5") — primary exit
2. Settling timeout (500ms) — but ONLY triggers after overshoot (`alongTrack <= 0`)
3. **No stall/approach timeout** — if robot stops short of target without overshooting, this action runs forever. Relies on mission-level timeouts for safety.

### Where It's Used

Every `driveTo()`, `driveToReversed()`, and `driveThrough()` call creates a PositionDriveAction as the drive segment between turns.

### Notes

- FLOAT zero-power behavior during driving (set at line 248) — gives PID full deceleration authority but means friction can stall the robot short of target on low battery.
- Switches back to BRAKE on completion (line 310).
- Slew rate limiting adds ~0.5s to acceleration phase (ACCEL_SLEW_RATE = 0.05 per tick at ~50ms loop = ~1 second to reach full power from standstill).

---

## 4. Position Drive Heading PID

**File:** `TankDriveActions.java` (inner class `PositionDriveAction`, line 213)
**Dashboard:** `Lebot2_TankDriveActions`

### Configuration

| Parameter | Value | Location | Description |
|---|---|---|---|
| `HEADING_DRIVE_PID.p` | 0.03 | TankDriveActions:52 | Proportional gain |
| `HEADING_DRIVE_PID.i` | 0.03 | TankDriveActions:52 | Integral gain |
| `HEADING_DRIVE_PID.d` | 0.001 | TankDriveActions:52 | Derivative gain |
| `HEAD_PID_I_CUTIN` | 5.0 deg | TankDriveActions:53 | Only integrate when within 5 degrees |
| Input range | -180 to 180 | TankDriveActions:240 | Degrees, continuous wrap |
| Output range | -1 to 1 | TankDriveActions:241 | Steering correction |

### How It Works

Uses our PIDController class. Input is heading error in degrees (locked heading minus current heading). Setpoint is 0 (maintain the heading locked at initialization). Output is steering correction mixed into tank drive:

```
leftPower  = drivePower + steerPower
rightPower = drivePower - steerPower
```

### Where It's Used

Same as #3 — runs alongside the distance PID inside every PositionDriveAction.

### Notes

- The locked heading is set ONCE at initialization from the robot's current heading (line 257). It is NOT recomputed during the drive. This prevents phantom Pinpoint drift from corrupting the heading reference.
- Very similar gains to the legacy heading PID (#1) — both use P=0.03 — but this one includes integral (0.03) and a tiny derivative (0.001).

---

## 5. Flywheel Velocity PIDF

**File:** `Launcher.java`
**Dashboard:** `Lebot2_Launcher`

### Configuration

| Parameter | Value | Location | Description |
|---|---|---|---|
| `NEW_P` | 400 | Launcher:51 | Proportional gain |
| `NEW_I` | 0 | Launcher:52 | Integral gain |
| `NEW_D` | 0 | Launcher:53 | Derivative gain |
| `NEW_F` | 20 | Launcher:54 | Feedforward gain |
| `MIN_LAUNCH_SPEED` | 725 deg/s | Launcher:90 | Target velocity |
| `SPEED_TOLERANCE` | 30 deg/s | Launcher:91 | "At speed" threshold |

### How It Works

Uses the REV hub's built-in velocity PIDF controller via `DcMotorEx.setVelocityPIDFCoefficients()`. Applied to both `flywheel` and `flywheelHelp` motors. The PIDF runs on the hub firmware, not in user code — we just set the coefficients and call `setVelocity(targetSpeed, AngleUnit.DEGREES)`.

Coefficients are re-applied every `calc()` cycle (line 296-298) to allow live Dashboard tuning.

### Where It's Used

- Launcher SPINNING_UP state — ramps flywheel to target speed
- Launcher READY state — maintains speed while waiting for fire command
- Launcher FIRING/LIFTING states — maintains speed during ball contact
- Launcher MANUAL state — for testing

### Notes

- Original factory PIDF coefficients are saved at init (line 178: `pidfOrig`) but never restored.
- The `NEW_*` naming suggests these were recently tuned — consider renaming to `FLYWHEEL_P/I/D/F` for clarity.
- No integral term (I=0) means steady-state error is corrected by feedforward only. This may be intentional if the F term is well-calibrated.

---

## Redundancy Analysis

### Heading Turn Control (Overlap between #1 and #2)

| Aspect | Legacy PID (#1) | RR Turn PID (#2) |
|---|---|---|
| Implementation | PIDController class | Hand-rolled math |
| Gains | P=0.03, I=0.04, D=0.0 | P=25.0 (rad), I=0.0, D=3.0 |
| Units | Degrees | Radians (P,I) / rad/s (D) |
| Motion profiling | No | Yes (trapezoidal) |
| Feedforward | No | Yes (kS/kV/kA + voltage comp) |
| Settling criteria | `onTarget()` (position only) | Position AND velocity AND timeout |
| Used by | Vision centering, legacy turns | All autonomous nav, tuning missions |

These do the same job (turn robot to heading) but with very different approaches. The legacy PID is simpler but less capable. The RR turn has motion profiling and voltage compensation but more complex settling behavior.

### Heading-While-Driving Control (Only in #4)

The heading PID inside PositionDriveAction (#4) has no overlap — it's the only controller that maintains heading during straight-line driving. However, its gains (P=0.03) are very similar to the legacy heading PID (#1), suggesting they were derived from the same tuning.

---

## Dashboard Config Summary

| Dashboard Section | Parameters |
|---|---|
| `Lebot2_TankDrivePinpoint` | `HEADING_PID`, `HEADING_TOLERANCE` |
| `TankDrivePinpoint_Params` | `turnGain`, `turnVelGain`, `turnIGain`, `turnICutIn`, `turnFeedforwardScale`, `turnCompleteTolerance`, `turnCompleteVelTolerance`, `turnCompleteTimeout`, `maxAngVel`, `maxAngAccel` |
| `Lebot2_TankDriveActions` | `DISTANCE_PID`, `DIST_PID_I_CUTIN`, `HEADING_DRIVE_PID`, `HEAD_PID_I_CUTIN`, `POSITION_TOLERANCE`, `MAX_DRIVE_POWER`, `ACCEL_SLEW_RATE`, `DECEL_SLEW_RATE_*`, `SETTLING_TIMEOUT_MS`, `*_TURN_SKIP_TOLERANCE` |
| `Lebot2_Launcher` | `NEW_P`, `NEW_I`, `NEW_D`, `NEW_F`, `MIN_LAUNCH_SPEED`, `SPEED_TOLERANCE` |
