# Post-Competition Cleanup and Turret Subsystem

This document explains the rationale behind changes made after the competition commits (`0d4e717c`, `731e1f45`, `66ea3105`). Those commits were rapid modifications under time pressure during a competition. This cleanup addresses bugs introduced by those shortcuts and completes the turret subsystem that was started at competition.

## Background

During competition, several changes were made quickly:
- The Limelight camera had connection issues, so the LED subsystem was gutted to show only a binary vision lock signal for debugging.
- The audience-side autonomous had two critical bugs that made it non-functional (setState calls without `else` that immediately overwrote the intended state).
- D-pad controls were repurposed for manual turret control, losing the `changeStar()` and slow mode toggle.
- A new turret subsystem was added with basic vision PID tracking but no pose-based fallback, no mechanical limits, and incomplete three-phase compliance.

---

## Phase 1: Autonomous Bug Fixes

**File:** `Autonomous.java`

### Bug 1: WAITING_LAUNCH setState overwrite

The audience-side auton needed to go to `START_RETURN_TO_FIRE` after launching, but the code set that state and then unconditionally set `START_BALL_ROW` on the next line, overwriting the audience branch:

```java
// BEFORE (broken):
if (audience) { setState(START_RETURN_TO_FIRE); }
setState(START_BALL_ROW);  // always runs, overwrites above

// AFTER (fixed):
if (audience) { setState(START_RETURN_TO_FIRE); }
else { setState(START_BALL_ROW); }
```

**Why this matters:** Without the `else`, audience-side auton skipped the return-to-fire navigation entirely, jumping straight to ball collection from the wrong position.

### Bug 2: WAITING_RETURN setState overwrite

Identical pattern in WAITING_RETURN. Audience needed `COMPLETE` but got overwritten by unconditional `START_TARGETING`:

```java
// BEFORE (broken):
if (IS_AUDIENCE_START) { setState(COMPLETE); }
setState(START_TARGETING);  // always runs

// AFTER (fixed):
if (IS_AUDIENCE_START) { setState(COMPLETE); }
else { setState(START_TARGETING); }
```

### Bug 3: START_TARGETING only set behavior for audience

The `robot.setBehavior(Robot.Behavior.TARGETING)` call was wrapped in an audience-only check, so goal-side auton never entered targeting behavior:

```java
// BEFORE: only audience got TARGETING behavior
if (audience) { robot.setBehavior(Behavior.TARGETING); }

// AFTER: both sides need TARGETING
robot.setBehavior(Robot.Behavior.TARGETING);
```

### Bug 4: WAITING_TARGET guard commented out

The WAITING_TARGET state had its completion guard (`robot.getBehavior() == MANUAL`) commented out, meaning it immediately fell through to START_LAUNCH without waiting for targeting to complete. The code also had duplicate audience/goal branches doing identical work. Fixed by restoring the guard and collapsing to a single branch.

### Bug 5: SKIP_LAUNCH path commented out

The `else` branch for `SKIP_LAUNCH` mode (which navigates to the opposing base for LEAVE points without firing) was commented out. Restored so the robot can be configured to just leave the starting position when launching isn't viable.

---

## Phase 2: Turret Subsystem Rewrite

**File:** `Turret.java`

### Core Design: One PID, Three Error Sources

The key insight driving this rewrite is that all turret modes are fundamentally the same operation: a PID loop driving a motor to minimize angular error in degrees. The only thing that changes is where the error comes from:

| Mode | Error Source | When Used |
|------|-------------|-----------|
| **Vision tracking** | `vision.getTx()` - camera degrees off-center | Vision has a target |
| **Bearing tracking** | `desiredTurretAngle - currentTurretAngle` (pose-computed) | No vision, but have odometry pose |
| **Locked forward** | `0 - currentTurretAngle` | Safety fallback, turret disabled |

This means a single `PIDController` with a single set of tuning constants handles all three modes. The mode switch is just which value gets fed to `setInput()` and `setSetpoint()`.

### Why not separate PIDs?

The turret motor, gearing, and inertia are the same regardless of error source. Vision tx and pose-computed bearing are both expressed in degrees. Using one PID means one set of constants to tune, and mode transitions are seamless (no PID reset, no different response characteristics).

### Behavior Enum

```
TRACKING  - Default. Uses vision when available, bearing fallback otherwise.
LOCKED    - PID holds turret at 0 degrees (forward). Safety fallback.
```

There is no manual/idle mode. The turret is always under PID control. Manual control was only needed during initial bring-up and can be done through a test mission if needed. The driver has an A button toggle between TRACKING and LOCKED in case the turret needs to be disabled during a match.

### Soft Limits via Target Clamping

The turret has approximately 210 degrees of mechanical range (~105 degrees each direction from center). Rather than applying power ramp-down near the limits, we simply clamp the PID target angle to `[CCW_LIMIT_DEG, CW_LIMIT_DEG]` before computing error.

**Why clamping is sufficient:** When the target is clamped to a limit, the PID error naturally decreases as the turret approaches that limit, causing the PID to decelerate smoothly. There's no need for a separate slow-down zone because the PID already handles deceleration. This is simpler and more predictable.

### Three-Phase Compliance

The original turret wrote motor power directly in `calc()`, violating the subsystem contract:

- `readSensors()`: Read turret encoder, compute `turretAngleDeg` from ticks
- `calc()`: Determine error source, run PID, write to `stagedPower` (not the motor)
- `act()`: `turretMotor.setPower(stagedPower)`

This ensures motor writes happen in a predictable order and at a consistent point in the loop, matching how all other subsystems operate.

### Bearing Computation

When vision is unavailable, the turret uses odometry to point at the goal:

```java
double bearingRad = FieldMap.bearingTo(currentPose, goalPose);
double chassisRad = currentPose.heading.toDouble();
double desiredTurretDeg = normalizeDeg(Math.toDegrees(bearingRad - chassisRad));
```

This computes the angle from the chassis heading to the goal, which is the turret angle needed to point at the goal. The bearing is alliance-aware via `FieldMap.getPose(FieldMap.GOAL, Robot.isRedAlliance)`.

### Calibration Required

The following constants need to be measured on the physical robot and entered via FTC Dashboard:
- `TURRET_CENTER_TICKS`: Encoder reading when turret is manually centered
- `CW_LIMIT_TICKS` / `CCW_LIMIT_TICKS`: Encoder readings at each mechanical stop
- `TICKS_PER_DEGREE`: Computed from `(CW_LIMIT_TICKS - CCW_LIMIT_TICKS) / 210.0`
- `TURRET_PID`: Start with the existing vision PID values (P=0.04, I=0.001, D=2.0) and adjust for turret inertia

---

## Phase 3: LED, Launcher, and Controls

### LED Priority State Machine

**File:** `LEDStatus.java`

During competition, the LED state machine was replaced with a binary vision lock indicator for debugging. This restores a priority-based state machine that shows the driver the most actionable information at any moment.

The design challenge was that multiple conditions can be simultaneously true (e.g., loader is full AND turret has vision lock). Rather than defining explicit "uber modes" (gathering/navigating/firing) that the driver would need to manage, we use a simple priority list where the highest-priority active condition wins:

| Priority | State | Color | Driver Meaning |
|----------|-------|-------|----------------|
| 1 | FIRING | Pulse white | Hold still, balls are launching |
| 2 | READY_TO_FIRE | Bright white | Vision locked + flywheel at speed, you can fire |
| 3 | AIMING | Cyan | Vision has target, turret converging |
| 4 | FULL | Green (1s flash) | Loader full, stop intaking |
| 5 | HAS_BALLS | Amber | Gathering balls |
| 6 | ALLIANCE | Red/Blue | Default, idle |

**Why this order:** The priority reflects what the driver needs to act on RIGHT NOW. If balls are actively launching (priority 1), the driver needs to hold still regardless of other conditions. If the system is ready to fire (priority 2), that's more actionable than knowing the loader is full. Ball count states (4-5) are useful during gathering but shouldn't override aiming/firing states.

**Future iteration:** As we add pre-spin and auto-navigate-to-fire features, the conditions for READY_TO_FIRE and AIMING will naturally shift, but the priority structure remains the same.

### Launcher IDLE_SPIN

**File:** `Launcher.java`

The `IDLE_SPIN` state existed but was unreachable. When `STAY_SPINNING_AFTER_FIRE=true`, the `handleCompleteState()` method transitioned to `READY` instead of `IDLE_SPIN`. This kept the flywheel at full launch speed unnecessarily.

**Fix:** `COMPLETE` now transitions to `IDLE_SPIN`, which maintains the flywheel at `FLYWHEEL_IDLE_SPEED` (800 DPS) - fast enough for quick spin-up but not wasting power at full launch speed. The `setBehavior(SPINNING)` method was also updated to accept transitions from `IDLE_SPIN` (not just `IDLE`).

### Driver Controls

**File:** `DriverControls.java`

| Button | Before (Competition) | After (Restored) |
|--------|---------------------|-------------------|
| A | `turret.setTracking()` | Toggle turret TRACKING/LOCKED |
| B | `turret.setIdle()` + `setPower(0)` | `turret.setLocked()` |
| D-Up | Manual turret power + | `launcher.changeStar()` (restored) |
| D-Down | Manual turret power - | Slow mode toggle (restored) |

**Why no manual turret controls:** The turret is always under PID control (either TRACKING or LOCKED). Manual power control was only needed during initial hardware bring-up. For calibration, a test mission can be added. The A button toggle provides the driver a way to disable turret seeking (LOCKED = PID holds forward) if something goes wrong during a match.

---

## Phase 4: Wiring

**Files:** `Robot.java`, `Lebot2_6832.java`

- Added `ledStatus.setTurret(turret)` in the Robot constructor so LEDs can check turret lock state.
- Added `robot.turret.setTracking()` in `Lebot2_6832.start()` after `resetStates()`. This activates turret goal-seeking at match start for both autonomous and teleop. The `resetStates()` call sets turret to LOCKED (safe default), then we immediately switch to TRACKING.
