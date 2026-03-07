# PRD: Targeting & Autofire System Overhaul

## Problem Statement

The turret-based launcher works but the targeting-to-fire pipeline is too slow and brittle. The turret vision PID oscillates instead of settling, the launch readiness check demands a perfect 1-degree lock sustained for 100ms, and multiple real-world factors cause unpredictable lock times. The result: the robot spends too long aiming and sometimes never fires at all within the autonomous timeout window.

We need a targeting system that fires quickly with "good enough" aim, handles vision interruptions gracefully, and gives the driver clear feedback about what's happening.

## Current Architecture

### Targeting Chain (Turret.java → Launcher.java → Missions.java)

1. **Turret.calc()** runs every loop:
   - TRACKING mode has three branches:
     - **Vision active + within limits**: PID on `vision.getTx()` with setpoint `VISION_OFFSET=0`
     - **Vision active + past limits**: PID toward nearest mechanical limit (our fix)
     - **Vision lost > 0.2s**: Pose-based bearing fallback using odometry
   - `readyToLaunch` requires `|tx| < 1°` sustained for `LOCKED_VISION_TIME = 0.1s`

2. **Launcher.handleSpinningUpState()** gates on TWO conditions for GATE trigger:
   - `isFlywheelAtSpeed()` — flywheel within SPEED_TOLERANCE of target
   - `turret.isReadyToLaunch()` — turret reports vision lock
   - Only when BOTH are true does it transition to READY

3. **Robot.handleLaunchAllBehavior2()** orchestrates:
   - Sets launcher to SPINNING → waits for READY → calls fire() → waits for COMPLETE

4. **Missions launch sequence** calls `launcher.updateTargetSpeed()` and manages timeouts

### Current Problems

| Problem | Root Cause | Impact |
|---|---|---|
| PID oscillation | P=0.04 too aggressive for low-friction turret, D=0.0075 too weak to damp | Turret hunts back and forth across target, never settling |
| Lock too strict | LAUNCH_TOLERANCE=1° with 0.1s hold time | Any oscillation >1° resets the timer |
| Vision interruptions | Tall robots occlude target, lighting varies by venue, intermittent detection at max range | Vision drops out for variable durations (0.1s to several seconds) |
| Odo fallback causes jerking | When vision drops, turret jumps to odo-based bearing — but odo drifts fast in competition (wheel slip, collisions, vibration) | Turret snaps to a wrong heading on every vision dropout, then snaps back when vision returns |
| No "good enough" fallback | Binary lock/no-lock — no concept of "close enough, fire anyway" | Robot sits at fire position indefinitely when lock can't be achieved |
| Flywheel speed goes stale | `updateTargetSpeed()` silently no-ops without botpose | If vision drops, flywheel runs at whatever speed was last calculated |
| MIN_LAUNCH_SPEED race | Written from 4+ locations (auton init, applyVisionPoseCorrection, shootShort, shootLong) | Unpredictable speed depending on call order |
| No fire position progression | `FIRE_POSITION++` commented out in execute2() | Robot always returns to same fire position |
| Relocalization fragile | `applyVisionPoseCorrection()` writes odo pose from a single vision frame — noisy readings can inject bad corrections | Odo pose accumulates error from both drift AND bad vision corrections |

## Proposed Changes

### Phase 1: Turret PID Tuning & "Good Enough" Firing

#### 1A. Oscillation-Aware Lock

Replace the current binary `readyToLaunch` with a graduated readiness check that considers oscillation amplitude over a time-based rolling window.

```
New fields:
  LOCK_WINDOW_MS = 500        // Time-based rolling window for oscillation measurement
  PERFECT_LOCK_DEG = 1.0      // Ideal: tx within this = perfect lock
  GOOD_ENOUGH_DEG = 3.0       // Acceptable: oscillation amplitude within this
  LOCK_TIMEOUT_MS = 1000      // Max time to wait for perfect lock before accepting good-enough
  txHistory[]                  // Ring buffer of (timestamp, tx) samples over LOCK_WINDOW_MS
```

**Logic:**
- Track timestamped tx samples over the rolling window; discard samples older than LOCK_WINDOW_MS
- Compute oscillation amplitude = max(txHistory) - min(txHistory)
- `readyToLaunch` conditions (in priority order):
  1. **Perfect lock**: amplitude < PERFECT_LOCK_DEG for 100ms → fire immediately
  2. **Good enough**: amplitude < GOOD_ENOUGH_DEG AND lock timer > LOCK_TIMEOUT_MS → fire
  3. **Timeout**: total aiming time > hard ceiling (e.g. 2s) → fire anyway if amplitude < 2 * GOOD_ENOUGH_DEG
  4. **Abort**: amplitude wildly off or no vision at all for > 1s → fall back to held-position shot (see Phase 3)

This gives the PID time to settle when it can, but doesn't let perfect be the enemy of good.

#### 1B. PID Retuning

The current PID (P=0.04, I=0.00005, D=0.0075) needs field-tuning. Rather than guessing coefficients, add instrumentation:

- Log tx, turretAngleDeg, stagedPower, and phase every loop cycle to CSV (reuse existing CsvLogKeeper pattern from Launcher)
- Dashboard-tune P, D, and INTEGRAL_CUTIN while watching oscillation in real-time
- Target behavior: <0.5s settling time to within GOOD_ENOUGH_DEG from any starting tx within ±30°

**Turret mechanical characteristics** (important for PID tuning):
- The turret carries the entire launcher assembly — high rotational inertia
- Motor is geared down to manage the load — max correction speed is limited
- Bearings are good (low friction) but the mass means momentum carries through setpoint
- The chassis can rotate faster than the turret can compensate — during chassis turns, the turret may fall behind and need to catch up after the turn completes
- This combination (high inertia + low max speed + low friction) means:
  - P should not be too high — the turret can't stop quickly once moving
  - D is critical — it's the main brake against overshoot from momentum
  - I should be very low — steady-state error is small due to low friction
  - MAX_SPEED may need to be limited further to prevent wind-up at large errors

PID tuning should be done empirically on the robot with CSV logging. Do NOT assume starting coefficients — profile the actual step response first, then tune.

**Tuning must happen in stages with increasing system load:**
1. **Isolated**: Turret only, no other motors. Establish baseline step response, measure inertia and braking distance. Get a working PID that settles cleanly.
2. **With flywheel**: Flywheel spinning at launch speed. The flywheel is a major vibration source and draws significant current — both affect turret behavior. Gyroscopic effects from the spinning flywheel may resist turret rotation. Voltage sag under flywheel load reduces available torque for the turret motor. Re-tune D and possibly P to account for the changed dynamics.
3. **With flywheel + intake**: Intake belt running (simulating ball loading during a fire sequence). This is the competition-realistic load profile. The intake usually cuts in after the target is found, so it primarily affects lock *maintenance* rather than initial acquisition. Verify that a lock achieved in stage 2 survives the intake starting.

Each stage may require different PID coefficients. If the difference between stages is significant, consider load-aware PID switching (e.g., slightly higher D when flywheel is active). But try a single set of coefficients tuned at stage 2 first — that's the most common firing condition.

#### 1C. Turret Telemetry Enhancement

Add to turret telemetry:
- `Oscillation (deg)` — current amplitude from rolling window
- `Lock Quality` — PERFECT / GOOD_ENOUGH / SEEKING / NO_VISION
- `Aiming Time (ms)` — time since tracking started on current target
- `tx History` — sparkline or min/max over last window

### Phase 2: Flywheel Speed Management

#### 2A. Centralize MIN_LAUNCH_SPEED

Currently mutated from Robot.applyVisionPoseCorrection(), Launcher.shootShort(), Launcher.shootLong(), and Autonomous.init(). Replace with a single source of truth.

**Proposal:** Move distance-based speed selection into `updateTargetSpeed()` itself:

```java
public void updateTargetSpeed() {
    if (vision != null && vision.hasBotPose()) {
        // Vision-based speed
        targetSpeed = vision.getFlywheelSpeed() * SPEED_MULTIPLIER;
    } else if (distanceHint == DistanceHint.NEAR) {
        targetSpeed = MIN_LAUNCH_SPEED_GOAL * SPEED_MULTIPLIER;
    } else {
        targetSpeed = MIN_LAUNCH_SPEED_AUDIENCE * SPEED_MULTIPLIER;
    }
}
```

Where `distanceHint` is set once based on starting position (AUDIENCE = FAR, GOAL = NEAR) and updated whenever applyVisionPoseCorrection() runs. This eliminates the multi-writer race.

#### 2B. Pre-Spin During Navigation

Currently the flywheel is IDLE during ball collection and navigation. By the time we reach the fire position and spin up, we've wasted 1-2 seconds.

**Proposal:** When `startNavigateToFire()` is called, compute the expected target speed from the destination fire position's known distance to the goal, and start spinning the flywheel to that speed during navigation.

```java
public void startNavigateToFire(int firePosition) {
    // Compute expected flywheel speed from destination fire position
    Pose2d firePose = getFirePose(firePosition);
    Pose2d goalPose = FieldMap.getPose(FieldMap.GOAL, Robot.isRedAlliance);
    double distanceM = poseDistanceMeters(firePose, goalPose);
    double expectedSpeed = Vision.computeFlywheelSpeed(distanceM);

    // Pre-spin to expected speed (not IDLE_SPIN — the actual target speed)
    robot.launcher.setPreSpinSpeed(expectedSpeed);
    robot.launcher.setBehavior(Launcher.Behavior.SPINNING);

    // ... existing navigation code ...
}
```

This means the flywheel is at or near the correct launch speed by the time we arrive, saving the entire spin-up window. The speed formula (`126.58*d + 873.7`) needs to be extracted from Vision into a static utility so Missions can call it without a vision instance.

Note: `setPreSpinSpeed()` is a new method that sets `targetSpeed` directly. Once at the fire position, `updateTargetSpeed()` will overwrite it with the live vision-based calculation if vision is available, which is fine — the pre-spin speed is just a head start.

### Phase 3: Vision Interruption Handling

The current system has a single `LOST_VISION_TIME = 0.2s` debounce, after which it falls back to odometry-based bearing. This is wrong for two reasons:

1. **Vision loss causes vary widely.** A momentary camera glitch is 1-2 frames. A tall robot crossing our line of sight is 0.5-3 seconds. Walking off the edge of camera FOV during a turn is indefinite. The same 0.2s threshold is too short for glitches (causes unnecessary fallback) and too simple for occlusions (doesn't distinguish recoverable from unrecoverable).

2. **Odometry drift makes the fallback worse than doing nothing.** In competition, wheel slip from collisions, defense, and acceleration cause odo position to drift several inches per cycle. When we switch to odo-based bearing, the turret jerks to a heading computed from a stale/wrong field position — often farther from correct than just holding the last vision-based position. Then when vision returns, it jerks back. The result is worse than if we'd just held still.

#### 3A. Three-Tier Vision Loss Response

Replace the single debounce with three tiers based on how long vision has been lost:

```
HOLD_POSITION_MS = 300     // Tier 1: hold turret position (covers glitches + short occlusions)
ODO_FALLBACK_MS  = 2000    // Tier 2: use odo bearing (only if odo is trusted, see 3B)
FIRE_ANYWAY_MS   = 3000    // Tier 3: fire from held position (better than not firing)
```

**Tier 1 — HOLD (0 to HOLD_POSITION_MS):**
Turret holds its current robot-frame angle. `stagedPower` drives PID to `turretAngleDeg` (i.e., setpoint = current angle). The turret doesn't move. This covers:
- Momentary detection dropouts (1-2 frames)
- Short robot occlusions (0.1-0.3s)
- The turret was already aimed approximately correctly, so holding position is a good bet

This is what the dead Branch 3 in the current code *almost* does (it holds last power implicitly). We're making it explicit and deliberate.

**Tier 2 — ODO BEARING (HOLD_POSITION_MS to ODO_FALLBACK_MS):**
If vision has been gone long enough that holding position may not be valid (robot may have turned), attempt odo-based bearing — BUT only if odo is recently calibrated (see 3B divergence check). If odo is stale/untrusted, stay in HOLD.

**Tier 3 — FIRE ANYWAY (after FIRE_ANYWAY_MS):**
If we're in the middle of a launch sequence and vision has been gone for 3 seconds, we need to fire and move on. Set `readyToLaunchDegraded = true` which the launcher accepts as an alternative to `readyToLaunch`. Log this event prominently for post-match analysis.

```java
// In Turret.calc(), TRACKING mode, vision lost branch:
double lostDuration = lostVisionTimer.seconds();

if (lostDuration < HOLD_POSITION_MS / 1000.0) {
    // Tier 1: Hold current position
    phase = TargetingPhase.HOLDING;
    turretPID.setSetpoint(turretAngleDeg);  // PID holds current angle
    turretPID.setInput(turretAngleDeg);
    stagedPower = turretPID.performPID();    // ~0 power, holds position

} else if (lostDuration < ODO_FALLBACK_MS / 1000.0 && odoTrusted) {
    // Tier 2: Odo bearing (only if odo is recently calibrated)
    phase = TargetingPhase.POSE_SEEKING;
    // ... existing pose-based bearing code ...

} else {
    // Tier 3: Hold position, signal degraded readiness
    phase = TargetingPhase.HOLDING;
    turretPID.setSetpoint(turretAngleDeg);
    turretPID.setInput(turretAngleDeg);
    stagedPower = turretPID.performPID();
    readyToLaunchDegraded = true;
}
```

Add `HOLDING` to `TargetingPhase` enum.

#### 3B. Odometry Trust Tracking

Odometry drifts fast in competition. We need to know whether the odo-based bearing is worth using.

**Approach: Vision/Odo Divergence Check**

Every time we have BOTH a valid vision target AND a valid odo pose, compare the two bearing solutions:

```java
// In Turret.calc(), when vision is active:
double visionBearing = turretAngleDeg - vision.getTx();  // Where vision says the goal is
double odoBearing = computeOdoBearing();                   // Where odo says the goal is
double divergenceDeg = Math.abs(normalizeDeg(visionBearing - odoBearing));

// Track divergence over time
lastDivergence = divergenceDeg;
lastDivergenceTime = System.currentTimeMillis();
odoTrusted = (divergenceDeg < ODO_TRUST_THRESHOLD_DEG);   // e.g., 10°
```

```
ODO_TRUST_THRESHOLD_DEG = 10   // If vision and odo disagree by more than this, don't trust odo
ODO_TRUST_STALE_MS = 5000      // If we haven't checked divergence in this long, mark odo untrusted
```

When odo is untrusted, the Tier 2 fallback is skipped entirely — we stay in HOLD until vision returns or Tier 3 fires.

**Telemetry:** Add `Odo Trust` (YES/NO), `Divergence (deg)`, `Last Check (ms ago)` to turret telemetry. This is critical diagnostic data for understanding competition behavior.

#### 3C. Timed Vision Window for Launch Sequence

Give the targeting system a finite budget after arriving at the fire position:

```
VISION_WINDOW_MS = 1500   // Max time to attempt vision lock

Timeline:
  0ms:     Arrive at fire position, start flywheel spin-up, turret in TRACKING
  0-1500ms: Vision PID active, measuring oscillation (Phase 1A)
  1500ms:  If no PERFECT or GOOD_ENOUGH lock achieved:
           → Accept HOLDING position or odo bearing as "good enough"
           → Set readyToLaunchDegraded = true
           → Fire when flywheel at speed
```

This ensures we always fire within a bounded time. The turret's current position at the fire position should be approximately correct even without vision lock, because:
- We navigated to a known fire position (odo got us close)
- The turret was tracking during approach (vision or odo had us aimed)
- Holding position preserves whatever aim we had

#### 3D. Launcher Accepts Degraded Lock

```java
// In Launcher.handleSpinningUpState():
if (TRIGGER_TYPE == TriggerType.GATE) {
    if (isFlywheelAtSpeed() &&
        (turret.isReadyToLaunch() || turret.isReadyToLaunchDegraded())) {
        state = LaunchState.READY;
    }
}
```

#### 3E. Wire FALL_BACK_TURN

The code team added `FALL_BACK_TURN = 35` but never used it. This is a fixed turret angle for a known fire position when both vision AND odometry are unavailable.

**Proposal:** Use it as a last-resort in Tier 3 when `driveTrain == null || !initialPositionSet`:
```java
turretPID.setSetpoint(clampToLimits(FALL_BACK_TURN));
```

But this is rarely needed — if we have a drivetrain and an initial position, Tier 2 odo bearing is available (even if untrusted, it's better than a fixed angle). FALL_BACK_TURN is really the "everything is broken" fallback.

### Phase 4: Autonomous Fire Position Progression

#### 4A. Re-enable FIRE_POSITION Advancement

`FIRE_POSITION++` is commented out in execute2(). Without this, the robot always fires from the same position, making it predictable and potentially less effective if the first position has an obstructed view.

**Proposal:** Re-enable with bounds checking:
```java
// In WAITING_LAUNCH success/failure:
FIRE_POSITION++;
if (FIRE_POSITION > MAX_FIRE_POSITION) {
    FIRE_POSITION = MAX_FIRE_POSITION; // Clamp rather than wrap
}
```

Where `MAX_FIRE_POSITION` is 3 for goal-side (FIRE_1, FIRE_2, FIRE_3) and 6 for audience-side (FIRE_4, FIRE_5, FIRE_6). Note: FIRE_5 and FIRE_6 currently have identical coordinates — these need to be differentiated in FieldMap.

#### 4B. Fix FIRE_5/FIRE_6 Coordinates

Both are currently `(58.7, 21.5, 88°)` — a copy-paste placeholder. The team needs to measure and set distinct positions for these.

### Phase 5: Driver Feedback & Teleop Integration

#### 5A. LED Integration with Lock Quality

Update LEDStatus to show targeting state using the new graduated lock quality:

| Lock Quality | LED Pattern |
|---|---|
| NO_VISION | Slow red pulse |
| SEEKING (oscillation > GOOD_ENOUGH) | Fast yellow blink |
| GOOD_ENOUGH (ready to fire) | Solid yellow |
| PERFECT (ideal lock) | Solid green |
| FIRING | White strobe |

#### 5B. Driver Override: Force Fire

Add a driver button (suggestion: left bumper hold for 0.5s) that bypasses the turret lock check and fires immediately if the flywheel is at speed. This is the "just send it" button for when the driver can see the alignment is close enough but the system disagrees.

```java
// In Launcher, add:
public void forceFire() {
    if (isFlywheelAtSpeed() && state == LaunchState.SPINNING_UP) {
        state = LaunchState.READY;
        fireRequested = true;
    }
}
```

#### 5C. Restore updateTargetSpeed on Spin-Up

`updateTargetSpeed()` is commented out on the X button press (DriverControls line 145). When the driver manually spins up, the flywheel should get the correct speed. Re-enable this.

### Phase 6: Relocalization

Odometry drift is the root cause of the odo-bearing jerk problem, and it compounds: every time vision corrects the pose, a noisy single-frame reading can inject error rather than removing it. We need relocalization to be more robust.

**Key constraint:** Continuous odo correction (blending vision into odo every frame) is not realistic with our hardware. The Limelight's botpose is not reliable enough while the chassis is moving — motion blur, vibration, and variable detection latency make moving-frame readings noisy. We should only trust vision field position when the chassis is stationary or moving very slowly.

**Additional constraint:** Audience-side fire positions are near the Limelight's maximum AprilTag detection range. Botpose readings from these positions may be systematically less accurate than goal-side readings. The system should account for this.

#### 6A. Stationary Filtered Vision Pose Correction

Currently `applyVisionPoseCorrection()` takes a single botpose frame and writes it directly to the drivetrain. A single noisy frame can shift the robot's believed position by inches.

**Proposal:** Only collect relocalization samples when the chassis is stationary (or nearly so), and average multiple frames before applying.

```java
// New fields in Robot:
private static final int RELOC_SAMPLES = 5;       // Frames to average
private static final double RELOC_MAX_SPREAD_M = 0.15;  // Reject if samples spread > 15cm
private static final double RELOC_MAX_VELOCITY = 2.0;   // inches/sec — below this is "stationary"
private List<Pose2d> relocSamples = new ArrayList<>();

public void collectRelocSample() {
    // Only collect when chassis is nearly stationary
    if (!vision.hasBotPose()) return;
    if (driveTrain.getVelocity() > RELOC_MAX_VELOCITY) {
        relocSamples.clear();  // Motion invalidates collected samples
        return;
    }

    Pose2d sample = transformBotposeToRobotCenter();
    relocSamples.add(sample);
}

public boolean applyFilteredCorrection() {
    if (relocSamples.size() < RELOC_SAMPLES) return false;

    // Check spread — if samples disagree too much, vision is unreliable
    double spreadM = computeSpread(relocSamples);
    if (spreadM > RELOC_MAX_SPREAD_M) {
        relocSamples.clear();
        return false;  // Reject noisy batch
    }

    // Apply average
    Pose2d averaged = averagePoses(relocSamples);
    driveTrain.setPose(averaged);
    relocSamples.clear();
    return true;
}
```

Call `collectRelocSample()` every loop when at a fire position (the robot is naturally stationary during spin-up and aiming). Call `applyFilteredCorrection()` before launching. This gives 5 frames of averaging (~100-200ms at typical Limelight rates) and rejects batches where the Limelight is giving inconsistent readings.

The stationary requirement means:
- Samples are only collected when the robot has stopped at the fire position
- Any chassis movement (joystick input, trajectory correction) clears the sample buffer
- The flywheel spin-up window naturally provides a stationary collection period

#### 6B. Distance-Aware Confidence

Audience-side fire positions are near the Limelight's maximum detection range. Botpose accuracy degrades with distance — the same pixel error in the image translates to a larger position error at range.

**Proposal:** Scale the relocalization acceptance threshold by distance:

```java
double effectiveSpread = RELOC_MAX_SPREAD_M;
if (vision.getDistanceToGoal() > AUDIENCE_RANGE_THRESHOLD_M) {
    // Relax spread threshold at long range — readings are noisier
    effectiveSpread *= 1.5;
    // But also require more samples for averaging
    requiredSamples = RELOC_SAMPLES * 2;  // 10 instead of 5
}
```

```
AUDIENCE_RANGE_THRESHOLD_M = 2.6  // Beyond this, vision is less reliable
```

This means we accept noisier readings from far away (because that's all we can get), but average more aggressively to compensate.

#### 6C. Divergence Logging

Log vision vs. odo divergence to CSV for post-match analysis:
- Timestamp, visionX, visionY, odoX, odoY, divergenceM, divergenceDeg, chassisVelocity, distanceToGoal
- Only log when chassis is stationary (same condition as sample collection)
- This data will show how fast odo drifts in real matches, how much audience-side readings vary vs goal-side, and whether the filtered corrections are helping

## Resolved Decisions

1. **Oscillation window**: Time-based (fixed 500ms). Simpler, good enough for our loop rates.
2. **Pre-spin speed**: Compute expected target speed from destination fire position's known distance to goal. Extract speed formula into a static utility.
3. **FALL_BACK_TURN**: Decide later — needs field testing to determine if it's even useful.
4. **Force-fire**: Bypass turret lock only. Flywheel must still be at speed — firing below speed wastes balls.
5. **Continuous odo correction**: Not realistic with our hardware. Vision botpose is only reliable when the chassis is stationary or moving very slowly. Relocalization must be checkpoint-based, not continuous.

## Implementation Priority

1. **Phase 3A** (three-tier vision loss response) — Most critical. The turret jerking to odo bearing on every vision dropout is actively harmful. Switching to HOLD as the default short-term response is the single most impactful change.
2. **Phase 1A + 1B** (oscillation-aware lock + PID tuning) — Second biggest impact. Directly addresses "takes too long to fire." PID tuning must be done empirically — profile the step response first.
3. **Phase 3B + 3C + 3D** (odo trust tracking + timed window + degraded lock) — Completes the vision fallback story. Ensures we always fire within a bounded time.
4. **Phase 2A** (centralize MIN_LAUNCH_SPEED) — Eliminates an entire class of bugs.
5. **Phase 6A** (stationary filtered relocalization) — Reduces odo drift at checkpoints, which makes Tier 2 odo fallback more useful when needed.
6. **Phase 2B** (pre-spin to fire position speed) — Free time savings, moderate implementation effort (need static speed utility).
7. **Phase 4A** (fire position progression) — Quick re-enable once positions are verified.
8. **Phase 5** (driver feedback) — Important for teleop but not blocking autonomous.
9. **Phase 6B** (distance-aware confidence) — Refinement. Only matters once 6A is working and we have data on audience-side accuracy.

## Testing Plan

1. **PID tuning — staged load** (CSV logging throughout):
   - **Stage 1** (turret isolated): Profile step response, characterize inertia, max slew rate, braking distance. Get baseline PID that settles cleanly.
   - **Stage 2** (turret + flywheel at launch speed): Re-profile with flywheel vibration and voltage sag. Retune D/P as needed. This is the primary tuning target.
   - **Stage 3** (turret + flywheel + intake): Verify lock survives intake motor cutting in mid-aim. Retune only if lock breaks.
   - Target across all stages: settle to <3° in <0.5s from 30° initial offset.
2. **Occlusion test** (bench): Wave a board in front of the Limelight during turret tracking. Verify Tier 1 HOLD behavior — turret should not move during brief occlusions. Time how long before Tier 2/3 kicks in.
3. **Chassis turn test** (bench): Rotate the chassis while turret is tracking. Verify turret can keep up, and measure the max chassis rotation rate at which the turret falls behind. This determines whether we need to limit chassis turn speed during aiming.
4. **Divergence test** (on field): Drive the robot around, log vision vs odo divergence at stationary checkpoints. Establish baseline drift rate. Compare goal-side vs audience-side accuracy.
5. **Static fire test** (robot parked at known position): Verify full targeting→fire pipeline. Measure time from SPINNING_UP to fire() call. Target: <1.5s with vision, <2.5s without.
6. **Dynamic test** (full auton): Run execute2() and log lock quality, fire timing, divergence. Verify fire position progression and row direction.
7. **Degraded fire test**: Partially block the Limelight during auton. Verify the timed window fires from held position rather than hanging indefinitely.
8. **Relocalization test**: Compare single-frame vs stationary-filtered correction accuracy at both goal-side and audience-side positions.
9. **Teleop driver test**: Verify LED feedback, force-fire button, manual speed presets.

## Open Questions

1. **HOLD_POSITION_MS tuning**: 300ms is a guess. Too short = unnecessary odo fallback. Too long = turret stays aimed at where the goal *was* after a chassis turn. Need data from the occlusion test to calibrate. May need to be chassis-velocity-aware: if chassis is turning, HOLD should be shorter since the held angle becomes wrong faster.
2. **Turret vs chassis speed**: If the chassis can rotate faster than the turret can track, we may need to either (a) limit chassis turn speed during aiming phases, or (b) accept that the turret will lag and need a catch-up window after turns complete. Need data from the chassis turn test.
3. **Relocalization during auton checkpoints**: Should we add explicit relocalization pauses (e.g., 200ms at fire position to collect samples), or rely on whatever frames arrive during the normal spin-up window? The spin-up window is naturally a stationary period, but it may not always last long enough for 5+ samples.
4. **Audience-side range threshold**: 2.6m is the current magic number for distance-based switching throughout the codebase. Should the relocalization confidence threshold use the same value, or does Limelight accuracy degrade at a different distance than flywheel speed needs to change?
