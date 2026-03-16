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

### Phase 1: Turret PID Tuning & "Good Enough" Firing [IMPLEMENTED]

#### 1A. Oscillation-Aware Lock [DONE]

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

#### 1B. PID Retuning [DONE — instrumentation added, Stage 1 tuned by code team]

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

#### 1C. Turret Telemetry Enhancement [DONE]

Add to turret telemetry:
- `Oscillation (deg)` — current amplitude from rolling window
- `Lock Quality` — PERFECT / GOOD_ENOUGH / SEEKING / NO_VISION
- `Aiming Time (ms)` — time since tracking started on current target
- `tx History` — sparkline or min/max over last window

### Phase 2: Flywheel Speed Management [IMPLEMENTED]

#### 2A. Centralize MIN_LAUNCH_SPEED [DONE]

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

#### 2B. Pre-Spin During Navigation [DONE]

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

#### 2C. Teleop Pre-Spin: Nearest Fire Position Prediction [NOT STARTED — follow-on]

Phase 2B only helps autonomous — in teleop there's no `startNavigateToFire()` call. But the driver tends to navigate to the same known-good fire positions defined in FieldMap. When vision is unavailable, we can use the odo-based pose to predict where the driver is headed.

**Trigger:** Intake completes (loader reports full or ball count reaches 3).

**Logic:**
1. Get current odo pose
2. Compute distance from current pose to each fire position (FIRE_1 through FIRE_6)
3. Select the nearest fire position
4. Compute expected flywheel speed from that position's distance to goal
5. Pre-spin flywheel to that speed

```java
// In Robot or DriverControls, when loader transitions to full:
Pose2d currentPose = driveTrain.getPose();
int nearestFire = findNearestFirePosition(currentPose);
double distanceM = computeDistanceToGoal(getFirePose(nearestFire));
double expectedSpeed = Vision.computeFlywheelSpeed(distanceM) * Launcher.SPEED_MULTIPLIER;
launcher.setPreSpinSpeed(expectedSpeed);
```

This gives the driver a head start on spin-up as they navigate to their chosen fire position. If vision becomes available before they fire, `updateTargetSpeed()` overwrites with the live calculation.

**Edge cases:**
- Driver goes to a non-preset position: pre-spin speed is approximate but close, vision corrects on arrival
- Driver changes direction mid-transit: nearest position updates, but we only compute once on intake-full. Could re-evaluate periodically but adds complexity for marginal benefit.
- Odo is drifted: nearest position may be wrong, but the speed difference between adjacent fire positions is small enough that a wrong guess still gets the flywheel close.

### Phase 3: Vision Interruption Handling [IMPLEMENTED]

The current system has a single `LOST_VISION_TIME = 0.2s` debounce, after which it falls back to odometry-based bearing. This is wrong for two reasons:

1. **Vision loss causes vary widely.** A momentary camera glitch is 1-2 frames. A tall robot crossing our line of sight is 0.5-3 seconds. Walking off the edge of camera FOV during a turn is indefinite. The same 0.2s threshold is too short for glitches (causes unnecessary fallback) and too simple for occlusions (doesn't distinguish recoverable from unrecoverable).

2. **Odometry drift makes the fallback worse than doing nothing.** In competition, wheel slip from collisions, defense, and acceleration cause odo position to drift several inches per cycle. When we switch to odo-based bearing, the turret jerks to a heading computed from a stale/wrong field position — often farther from correct than just holding the last vision-based position. Then when vision returns, it jerks back. The result is worse than if we'd just held still.

#### 3A. Three-Tier Vision Loss Response [DONE]

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

#### 3B. Odometry Trust Tracking [DONE]

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

#### 3C. Timed Vision Window for Launch Sequence [DONE — via Tier 3 degraded readiness]

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

#### 3D. Launcher Accepts Degraded Lock [DONE]

```java
// In Launcher.handleSpinningUpState():
if (TRIGGER_TYPE == TriggerType.GATE) {
    if (isFlywheelAtSpeed() &&
        (turret.isReadyToLaunch() || turret.isReadyToLaunchDegraded())) {
        state = LaunchState.READY;
    }
}
```

#### 3E. Wire FALL_BACK_TURN [DEFERRED]

The code team added `FALL_BACK_TURN = 35` but never used it. This is a fixed turret angle for a known fire position when both vision AND odometry are unavailable.

**Proposal:** Use it as a last-resort in Tier 3 when `driveTrain == null || !initialPositionSet`:
```java
turretPID.setSetpoint(clampToLimits(FALL_BACK_TURN));
```

But this is rarely needed — if we have a drivetrain and an initial position, Tier 2 odo bearing is available (even if untrusted, it's better than a fixed angle). FALL_BACK_TURN is really the "everything is broken" fallback.

### Phase 4: Autonomous Fire Position Progression [IMPLEMENTED]

#### 4A. Re-enable FIRE_POSITION Advancement [DONE]

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

#### 4B. Fix FIRE_5/FIRE_6 Coordinates [PENDING — needs physical measurement]

Both are currently `(58.7, 21.5, 88°)` — a copy-paste placeholder. The team needs to measure and set distinct positions for these.

### Phase 5: Driver Feedback & Teleop Integration [NOT STARTED]

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

### Phase 6: Relocalization [IN PROGRESS — code team attempted, needs retooling]

Odometry drift is the root cause of the odo-bearing jerk problem, and it compounds: every time vision corrects the pose, a noisy single-frame reading can inject error rather than removing it. We need relocalization to be more robust.

**Key constraints:**
1. **Stationary only.** The Limelight's botpose is not reliable while the chassis is moving — motion blur, vibration, and detection latency make moving-frame readings noisy. We should only trust vision field position when the chassis is stationary.
2. **Camera is on the turret.** Turret rotation moves the camera even when the chassis is still. Turret motion during sample collection degrades accuracy just like chassis motion.
3. **Audience-side is noisier.** Fire positions near the Limelight's maximum AprilTag range produce systematically less accurate botpose readings. The system must account for this.

**Key terminology:**
- **Relocalization opportunity** — a brief window (target <500ms) where the robot is still enough that vision readings can be trusted. Occurs naturally at fire positions during spin-up, during brief navigation pauses, or when the driver stops to shoot.
- **Sample** — a single vision botpose frame, transformed to robot-center coordinates, with a timestamp.
- **Sample set** — a group of samples collected during one relocalization opportunity. All samples must come from similar conditions.
- **Correction** — writing an averaged, validated sample set to the drivetrain pose. This is the output of a successful relocalization.

#### Code Team Status

The code team (commits `ba402bdb`, `af50b5e4`, `bc59c535`) implemented the right concept: collect multiple stationary frames, average them, reject noisy batches. They built working utility methods (`averagePoses`, `computeSpread`, `transformBotposeToRobotCenter`, `getRobotSpeed`). However the implementation has several bugs that prevent it from ever succeeding:

1. **Single-call pattern**: `collectRelocSample()` is called at discrete state transitions (once per event), not continuously. Each call adds one sample, but `applyFilteredCorrection()` needs 5-10. The sample buffer never fills.
2. **Audience buffer impossible**: Buffer caps at 5 samples (`relocSamples.size() > RELOC_SAMPLES` removes oldest), but audience mode requires 10 (`requiredSamples = RELOC_SAMPLES * 2`). The check can never pass.
3. **Runaway spread multiplier**: `effectiveSpread *= 1.5` runs every call from audience range with no reset. After N calls: 0.15 × 1.5^N → unbounded, eventually accepts garbage.
4. **Missing motion gates**: No check for chassis rotation rate or turret rotation rate. A robot spinning in place at zero translational velocity would pass the velocity check but produce wildly different botpose readings per frame.

The utility methods are sound and will be reused. The collection/application architecture needs a full redesign.

#### 6A. Background Relocalization Collector

The fundamental design change: relocalization runs as a **self-contained background process** every cycle. External code doesn't manage collection — it just checks `isRelocReady()` and calls `applyReloc()`. The collector manages its own state, including flushing when conditions break.

##### Individual Sample Acceptance Rules

ALL must be true for a single frame to enter the sample set:

| # | Gate | Source | Threshold | Rationale |
|---|------|--------|-----------|-----------|
| 1 | Vision available | `vision.hasBotPose()` | boolean | No tag solution = no sample |
| 2 | Chassis translation | `PoseVelocity2d` from `updatePoseEstimate()` | < 2.0 in/s | Motion blur corrupts pose solve |
| 3 | Chassis rotation | `PoseVelocity2d` angular component | < 5.0 deg/s | Heading change smears tag positions in frame |
| 4 | Turret rotation | Δ`turretAngleDeg` between cycles | < 3.0 deg/s | Camera is on turret — turret hunting moves the camera even if chassis is still |
| 5 | Sample freshness | Oldest sample in buffer | < 500ms old | Samples from a previous stop cannot be mixed with samples from this stop |

**Any gate failure flushes the entire sample buffer.** A single frame of motion during collection invalidates the batch because we can't know which samples were affected. This is deliberately conservative — a brief flush and restart costs ~200ms, while a bad correction costs an entire launch cycle.

##### Sample Set Acceptance Rules

A collected set is ready for application when ALL are true:

| # | Check | Method | Threshold |
|---|-------|--------|-----------|
| 1 | Minimum count | `samples.size()` | ≥ `MIN_SAMPLES` (3) |
| 2 | Position spread | Max distance from median position | < `MAX_SPREAD_INCHES` (6.0 in / ~15cm) |
| 3 | Heading spread | Max heading deviation from median heading | < `MAX_HEADING_SPREAD_DEG` (5.0°) |
| 4 | Outlier rejection | Distance from median > 2 × MAD | Remove outliers, re-check count ≥ MIN_SAMPLES |

**No fixed required count.** Unlike the previous design (5 goal-side, 10 audience-side), the collector accepts a set as soon as it has enough agreeing samples. At close range where readings are tight, 3-4 samples may converge in ~100ms. At audience range where readings are noisy, it may take 8-10 samples and ~300ms before spread drops below threshold. The data tells us when it's ready, not a hardcoded number.

**Maximum buffer size** is capped at 15 samples (~500ms at typical Limelight rates). If 15 samples can't converge, the vision conditions are too noisy and we should not attempt correction.

##### Outlier Rejection Within a Set

The previous design (and students' implementation) used max-distance-from-centroid (mean). This is vulnerable to outliers pulling the mean. Better approach:

1. Compute **median X** and **median Y** (robust to up to 50% outliers)
2. Compute **median absolute deviation** (MAD) for each axis
3. Reject any sample where distance from median position > 2 × MAD
4. If > 30% of samples rejected → the set is too noisy, flush everything
5. Average the **surviving** samples (now safe to use mean — outliers are gone)

Heading outliers are handled separately: compute circular median, reject samples with heading deviation > `MAX_HEADING_SPREAD_DEG` from median.

##### Flush Conditions

The sample buffer is flushed (cleared) when any of:
- **Any gate fails** — chassis moved, turned, turret rotated, or vision lost (even one frame)
- **Oldest sample exceeds age limit** — the opportunity window has passed
- **Set rejected** — too much spread or too many outliers after 15 samples collected
- **Correction applied** — start fresh for next opportunity

##### Distance-Aware Thresholds

Rather than a runaway multiplier, compute the effective threshold once per collection attempt based on current distance:

```
if distance > AUDIENCE_RANGE_THRESHOLD_M (2.3):
    effectiveSpreadInches = MAX_SPREAD_INCHES * 1.5    // fixed multiplier, not cumulative
else:
    effectiveSpreadInches = MAX_SPREAD_INCHES
```

At audience range, we accept noisier data because that's all we can get — but the convergence requirement (spread must be below threshold) still ensures internal consistency. The noisier readings just mean more samples are needed before they agree, which happens naturally.

##### Background Collector Pseudocode

```
// Called every cycle from Robot.calc() or equivalent update loop
updateRelocalization():
    // --- Gate checks ---
    if (!vision.hasBotPose()):              flush(); return
    if (chassisTranslationRate > MAX_VEL):  flush(); return
    if (chassisRotationRate > MAX_ROT):     flush(); return
    if (turretRotationRate > MAX_TURRET):   flush(); return

    // --- Collect ---
    sample = transformBotposeToRobotCenter() with timestamp
    add sample to buffer
    drop samples older than WINDOW_MS

    // --- Evaluate ---
    if buffer.size() < MIN_SAMPLES:
        relocReady = false; return

    medianPos = computeMedianPosition(buffer)
    survivors = rejectOutliers(buffer, medianPos, 2 * MAD)

    if survivors.size() < MIN_SAMPLES:
        // Too many outliers — set is noisy
        if buffer.size() >= MAX_SAMPLES: flush()  // give up
        relocReady = false; return

    posSpread = maxDistFromMedian(survivors)
    hdgSpread = maxHeadingDeviation(survivors)
    effectiveSpread = distanceAware(MAX_SPREAD_INCHES)

    if posSpread < effectiveSpread AND hdgSpread < MAX_HEADING_SPREAD_DEG:
        relocReady = true
        averagedPose = averagePoses(survivors)
    else if buffer.size() >= MAX_SAMPLES:
        flush()  // 15 samples and still can't converge
        relocReady = false

// Called by external code (auton state machine, driver button) when ready
applyReloc():
    if !relocReady: return false
    driveTrain.setPose(averagedPose)
    flush()
    return true
```

##### Integration Points

The collector runs **every cycle** — no need for explicit collect calls at state transitions. External code interacts only through:
- `isRelocReady()` — are we confident in the averaged pose?
- `applyReloc()` — write it to drivetrain and reset
- Telemetry: sample count, spread, age, distance, ready status

**Autonomous:** Check `isRelocReady()` at fire position arrival. If ready, apply before launching. If not ready within the spin-up window, launch anyway — the odo pose is still usable for this cycle, and we'll try again next cycle.

**Teleop:** Driver presses relocalization button. If `isRelocReady()`, apply immediately and rumble. If not, brief rumble pattern to indicate "not yet" — the collector is working, try again in a moment.

##### Constants Summary

```
MIN_SAMPLES = 3                  // Minimum to compute meaningful spread
MAX_SAMPLES = 15                 // Give up after this many without convergence
WINDOW_MS = 500                  // Max age of oldest sample in buffer
MAX_TRANSLATION_VEL = 2.0        // in/s — chassis translational velocity gate
MAX_ROTATION_VEL = 5.0           // deg/s — chassis rotational velocity gate
MAX_TURRET_VEL = 3.0             // deg/s — turret angular velocity gate
MAX_SPREAD_INCHES = 6.0          // ~15cm — position spread threshold
MAX_HEADING_SPREAD_DEG = 5.0     // heading spread threshold
AUDIENCE_RANGE_THRESHOLD_M = 2.3 // beyond this, relax spread by 1.5×
OUTLIER_REJECTION_FACTOR = 2.0   // reject samples > 2×MAD from median
MAX_OUTLIER_FRACTION = 0.3       // flush if >30% of samples are outliers
```

#### 6B. Divergence Logging

Log vision vs. odo divergence to CSV for post-match analysis:
- Timestamp, visionX, visionY, odoX, odoY, divergenceInches, chassisVelocity, distanceToGoal, sampleCount, spread, relocApplied
- Only log when the collector has valid samples (same stationarity conditions)
- This data will show how fast odo drifts in real matches, how much audience-side readings vary vs goal-side, and whether the filtered corrections are helping

### Phase 7: Flywheel Recovery & Shot Cadence [IN PROGRESS — logging added, analysis pending]

Each ball that contacts the flywheel steals angular momentum, causing a speed dip. The current system fires all 3 balls with a continuous conveyor feed — the second and third balls hit a progressively slower flywheel, reducing range and accuracy. At audience-side distances the energy budget is tighter and this problem is worse.

The code team has already tuned around this by:
- Slowing the conveyor feed for audience shots (`FEED_AUDIENCE = -0.5` vs `FEED_GOAL = -0.7`)
- Increasing `FIRING_TIME` to 6 seconds (from 2.4) to give more recovery time
- Using `VISION_OFFSET_AUDIENCE = -2°` to steer balls off the rails (higher energy = more rebounds)

But slowing the feed has diminishing returns — too slow and the last ball jams with nothing pushing from behind.

#### 7A. Flywheel Power Analysis [DONE — logging implemented, awaiting data collection]

Before changing the control strategy, we need to understand what's happening. Motor power logging has been added to the existing CSV logger (toggle via `LOGGING_ENABLED` on Dashboard):

**New columns:** `primaryPower`, `helperPower`, `primaryAmps`, `helperAmps`

**Key questions the log should answer:**
1. Are the motors already at full power (1.0) during speed recovery? If so, there's no headroom — we're motor-limited and need to optimize cadence.
2. If power is below 1.0, the PIDF isn't aggressive enough and we're leaving performance on the table.
3. How much does speed drop per ball? If it's consistent, we can predict recovery time.
4. How long does recovery take? This determines the minimum safe pause between balls.
5. Do the primary and helper motors track each other, or does one lag?

#### 7B. Feed-Pause-Feed Strategy [NOT STARTED — depends on 7A data]

If motor analysis shows the flywheel can recover between shots but the continuous feed doesn't give it time:

```
FEED_PAUSE_MS = 300     // Pause between balls to let flywheel recover
FEED_BURST_MS = 500     // Feed duration to advance one ball into the flywheel

Firing sequence:
  0ms:     Gate opens, conveyor runs at FEED_POWER
  500ms:   Conveyor pauses (first ball has fired)
  800ms:   Conveyor runs again (flywheel has recovered ~300ms)
  1300ms:  Conveyor pauses (second ball has fired)
  1600ms:  Conveyor runs again
  2100ms:  Third ball fires
  ~2500ms: Gate closes, COMPLETE
```

Implementation: Replace the continuous `claimResources()` belt run in `handleFiringState()` with a state machine that alternates between feed and pause. The pause duration is calibrated from the power analysis (it needs to be long enough for the flywheel to recover but short enough that the ball doesn't settle and jam).

**For the last ball:** Use a longer feed burst or higher feed power, since there's nothing behind it pushing it into the flywheel. The current `LAUNCH_SPACER_TIMER_LAST` already accounts for this but may need to be re-tuned alongside the pause strategy.

#### 7C. Aggressive Flywheel PIDF / Bang-Bang [NOT STARTED — depends on 7A data]

If the power analysis shows the motors are NOT at full power during recovery, the PIDF isn't aggressive enough. Current coefficients: `P=400, I=0, D=0, F=20`.

**Option A: Increase P dramatically.** With `P=400`, a 100 deg/s speed deficit produces a correction of `400 * (100 / maxVelocity)`. If `maxVelocity` is ~2500 deg/s, that's `400 * 0.04 = 16` — a small correction on top of the F=20 feedforward. Increasing P to 2000+ would make the controller much more aggressive about recovering.

**Option B: Software bang-bang.** Switch the motors to `RUN_WITHOUT_ENCODER` and control power directly:

```java
// In handleFiringState(), replace setVelocity with:
if (currentSpeed < targetSpeed) {
    flywheel.setPower(1.0);       // Full power when below target
    flywheelHelp.setPower(1.0);
} else {
    // At or above target — hold with feedforward estimate
    double holdPower = targetSpeed / MAX_FLYWHEEL_SPEED;  // Open-loop estimate
    flywheel.setPower(holdPower);
    flywheelHelp.setPower(holdPower);
}
```

This eliminates the PIDF latency entirely. At high speeds, the motor's own back-EMF limits overshoot naturally — the motor can't accelerate much past the target before back-EMF balances the applied voltage. This makes bang-bang safe at high RPM.

**Option C: Only use bang-bang during FIRING state.** Keep the SDK PIDF for SPINNING_UP and READY (where smooth approach matters), but switch to bang-bang during FIRING when maximum recovery speed is the priority. This is the most conservative approach — it only changes behavior during the critical window.

**Recommendation:** Start with Option A (increase P) since it requires no code change — just Dashboard-tune P upward during a firing test while watching the CSV log. If that's insufficient, move to Option C. Option B is the nuclear option if nothing else works.

#### 7D. PIDF Target Boost During Firing [DONE]

Rather than switching motor control modes (bang-bang, RUN_WITHOUT_ENCODER), boost the PIDF velocity target during FIRING state by a fixed offset (`FIRING_BOOST_SPEED = 150 deg/s`). This makes the PIDF apply near-maximum power during recovery (it sees a larger error) while still capping speed (no runaway overspeed). Dashboard-tunable via `FIRING_BOOST` (boolean) and `FIRING_BOOST_SPEED` (deg/s).

With P=400, even a +82 deg/s boost saturates the motor controller output (400 × 82 = 32,800 > 32,767 max). So +150 is more than sufficient for full-power recovery, and the max overspeed between ball impacts is bounded at 150 deg/s above target.

`FIRING_TIME` reduced from 6.0s to 1.5s (hard backstop — all balls typically exit within 1s).

#### 7E. Ball Exit Detection [DONE — experimental, off by default]

Speed-drop counter to detect when all balls have exited, ending FIRING early:
- Each cycle compares previous speed to current speed
- Drop > `BALL_EXIT_DROP_THRESHOLD` (80 deg/s) counts as a ball exit
- Cooldown of `BALL_EXIT_COOLDOWN_MS` (200ms) prevents double-counting from recovery oscillation
- FIRING ends when `ballExitCount >= BALL_EXIT_EXPECTED_COUNT` (3)
- Hard timeout (`FIRING_TIME`) still acts as backup

Toggle via `BALL_EXIT_DETECTION` (default false). Ball exit count logged in CSV and telemetry for validation. The threshold of 80 deg/s is based on pre-boost log analysis (drops were ~115-130 observed). With FIRING_BOOST active, the faster recovery may reduce observed drops to ~100 — threshold should still trigger but needs validation with boost-enabled logs.

## Resolved Decisions

1. **Oscillation window**: Time-based (fixed 500ms). Simpler, good enough for our loop rates.
2. **Pre-spin speed**: Compute expected target speed from destination fire position's known distance to goal. Extract speed formula into a static utility.
3. **FALL_BACK_TURN**: Decide later — needs field testing to determine if it's even useful.
4. **Force-fire**: Bypass turret lock only. Flywheel must still be at speed — firing below speed wastes balls.
5. **Continuous odo correction**: Not realistic with our hardware. Vision botpose is only reliable when the chassis is stationary or moving very slowly. Relocalization must be checkpoint-based, not continuous.
6. **Flywheel recovery strategy**: PIDF target boost (Option D — not originally in PRD). Simpler than bang-bang, no mode switching, PIDF still controls speed, max overspeed is bounded and tunable.

## Implementation Priority

### Completed
1. ~~**Phase 3A** (three-tier vision loss response)~~ DONE
2. ~~**Phase 1A + 1B** (oscillation-aware lock + PID tuning)~~ DONE (Stage 1 PID tuned by code team)
3. ~~**Phase 3B + 3C + 3D** (odo trust tracking + timed window + degraded lock)~~ DONE
4. ~~**Phase 2A** (centralize MIN_LAUNCH_SPEED)~~ DONE (code team extended with feed power + vision offset switching)
5. ~~**Phase 2B** (pre-spin to fire position speed)~~ DONE
6. ~~**Phase 4A** (fire position progression)~~ DONE
7. ~~**Phase 7A** (flywheel power logging)~~ DONE
8. ~~**Phase 7D** (PIDF target boost during firing)~~ DONE
9. ~~**Phase 7E** (ball exit detection — experimental)~~ DONE — needs validation with boost-enabled logs

### Remaining (in priority order)
1. **Phase 6A** (background relocalization collector) — Full retool of code team's attempt. Needs turret motion gate, convergence-based readiness, outlier rejection, continuous background collection. Existing utility methods (averagePoses, computeSpread, transformBotposeToRobotCenter, getRobotSpeed) can be reused.
2. **Phase 7B** (feed-pause cadence) — May not be needed if 7D boost + 7E detection work well. Depends on competition testing.
3. **Phase 5** (driver feedback) — LED lock quality, force-fire button, restore updateTargetSpeed on X.
4. **Phase 6B** (divergence logging) — Collect data on odo drift rates and vision accuracy at different distances. Informs whether 6A thresholds need adjustment.
5. **Phase 2C** (teleop pre-spin from nearest fire position) — Follow-on enhancement.
6. **Phase 4B** (FIRE_5/FIRE_6 coordinates) — Needs physical measurement by code team.
7. **Phase 1B Stages 2-3** (PID tuning with flywheel + intake load) — Code team has Stage 1. Stages 2-3 needed for competition-realistic tuning.

## Testing Plan

1. **PID tuning — staged load** (CSV logging throughout):
   - **Stage 1** (turret isolated): Profile step response, characterize inertia, max slew rate, braking distance. Get baseline PID that settles cleanly. *(Code team completed this: P=0.025, I=0, D=0.002)*
   - **Stage 2** (turret + flywheel at launch speed): Re-profile with flywheel vibration and voltage sag. Retune D/P as needed. This is the primary tuning target.
   - **Stage 3** (turret + flywheel + intake): Verify lock survives intake motor cutting in mid-aim. Retune only if lock breaks.
   - Target across all stages: settle to <3° in <0.5s from 30° initial offset.
2. **Occlusion test** (bench): Wave a board in front of the Limelight during turret tracking. Verify Tier 1 HOLD behavior — turret should not move during brief occlusions. Time how long before Tier 2/3 kicks in.
3. **Chassis turn test** (bench): Rotate the chassis while turret is tracking. Verify turret can keep up, and measure the max chassis rotation rate at which the turret falls behind. This determines whether we need to limit chassis turn speed during aiming.
4. **Divergence test** (on field): Drive the robot around, log vision vs odo divergence at stationary checkpoints. Establish baseline drift rate. Compare goal-side vs audience-side accuracy.
5. **Static fire test** (robot parked at known position): Verify full targeting→fire pipeline. Measure time from SPINNING_UP to fire() call. Target: <1.5s with vision, <2.5s without.
6. **Dynamic test** (full auton): Run execute2() and log lock quality, fire timing, divergence. Verify fire position progression and row direction.
7. **Degraded fire test**: Partially block the Limelight during auton. Verify the timed window fires from held position rather than hanging indefinitely.
8. **Relocalization test — gate validation**: Park robot at a known position. Verify samples accumulate when still, flush when chassis nudged, flush when turret manually rotated, flush when vision blocked. Check telemetry: sample count, spread, ready status.
9. **Relocalization test — accuracy**: At goal-side and audience-side fire positions, compare corrected pose vs known position. Run 10 trials each. Measure correction error (should be <2 inches goal-side, <4 inches audience-side). Compare against old single-frame correction.
10. **Relocalization test — convergence speed**: Time from robot stop to `isRelocReady()` at both distances. Target: <300ms goal-side, <500ms audience-side.
11. **Flywheel boost validation**: Fire 3 balls with `FIRING_BOOST=true` and `LOGGING_ENABLED=true`. Verify observed speed drops are still >80 deg/s (ball exit detection threshold). If drops are smaller, adjust `BALL_EXIT_DROP_THRESHOLD` based on data.
9. **Flywheel recovery test**: Fire 3 balls from goal-side and audience-side with `LOGGING_ENABLED = true`. Analyze CSV for: peak power during recovery, speed drop per ball, recovery time between balls, primary vs helper motor symmetry. Determines whether to pursue 7B (cadence) or 7C (control strategy).
10. **Teleop driver test**: Verify LED feedback, force-fire button, manual speed presets.

## Open Questions

1. **HOLD_POSITION_MS tuning**: 300ms is a guess. Too short = unnecessary odo fallback. Too long = turret stays aimed at where the goal *was* after a chassis turn. Need data from the occlusion test to calibrate. May need to be chassis-velocity-aware: if chassis is turning, HOLD should be shorter since the held angle becomes wrong faster.
2. **Turret vs chassis speed**: If the chassis can rotate faster than the turret can track, we may need to either (a) limit chassis turn speed during aiming phases, or (b) accept that the turret will lag and need a catch-up window after turns complete. Need data from the chassis turn test.
3. **Relocalization during auton checkpoints**: Should we add explicit relocalization pauses (e.g., 200ms at fire position to collect samples), or rely on whatever frames arrive during the normal spin-up window? The spin-up window is naturally a stationary period, but it may not always last long enough for 5+ samples.
4. **Audience-side range threshold**: 2.3m is now the distance threshold (changed from 2.6 by code team). Should the relocalization confidence threshold use the same value, or does Limelight accuracy degrade at a different distance than flywheel speed needs to change?
5. **VISION_OFFSET as rebound management**: The -2° audience offset steers balls to reduce bounce-outs. The blue-side offset likely mirrors red but needs testing. Should this be stored per-alliance, or is the geometry symmetric enough that one value works for both?
6. **Feed-pause last ball**: The last ball has nothing pushing from behind. Does it need higher feed power, longer burst, or a different approach entirely (e.g., a brief reverse pulse to re-seat it before the final feed)?
7. **Bang-bang hysteresis**: If we go with Option C (bang-bang during FIRING only), do we need a hysteresis band (e.g., full power below target-5, hold power above target) to prevent chatter at the setpoint? At high RPM the electromechanical lag probably provides natural hysteresis, but this needs verification.
