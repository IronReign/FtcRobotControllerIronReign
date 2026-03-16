# Autonomous Strategy - Goal Wall Start

This document describes the autonomous strategy for starting at the goal wall (near the alliance gate).

---

## Starting Position

- Robot positioned at base of alliance gate, facing the gate (heading ~135° for red in DECODE coordinates)
- This is `FieldMap.START_GOAL` waypoint
- Robot may have preloaded balls

---

## Phase Sequence Overview

```
START_GOAL → FIRE_1 → BALL_ROW_1 → FIRE_1 → BALL_ROW_2 → FIRE_1 → GATE → BALL_ROW_3 → FIRE_1 → END
     ↓           ↓         ↓           ↓         ↓           ↓       ↓        ↓           ↓
  [backup]   [target]  [intake]   [target]  [intake]   [target] [release] [intake]   [target]
  [spinup]   [launch]             [launch]             [launch]           [launch]
```

---

## Detailed Phase Breakdown

### Phase 1: Initial Backup to Fire Position

**Current Location:** START_GOAL (at gate, facing gate)
**Target Location:** FIRE_1

**Actions:**
1. Begin spinning up launcher (while moving)
2. Drive BACKWARD (reversed) to FIRE_1 position
3. AprilTag should come into view during this motion

**Code Pattern:**
```java
// Start launcher spinup
robot.launcher.spinUp();

// Build reversed trajectory
Action backupToFire = driveTrain.actionBuilder(startGoal)
    .setReversed(true)
    .splineTo(fire1.position, fire1.heading)  // or turn-then-lineToX pattern
    .build();
```

**Issues in Current Code:**
- `execute2()` uses `lineToX` then `lineToY` sequentially - won't work for arbitrary positions
- No launcher spinup during backup
- State doesn't advance after starting action (will rebuild every loop)

---

### Phase 2: Target Acquisition

**Current Location:** FIRE_1 (should have AprilTag in view)
**Target:** Center on goal using vision

**Actions:**
1. Check if goal AprilTag is visible
2. If not centered, use vision targeting to align
3. Apply vision pose correction to update odometry

**Code Pattern:**
```java
// Targeting behavior handles alignment
robot.setBehavior(Robot.Behavior.TARGETING);

// Wait for targeting to complete (returns to MANUAL)
// Then apply vision pose correction
robot.applyVisionPoseCorrection();
```

**Notes:**
- Robot should already be roughly facing goal from FIRE_1 heading (135°)
- Targeting makes fine adjustments for accuracy

---

### Phase 3: Launch Sequence

**Current Location:** FIRE_1, aligned with goal
**Action:** Fire all loaded balls

**Code Pattern:**
```java
robot.setBehavior(Robot.Behavior.LAUNCH_ALL);
// or
robot.missions.startLaunchPreloads();
```

---

### Phase 4: Navigate to Ball Row 1

**Current Location:** FIRE_1
**Target Location:** BALL_ROW_1_START → BALL_ROW_1_END

**Actions:**
1. Turn to face ball row (heading 90° = toward blue wall)
2. Navigate to BALL_ROW_1_START
3. Start intake
4. Drive forward through row to BALL_ROW_1_END
5. Stop intake when full or at end

**Code Pattern:**
```java
Pose2d rowStart = FieldMap.getPose(FieldMap.BALL_ROW_1_START, Robot.isRedAlliance);
Pose2d rowEnd = FieldMap.getPose(FieldMap.BALL_ROW_1_END, Robot.isRedAlliance);

Action collectRow1 = driveTrain.actionBuilder(currentPose)
    .turnTo(FieldMap.bearingTo(currentPose, rowStart))
    .lineToY(rowStart.position.y)  // Navigate to row start
    .turnTo(rowStart.heading)       // Face along the row (90°)
    // Now drive through row while intaking
    .lineToY(rowEnd.position.y)
    .build();

// Intake should be running during the final lineToY
robot.intake.loadAll();
```

**Issues in Current Code:**
- `Missions.updateBallGroupMission()` uses `splineToLinearHeading` (doesn't work for tank)
- Uses placeholder poses, not FieldMap waypoints
- Only navigates TO the group, doesn't drive THROUGH the row

---

### Phase 5: Return to Fire Position

**Current Location:** BALL_ROW_1_END
**Target Location:** FIRE_1

**Actions:**
1. Turn toward FIRE_1
2. Navigate back to FIRE_1
3. Begin launcher spinup during return

---

### Phase 6: Target and Launch (Repeat)

Same as Phases 2-3.

---

### Phase 7: Ball Row 2

Same pattern as Phase 4, using BALL_ROW_2_START/END.

---

### Phase 8: Return, Target, Launch (Repeat)

Same as Phases 5-6.

---

### Phase 9: Gate Release (Open Sesame)

**Current Location:** FIRE_1 (after launching row 2 balls)
**Target Location:** GATE

**Actions:**
1. Navigate to GATE position
2. Back into gate lever to release scored balls
3. This releases the opponent's balls for collection

**Timing:** This happens AFTER ball rows 1 & 2, BEFORE ball row 3.

**Code Pattern:**
```java
Pose2d gate = FieldMap.getPose(FieldMap.GATE, Robot.isRedAlliance);

Action toGate = driveTrain.actionBuilder(currentPose)
    .turnTo(FieldMap.bearingTo(currentPose, gate))
    .lineToX(gate.position.x)
    .turnTo(gate.heading)
    .build();

// Then back into lever
robot.driveTrain.drive(-0.3, 0, 0);
// Wait for press duration
robot.driveTrain.drive(0, 0, 0);
```

---

### Phase 10: Ball Row 3 + Released Ball Cluster

**Current Location:** GATE
**Target Location:** BALL_ROW_3_START → BALL_ROW_3_END

**Actions:**
1. Navigate to ball row 3
2. Collect balls (mix of original row 3 + released balls)
3. May need vision-based or statistical approach for released balls

---

### Phase 11: Final Launch

Return to FIRE_1, target, launch remaining balls.

---

## Current Code Analysis

### `execute2()` Method Issues

1. **Waypoint initialization timing:**
   ```java
   // These are evaluated at class load time, not when auton starts
   Pose2d startGoal = FieldMap.getPose(FieldMap.START_GOAL, Robot.isRedAlliance);
   ```
   Should be evaluated in `init()` or at start of `execute2()`.

2. **Incorrect trajectory pattern:**
   ```java
   .lineToX(fire1.position.x)
   .lineToY(fire1.position.y)
   ```
   This attempts to drive to X first, then to Y - requires lateral motion.

   **Fix:** Use turn-then-drive pattern or simple splineTo:
   ```java
   .setReversed(true)
   .splineTo(fire1.position, fire1.heading)
   ```

3. **No state advancement:**
   ```java
   case MOVEBACK_TO_SHOOT:
       // Builds and runs action, but never advances state
       // Will rebuild every loop!
   ```

   **Fix:** Track action completion and advance state.

4. **Missing launcher spinup:** Should start spinning during backup.

5. **Missing subsequent phases:** Only has INIT and MOVEBACK_TO_SHOOT.

### `Missions` Class Issues

1. **Uses splineToLinearHeading:** Won't work for tank drive.

2. **Placeholder poses:** Uses hardcoded poses instead of FieldMap:
   ```java
   public static Pose2d BALL_GROUP_0_POSE = new Pose2d(24, 24, Math.toRadians(0));
   ```
   Should use:
   ```java
   Pose2d start = FieldMap.getPose(FieldMap.BALL_ROW_1_START, Robot.isRedAlliance);
   Pose2d end = FieldMap.getPose(FieldMap.BALL_ROW_1_END, Robot.isRedAlliance);
   ```

3. **Ball collection pattern:** Only navigates TO group, doesn't drive THROUGH row.

---

## Implementation Using Missions

The Missions class provides reusable, timeout-protected behaviors. Use these instead of building trajectories directly in Autonomous.

### Recommended AutonState Enum

```java
public enum AutonState {
    INIT,

    // Phase 1: Backup to fire position
    START_BACKUP_TO_FIRE,
    WAITING_BACKUP,

    // Phase 2-3: Target and launch
    START_TARGETING,
    WAITING_TARGET,
    START_LAUNCH,
    WAITING_LAUNCH,

    // Ball collection cycle (repeated for each row)
    START_BALL_ROW,
    WAITING_BALL_ROW,
    START_RETURN_TO_FIRE,
    WAITING_RETURN,

    // Gate release (after rows 1 & 2, before row 3)
    START_GATE,
    WAITING_GATE,

    // End
    COMPLETE
}
```

### Full Autonomous Implementation

```java
public class Autonomous {
    private Robot robot;
    private AutonState state = AutonState.INIT;
    private int currentRow = 0;  // 0, 1, 2 for rows 1, 2, 3
    private ElapsedTime autonTimer = new ElapsedTime();

    public void init() {
        robot.missions.resetGroupProgress();
        autonTimer.reset();
        state = AutonState.INIT;
        currentRow = 0;
    }

    public void execute() {
        // Clear completed mission state before checking
        robot.missions.clearState();

        // Time remaining for potential branching
        double timeRemaining = 30.0 - autonTimer.seconds();

        switch (state) {
            case INIT:
                state = AutonState.START_BACKUP_TO_FIRE;
                break;

            // ========== PHASE 1: Backup to Fire ==========
            case START_BACKUP_TO_FIRE:
                // Backs up from gate to FIRE_1, spins up launcher
                robot.missions.startNavigateToFire(1, true);
                state = AutonState.WAITING_BACKUP;
                break;

            case WAITING_BACKUP:
                if (robot.missions.isComplete()) {
                    state = AutonState.START_TARGETING;
                } else if (robot.missions.isFailed()) {
                    // Timeout - try to launch anyway
                    state = AutonState.START_LAUNCH;
                }
                break;

            // ========== PHASE 2-3: Target and Launch ==========
            case START_TARGETING:
                robot.setBehavior(Robot.Behavior.TARGETING);
                state = AutonState.WAITING_TARGET;
                break;

            case WAITING_TARGET:
                if (robot.getBehavior() == Robot.Behavior.MANUAL) {
                    // Targeting complete, apply vision correction
                    robot.applyVisionPoseCorrection();
                    state = AutonState.START_LAUNCH;
                }
                break;

            case START_LAUNCH:
                if (!robot.loader.isEmpty()) {
                    robot.missions.startLaunchPreloads();
                    state = AutonState.WAITING_LAUNCH;
                } else {
                    state = AutonState.START_BALL_ROW;
                }
                break;

            case WAITING_LAUNCH:
                if (robot.missions.isComplete() || robot.missions.isFailed()) {
                    robot.applyVisionPoseCorrection();
                    state = AutonState.START_BALL_ROW;
                }
                break;

            // ========== BALL ROW COLLECTION CYCLE ==========
            case START_BALL_ROW:
                if (currentRow >= 3) {
                    state = AutonState.COMPLETE;
                    break;
                }

                // After rows 1 & 2 (currentRow == 2), do gate release before row 3
                if (currentRow == 2) {
                    state = AutonState.START_GATE;
                    break;
                }

                // Check if enough time for another row (rough estimate: 12s per cycle)
                if (timeRemaining < 12) {
                    state = AutonState.COMPLETE;
                    break;
                }

                robot.missions.startBallGroup(currentRow);
                state = AutonState.WAITING_BALL_ROW;
                break;

            case WAITING_BALL_ROW:
                if (robot.missions.isComplete() || robot.missions.isFailed()) {
                    currentRow++;
                    state = AutonState.START_RETURN_TO_FIRE;
                }
                break;

            case START_RETURN_TO_FIRE:
                robot.missions.startNavigateToFire(1);  // Auto-direction to fire
                state = AutonState.WAITING_RETURN;
                break;

            case WAITING_RETURN:
                if (robot.missions.isComplete() || robot.missions.isFailed()) {
                    state = AutonState.START_TARGETING;  // Target and launch again
                }
                break;

            // ========== GATE RELEASE ==========
            case START_GATE:
                robot.missions.startOpenSesame();
                state = AutonState.WAITING_GATE;
                break;

            case WAITING_GATE:
                if (robot.missions.isComplete() || robot.missions.isFailed()) {
                    // Now do row 3
                    robot.missions.startBallGroup(2);
                    state = AutonState.WAITING_BALL_ROW;
                }
                break;

            // ========== COMPLETE ==========
            case COMPLETE:
                // Autonomous done - wait for teleop
                break;
        }
    }
}
```

### Key Points

1. **Use Missions, not raw trajectories** - Missions handle timeouts, FieldMap lookup, and state management.

2. **Clear state each loop** - Call `robot.missions.clearState()` at the start of execute().

3. **Check both complete AND failed** - Missions can timeout. Decide how to handle failures.

4. **Vision correction at fire position** - Call `robot.applyVisionPoseCorrection()` when stationary and facing goal.

5. **Time-based branching** - Check `timeRemaining` before starting long operations.

6. **Gate timing** - Release gate after rows 1 & 2 (`currentRow == 2`), before collecting row 3.

---

## Mission Summary

| Mission | Method | What It Does |
|---------|--------|--------------|
| NAVIGATE_TO_FIRE | `startNavigateToFire(pos)` or `(pos, reversed)` | Navigate to FIRE_n, spin up launcher, auto-direction if not specified |
| LAUNCH_PRELOADS | `startLaunchPreloads()` | Fire all balls via Robot.LAUNCH_ALL |
| BALL_GROUP | `startBallGroup(index)` | Navigate to row start, intake through to end |
| OPEN_SESAME | `startOpenSesame()` | Navigate to gate, back into lever |

All missions:
- Use FieldMap waypoints automatically
- Have configurable timeouts
- Report RUNNING / COMPLETE / FAILED status
- Can be aborted with `robot.missions.abort()`

---

## Issues Fixed in Missions Class

1. ✅ **Tank-compatible trajectories** - Uses `splineTo()` and turn patterns, not `splineToLinearHeading`

2. ✅ **FieldMap integration** - All waypoints come from FieldMap

3. ✅ **Ball row traversal** - Navigates to START, drives through to END

4. ✅ **Timeout handling** - All missions timeout and transition to FAILED

5. ✅ **Launcher spinup** - NAVIGATE_TO_FIRE spins up during navigation
