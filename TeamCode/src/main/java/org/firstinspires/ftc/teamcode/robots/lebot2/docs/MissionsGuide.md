# Missions Guide

Missions are high-level autonomous behaviors that coordinate robot subsystems, trajectories, and Robot behaviors to accomplish game-specific goals.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                      AUTONOMOUS                              │
│  (30-second match strategy, sequences missions)              │
├─────────────────────────────────────────────────────────────┤
│                       MISSIONS                               │
│  (game-specific composite behaviors with timeout handling)   │
│  NavigateToFire, LaunchPreloads, BallGroup, OpenSesame      │
├─────────────────────────────────────────────────────────────┤
│                    ROBOT BEHAVIORS                           │
│  (cross-subsystem coordination)                              │
│  TARGETING, LAUNCH_ALL, MANUAL                               │
├─────────────────────────────────────────────────────────────┤
│                 SUBSYSTEM BEHAVIORS                          │
│  (single-subsystem actions)                                  │
│  Intake.loadAll(), Launcher.spinUp(), etc.                  │
└─────────────────────────────────────────────────────────────┘
```

**Key Principle:** Implement functionality at the lowest level where it can be accomplished. Only escalate to a higher level when coordination is required.

**Navigation:** All missions use `TankDriveActions` for position-feedback driving (Turn-Drive-Turn pattern). This replaces RoadRunner spline trajectories with a dual-PID controller that drives straight to the target. Each navigation is bracketed with lazy turn actions for initial alignment and final heading correction. See [ActionsGuide.md](ActionsGuide.md#tankdriveactions-turn-drive-turn) for details and how to use it in new code.

---

## Available Missions

### NAVIGATE_TO_FIRE

Navigate to a fire position while spinning up the launcher.

```java
// Auto-direction: calculates whether forward or reverse requires less turning
robot.missions.startNavigateToFire(1);

// Explicit direction: back up from gate to fire position 1
robot.missions.startNavigateToFire(1, true);

// Explicit direction: drive forward to fire position 2
robot.missions.startNavigateToFire(2, false);
```

**Parameters:**
- `firePosition`: 1-4 (corresponds to FIRE_1 through FIRE_4 in FieldMap)
- `reversed` (optional): true = drive backward, false = drive forward
  - If omitted, automatically chooses the direction requiring less turning

**Timeout:** `NAVIGATION_TIMEOUT_SECONDS` (default 10s)

---

### LAUNCH_PRELOADS

Fire all balls currently in the loader using the Robot.LAUNCH_ALL behavior.

```java
robot.missions.startLaunchPreloads();
```

**Completes when:** Loader is empty or Robot behavior returns to MANUAL
**Timeout:** `LAUNCH_TIMEOUT_SECONDS` (default 8s)

---

### BALL_GROUP

Navigate to a ball row start position, then drive through the row while intaking.

```java
// Collect from ball row 1 (index 0)
robot.missions.startBallGroup(0);

// Collect from ball row 2 (index 1)
robot.missions.startBallGroup(1);
```

**Parameters:**
- `groupIndex`: 0, 1, or 2 (corresponds to ball rows 1, 2, 3)

**Sequence:**
1. Navigate to BALL_ROW_n_START
2. Start intake
3. Drive through row to BALL_ROW_n_END
4. Stop when: reached end, collected BALLS_PER_GROUP, or loader full

**Timeout:** `NAVIGATION_TIMEOUT_SECONDS + INTAKE_TIMEOUT_SECONDS` (default 15s)

---

### OPEN_SESAME

Navigate to the gate and back into the release lever.

```java
robot.missions.startOpenSesame();
```

**Sequence:**
1. Navigate to GATE waypoint
2. Drive backward for PRESS_DURATION_SECONDS
3. Stop

**Timeout:** `NAVIGATION_TIMEOUT_SECONDS + PRESS_TIMEOUT_SECONDS` (default 12s)

---

## Using Missions in Autonomous

### Basic Pattern

```java
public class Autonomous {
    private Robot robot;

    // Track which mission we're waiting for
    private enum Phase {
        INIT,
        BACKUP_TO_FIRE,
        WAITING_BACKUP,
        LAUNCH,
        WAITING_LAUNCH,
        // ... more phases
        COMPLETE
    }
    private Phase phase = Phase.INIT;

    public void execute() {
        // Clear any completed mission state
        robot.missions.clearState();

        switch (phase) {
            case INIT:
                phase = Phase.BACKUP_TO_FIRE;
                break;

            case BACKUP_TO_FIRE:
                robot.missions.startNavigateToFire(1, true);  // Backup to FIRE_1
                phase = Phase.WAITING_BACKUP;
                break;

            case WAITING_BACKUP:
                if (robot.missions.isComplete()) {
                    phase = Phase.LAUNCH;
                } else if (robot.missions.isFailed()) {
                    // Handle failure - skip to next phase or retry
                    phase = Phase.LAUNCH;  // Try to launch anyway
                }
                break;

            case LAUNCH:
                robot.missions.startLaunchPreloads();
                phase = Phase.WAITING_LAUNCH;
                break;

            case WAITING_LAUNCH:
                if (robot.missions.isComplete() || robot.missions.isFailed()) {
                    phase = Phase.COMPLETE;
                }
                break;

            case COMPLETE:
                // Autonomous done
                break;
        }
    }
}
```

### Mission Status Checks

```java
robot.missions.isActive()    // True while mission is RUNNING
robot.missions.isComplete()  // True when mission SUCCEEDED
robot.missions.isFailed()    // True when mission timed out (FAILED)
robot.missions.isAborted()   // True when mission was manually aborted
robot.missions.isDone()      // True if complete OR failed OR aborted
robot.missions.clearState()  // Resets terminal state to IDLE for next mission
```

### Aborting and Restarting Missions

For collision avoidance during TeleOp, drivers may need to abort a mission and restart it:

```java
// Driver initiates navigation to fire position
robot.missions.startNavigateToFire(1);

// ... obstacle detected, driver aborts ...
robot.missions.abort();  // Immediately stops, sets state to ABORTED

// ... obstacle cleared, restart from current position ...
robot.missions.clearState();              // Clear ABORTED state
robot.missions.startNavigateToFire(1);    // Restart - builds new trajectory from current pose
```

**What `abort()` does:**
- Cancels any running trajectory
- Stops all motors
- Stops launcher spinup
- Turns off intake
- Sets robot behavior to MANUAL
- Sets mission state to ABORTED

**Key point:** After abort, calling `startNavigateToFire()` again builds a fresh trajectory from the robot's current position - it doesn't resume the old trajectory.

---

## Timeout Handling

Each mission has configurable timeouts (tunable via FTC Dashboard):

| Constant | Default | Used By |
|----------|---------|---------|
| `NAVIGATION_TIMEOUT_SECONDS` | 10.0 | NavigateToFire, BallGroup, OpenSesame |
| `INTAKE_TIMEOUT_SECONDS` | 5.0 | BallGroup (intake phase) |
| `LAUNCH_TIMEOUT_SECONDS` | 8.0 | LaunchPreloads |
| `PRESS_TIMEOUT_SECONDS` | 2.0 | OpenSesame (pressing phase) |

**When timeout occurs:**
- Mission transitions to `FAILED` state
- Motors are stopped
- Subsystems are reset (intake off, etc.)
- Autonomous can check `isFailed()` and decide how to proceed

---

## Ball Group Order Configuration

The order to approach ball groups can be configured for different strategies:

```java
// In Autonomous.init()
if (startingAtGoalWall) {
    // Start with closest row
    robot.missions.setBallGroupOrder(new int[]{0, 1, 2});
} else {
    // Reverse order for audience wall start
    robot.missions.setBallGroupOrder(new int[]{2, 1, 0});
}
robot.missions.resetGroupProgress();
```

```java
// Get next group to collect
int nextGroup = robot.missions.getNextBallGroup();  // Returns -1 when all done

// After completing a group
robot.missions.advanceToNextGroup();
```

---

## Future: Composable Behaviors

The current implementation requires Autonomous to explicitly manage mission sequencing. A future enhancement could allow missions to be composed:

### Proposed Behavior Interface

```java
public enum BehaviorStatus {
    RUNNING,    // Still executing
    SUCCEEDED,  // Completed successfully
    FAILED      // Could not complete (timeout, error)
}

public interface Behavior {
    void init();              // Called once when behavior starts
    BehaviorStatus tick();    // Called each loop, returns status
    void abort();             // Called if interrupted
}
```

### Sequence Composition

```java
// Define full autonomous as composed behaviors
Behavior fullAuton = new SequenceBehavior(
    new NavigateToFireBehavior(robot, 1, true),   // Backup to fire
    new LaunchBehavior(robot),                     // Launch
    new BallGroupBehavior(robot, 0),               // Row 1
    new NavigateToFireBehavior(robot, 1, false),   // Return
    new LaunchBehavior(robot),                     // Launch
    new BallGroupBehavior(robot, 1),               // Row 2
    new NavigateToFireBehavior(robot, 1, false),   // Return
    new LaunchBehavior(robot),                     // Launch
    new OpenSesameBehavior(robot),                 // Release gate
    new BallGroupBehavior(robot, 2),               // Row 3
    new NavigateToFireBehavior(robot, 1, false),   // Return
    new LaunchBehavior(robot)                      // Final launch
);

// In execute():
BehaviorStatus status = fullAuton.tick();
if (status != BehaviorStatus.RUNNING) {
    // Autonomous complete (succeeded or failed)
}
```

### Additional Composition Patterns

```java
// Parallel execution (all must succeed)
new ParallelBehavior(
    new SpinUpBehavior(robot),
    new NavigateBehavior(robot, target)
)

// Conditional branching
new ConditionalBehavior(
    () -> autonTimer.seconds() < 20,  // If enough time
    new FullSequence(robot),           // Do full sequence
    new AbbreviatedSequence(robot)     // Otherwise abbreviate
)

// Selector (try alternatives until one succeeds)
new SelectorBehavior(
    new VisionTargetBehavior(robot),   // Try vision first
    new OdometryTargetBehavior(robot)  // Fall back to odometry
)
```

This composable approach would:
- Make autonomous sequences more readable
- Enable reuse of behavior patterns
- Support time-based branching naturally
- Allow testing behaviors in isolation

**Note:** This is proposed for future implementation. Current code uses explicit phase management in Autonomous.

---

## Telemetry

Missions provide telemetry showing:
- Current mission and state
- Time elapsed (while running)
- Mission-specific details (in debug mode)

```
Missions
  Mission: BALL_GROUP
  State: RUNNING
  Time: 3.2s
  Target Group: 1
  BallGroup State: INTAKING_THROUGH_ROW
```
