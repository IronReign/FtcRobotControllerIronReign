# RoadRunner Actions Guide

This guide covers how to use RoadRunner 1.0 actions in competition code, where you need full control of your robot while trajectories execute.

## Table of Contents

1. [Blocking vs Non-Blocking](#blocking-vs-non-blocking)
2. [Non-Blocking Pattern](#non-blocking-pattern)
3. [Aborting Actions](#aborting-actions)
4. [Cleanup After Abort](#cleanup-after-abort)
5. [Chaining Actions](#chaining-actions)
6. [Barging Actions](#barging-actions)
7. [Define vs Evaluate](#define-vs-evaluate)
8. [Resuming Aborted Actions](#resuming-aborted-actions)
9. [Reverse Trajectories](#reverse-trajectories)
10. [TankDriveActions (Turn-Spline-Turn)](#tankdriveactions-turn-spline-turn)

---

## Blocking vs Non-Blocking

### The Problem with runBlocking()

`Actions.runBlocking()` is a convenience method that monopolizes your thread:

```java
// DON'T use this in competition code
Actions.runBlocking(drive.actionBuilder(pose)
    .splineTo(target, heading)
    .build());
// Nothing else runs until trajectory completes!
```

During `runBlocking()`:
- No sensor updates
- No subsystem control (launcher, intake, etc.)
- No response to driver input
- No collision avoidance
- Robot is "blind" except for odometry

### When runBlocking() Is Acceptable

- Tuning opmodes (LocalizationTest, SplineTest, etc.)
- Simple autonomous routines with no parallel subsystem activity
- Testing trajectory shapes

---

## Non-Blocking Pattern

For competition, manually tick actions inside your main loop:

```java
public class Robot {
    private Action currentAction = null;

    public void update() {
        // 1. Refresh all sensors
        pinpointLocalizer.refresh();

        // 2. Update all subsystems
        launcher.update();
        intake.update();

        // 3. Tick the current action (ONE iteration only)
        if (currentAction != null) {
            TelemetryPacket packet = new TelemetryPacket();
            boolean stillRunning = currentAction.run(packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            if (!stillRunning) {
                currentAction = null;  // Action completed
                onActionComplete();
            }
        }

        // 4. Mark sensor cycle complete
        pinpointLocalizer.markCycleComplete();
    }

    public void startAction(Action action) {
        currentAction = action;
    }

    public boolean isActionRunning() {
        return currentAction != null;
    }
}
```

This gives you:
- Full sensor data every cycle
- Subsystem updates every cycle
- Ability to interrupt at any time
- Responsive to driver input

---

## Aborting Actions

### Basic Abort

```java
public void abortCurrentAction() {
    if (currentAction != null) {
        currentAction = null;
        onActionAborted();
    }
}
```

### Abort on Driver Input

```java
// In your controls handling:
if (gamepad1.b) {  // B button = emergency stop
    robot.abortCurrentAction();
    robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
}

// Or abort when driver takes manual control:
boolean driverWantsControl = Math.abs(gamepad1.left_stick_y) > 0.1
                          || Math.abs(gamepad1.right_stick_x) > 0.1;
if (driverWantsControl && robot.isActionRunning()) {
    robot.abortCurrentAction();
}
```

### Abort with Condition Check

```java
public void update() {
    // Check abort conditions BEFORE ticking action
    if (currentAction != null && shouldAbort()) {
        abortCurrentAction();
        return;
    }

    // ... rest of update
}

private boolean shouldAbort() {
    // Examples:
    return driverRequestedAbort
        || distanceSensor.getDistance(DistanceUnit.CM) < 10  // Collision imminent
        || matchTimer.seconds() > 28;  // End of auto period
}
```

---

## Cleanup After Abort

When you abort mid-trajectory, the robot may be moving. You need cleanup:

### Stop the Drivetrain

```java
public void abortCurrentAction() {
    if (currentAction != null) {
        // CRITICAL: Stop the motors
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

        // Or for tank drive:
        drive.leftMotor.setPower(0);
        drive.rightMotor.setPower(0);

        currentAction = null;
        onActionAborted();
    }
}
```

### Reset Subsystem States

If your action was coordinating with subsystems:

```java
public void abortCurrentAction() {
    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

    // Reset any subsystems that were mid-operation
    launcher.setBehavior(Launcher.Behavior.IDLE);
    intake.stop();

    currentAction = null;
    abortedPose = drive.pose;  // Save where we stopped
}
```

### Capture State for Resume

```java
private Pose2d abortedPose = null;
private Pose2d originalTargetPose = null;

public void abortCurrentAction() {
    if (currentAction != null) {
        abortedPose = drive.pose;  // Where we actually stopped
        // originalTargetPose was saved when action started

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        currentAction = null;
    }
}
```

---

## Chaining Actions

### Sequential Actions with SequentialAction

```java
Action sequence = new SequentialAction(
    drive.actionBuilder(startPose)
        .splineTo(waypoint1, heading1)
        .build(),
    drive.actionBuilder(waypoint1Pose)
        .splineTo(waypoint2, heading2)
        .build(),
    drive.actionBuilder(waypoint2Pose)
        .splineTo(finalTarget, finalHeading)
        .build()
);

robot.startAction(sequence);
```

### Parallel Actions with ParallelAction

Run trajectory while operating subsystems:

```java
Action driveAndPrepare = new ParallelAction(
    // Drive to launch position
    drive.actionBuilder(currentPose)
        .splineTo(launchPosition, launchHeading)
        .build(),
    // Spin up flywheel while driving
    new Action() {
        @Override
        public boolean run(TelemetryPacket packet) {
            launcher.setBehavior(Launcher.Behavior.SPINNING_UP);
            return false;  // Completes immediately (fire-and-forget)
        }
    }
);
```

### Custom Action for Subsystem Waits

```java
// Action that waits for launcher to be ready
Action waitForLauncher = new Action() {
    @Override
    public boolean run(TelemetryPacket packet) {
        packet.put("launcher_state", launcher.getBehavior().name());
        return launcher.getBehavior() != Launcher.Behavior.READY;  // Keep running until ready
    }
};

// Full sequence: drive, wait for ready, fire
Action launchSequence = new SequentialAction(
    driveToLaunchPosition,
    waitForLauncher,
    fireAction
);
```

### Chaining with Behavior State Machine

For complex coordination, use your behavior system:

```java
// In Robot.java behavior handling:
case APPROACH_AND_LAUNCH:
    switch (subState) {
        case 0:  // Start driving
            startAction(buildDriveToLaunchAction());
            launcher.setBehavior(Launcher.Behavior.SPINNING_UP);
            subState = 1;
            break;
        case 1:  // Wait for both
            if (!isActionRunning() && launcher.isReady()) {
                launcher.fire();
                subState = 2;
            }
            break;
        case 2:  // Wait for launch complete
            if (launcher.getBehavior() == Launcher.Behavior.COMPLETE) {
                setBehavior(Behavior.MANUAL);
            }
            break;
    }
    break;
```

---

## Barging Actions

"Barging" = interrupting one action by starting another.

### Simple Barge

```java
public void startAction(Action newAction) {
    if (currentAction != null) {
        // Implicitly abort the old action
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }
    currentAction = newAction;
}
```

### Barge with Transition

Sometimes you want a smooth transition:

```java
public void bargeWithNewTarget(Pose2d newTarget) {
    // Stop current action
    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

    // Build new trajectory FROM CURRENT POSE
    Pose2d currentPose = drive.pose;
    Action newAction = drive.actionBuilder(currentPose)
        .splineTo(newTarget.position, newTarget.heading.toDouble())
        .build();

    currentAction = newAction;
}
```

### Priority-Based Barging

```java
public enum ActionPriority { LOW, NORMAL, HIGH, CRITICAL }

private ActionPriority currentPriority = ActionPriority.LOW;

public boolean startAction(Action action, ActionPriority priority) {
    if (priority.ordinal() >= currentPriority.ordinal()) {
        if (currentAction != null) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        }
        currentAction = action;
        currentPriority = priority;
        return true;
    }
    return false;  // Rejected - current action has higher priority
}

// Usage:
robot.startAction(routineTrajectory, ActionPriority.NORMAL);
// Later, if collision detected:
robot.startAction(evasionTrajectory, ActionPriority.CRITICAL);  // This will barge
```

---

## Define vs Evaluate

### When Actions Are Built

Actions are **built** (trajectory calculated) when you call `.build()`:

```java
// Trajectory is calculated HERE, using currentPose at this moment
Action action = drive.actionBuilder(currentPose)
    .splineTo(target, heading)
    .build();  // <-- Math happens now

// Later execution uses the pre-calculated path
robot.startAction(action);
```

### Problem: Stale Start Pose

```java
// BAD: Building action before you need it
Action goToLaunch = drive.actionBuilder(startPose)  // Uses pose from 5 seconds ago!
    .splineTo(launchPos, launchHeading)
    .build();

// ... robot moves around ...

robot.startAction(goToLaunch);  // Trajectory starts from wrong place!
```

### Solution: Build Just-In-Time

```java
// GOOD: Build when you need it
public void driveToLaunchPosition() {
    Pose2d currentPose = drive.pose;  // Fresh pose
    Action action = drive.actionBuilder(currentPose)
        .splineTo(launchPos, launchHeading)
        .build();
    startAction(action);
}
```

### Pre-Building for Autonomous

For autonomous, pre-building is fine if poses are known:

```java
// In init, before start:
Action auto = new SequentialAction(
    drive.actionBuilder(new Pose2d(0, 0, 0))  // Known start pose
        .splineTo(firstTarget, firstHeading)
        .build(),
    // ... etc
);

// In loop after start:
if (!autoStarted) {
    robot.startAction(auto);
    autoStarted = true;
}
```

### Lazy Action Building

For dynamic sequences:

```java
// Action that builds the next trajectory when it runs
Action lazyDriveToTarget = new Action() {
    private Action innerAction = null;

    @Override
    public boolean run(TelemetryPacket packet) {
        if (innerAction == null) {
            // Build trajectory NOW with current pose
            innerAction = drive.actionBuilder(drive.pose)
                .splineTo(dynamicTarget, dynamicHeading)
                .build();
        }
        return innerAction.run(packet);
    }
};
```

---

## Resuming Aborted Actions

### Strategy 1: Rebuild from Current Pose

Simplest approach - just build a new trajectory to the original target:

```java
private Pose2d savedTargetPose = null;

public void driveToLaunchPosition() {
    savedTargetPose = LAUNCH_POSITION;
    Action action = drive.actionBuilder(drive.pose)
        .splineTo(savedTargetPose.position, savedTargetPose.heading.toDouble())
        .build();
    startAction(action);
}

public void resumeAbortedAction() {
    if (savedTargetPose != null) {
        // Build fresh trajectory from where we are now
        Action action = drive.actionBuilder(drive.pose)
            .splineTo(savedTargetPose.position, savedTargetPose.heading.toDouble())
            .build();
        startAction(action);
    }
}
```

### Strategy 2: Waypoint Resume

If you aborted mid-sequence, resume from the next waypoint:

```java
private List<Pose2d> waypointQueue = new ArrayList<>();
private int currentWaypointIndex = 0;

public void startWaypointSequence(List<Pose2d> waypoints) {
    waypointQueue = new ArrayList<>(waypoints);
    currentWaypointIndex = 0;
    driveToNextWaypoint();
}

private void driveToNextWaypoint() {
    if (currentWaypointIndex < waypointQueue.size()) {
        Pose2d target = waypointQueue.get(currentWaypointIndex);
        Action action = drive.actionBuilder(drive.pose)
            .splineTo(target.position, target.heading.toDouble())
            .build();
        startAction(action);
    }
}

public void onActionComplete() {
    currentWaypointIndex++;
    driveToNextWaypoint();
}

public void resumeAfterAbort() {
    // Resume from current waypoint (don't increment)
    driveToNextWaypoint();
}
```

### Strategy 3: Time-Based Resume Point

For complex trajectories, estimate where you were:

```java
private TimeTrajectory savedTrajectory = null;
private double trajectoryStartTime = 0;
private double abortTime = 0;

public void abortCurrentAction() {
    abortTime = System.currentTimeMillis() / 1000.0;
    // ... cleanup
}

public void resumeAbortedAction() {
    if (savedTrajectory != null) {
        double elapsedBeforeAbort = abortTime - trajectoryStartTime;
        double remainingTime = savedTrajectory.duration - elapsedBeforeAbort;

        // Get the pose we should have been at
        Pose2d resumePose = savedTrajectory.get(elapsedBeforeAbort);
        Pose2d endPose = savedTrajectory.get(savedTrajectory.duration);

        // Build trajectory from current pose to end
        Action action = drive.actionBuilder(drive.pose)
            .splineTo(endPose.position, endPose.heading.toDouble())
            .build();
        startAction(action);
    }
}
```

### Driver-Controlled Resume Flow

```java
// In controls:
if (gamepad1.b) {
    // B = abort and take manual control
    robot.abortCurrentAction();
    manualControlActive = true;
}

if (gamepad1.a && manualControlActive) {
    // A = resume trajectory (after driver clears the obstacle)
    robot.resumeAbortedAction();
    manualControlActive = false;
}
```

---

## Reverse Trajectories

### Method 1: Negative Velocity Profile

Build trajectory that drives backward:

```java
// Drive backward to target (robot faces away from direction of travel)
Action reverseAction = drive.actionBuilder(currentPose)
    .setReversed(true)  // RoadRunner 1.0 method
    .splineTo(target, targetHeading)
    .build();
```

### Method 2: Turn and Drive Forward

Sometimes cleaner to turn first:

```java
Action turnThenDrive = drive.actionBuilder(currentPose)
    .turnTo(Math.atan2(target.y - currentPose.position.y,
                       target.x - currentPose.position.x) + Math.PI)  // Face away
    .setReversed(true)
    .lineTo(target)
    .build();
```

### Method 3: Explicit Backward Line

For tank drive, simple backward motion:

```java
// lineToX with reversed=true
Action backUp = drive.actionBuilder(currentPose)
    .setReversed(true)
    .lineToX(currentPose.position.x - 24)  // Back up 24 inches
    .build();
```

### Method 4: Manual Reverse for Evasion

For quick reverse (collision avoidance), skip trajectory planning:

```java
public void emergencyReverse(double seconds) {
    // Direct motor control, no trajectory
    startAction(new Action() {
        private double startTime = -1;

        @Override
        public boolean run(TelemetryPacket packet) {
            if (startTime < 0) startTime = Actions.now();

            double elapsed = Actions.now() - startTime;
            if (elapsed < seconds) {
                drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-0.5, 0),  // Negative = backward
                    0
                ));
                return true;
            } else {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                return false;
            }
        }
    });
}
```

### Reverse After Scoring

Common pattern - approach forward, back away:

```java
Action scoreAndRetreat = new SequentialAction(
    // Approach scoring position (forward)
    drive.actionBuilder(currentPose)
        .splineTo(scoringPos, scoringHeading)
        .build(),

    // Score action
    scoreGamePieceAction,

    // Back away (reversed)
    drive.actionBuilder(scoringPose)
        .setReversed(true)
        .lineToX(scoringPos.x - 12)  // Back up 12 inches
        .build()
);
```

---

## TankDriveActions (Turn-Spline-Turn)

Tank drives cannot independently control heading during motion -- the heading is always the tangent to the path. This means starting a spline with misaligned heading leads to accumulated errors that Ramsete struggles to correct.

**TankDriveActions** solves this by wrapping every spline trajectory with explicit turn corrections:

1. **Initial Turn** -- Face toward target before driving (ensures Ramsete starts with correct heading)
2. **Spline** -- Drive curved path to waypoint
3. **Final Turn** -- Correct to desired final heading

All missions use TankDriveActions internally. You should use it for any new navigation code.

### Setup

```java
// Create once (e.g., in constructor or init)
TankDriveActions actions = new TankDriveActions(robot.driveTrain);
```

### driveTo -- Single Waypoint

```java
// Drive to a pose: position (48, 24) with final heading 90 degrees
Action driveAction = actions.driveTo(new Pose2d(48, 24, Math.toRadians(90)));
robot.driveTrain.runAction(driveAction);
```

What happens internally:
1. Turns to face (48, 24) from current position (~26.6 degrees via atan2)
2. Splines to (48, 24) with end tangent = 90 degrees
3. Final turn to 90 degrees (skipped if already within tolerance)

### driveThrough -- Multiple Waypoints

```java
// Drive through a sequence of waypoints
Action multiAction = actions.driveThrough(
    new Pose2d(24, 0, Math.toRadians(0)),      // wp1
    new Pose2d(48, 24, Math.toRadians(45)),     // wp2
    new Pose2d(48, 48, Math.toRadians(90))      // wp3 (final heading matters)
);
robot.driveTrain.runAction(multiAction);
```

Intermediate waypoint headings set the spline tangent direction (pointing toward the next waypoint). Only the last waypoint's heading is used for the final turn.

### driveToReversed -- Driving Backwards

```java
// Back up to a pose (robot faces away from target, drives in reverse)
Action reverseAction = actions.driveToReversed(
    new Pose2d(0, 0, Math.toRadians(180))
);
robot.driveTrain.runAction(reverseAction);
```

What happens internally:
1. Turns to face **away** from target (bearing + 180 degrees)
2. Reversed spline to target (end tangent = target heading - 180 degrees, so robot arrives facing target heading)
3. Final turn if needed

### Velocity and Acceleration Constraints

All three methods accept optional `VelConstraint` and `AccelConstraint` overrides to slow down (or speed up) the spline segment. Turns always use default speed.

**Units:**
- Velocity: **inches per second** (default maxWheelVel = 50 in/s)
- Acceleration: **inches per second squared** (default range: -30 to +50 in/s^2)
- Angular velocity: **radians per second** (default maxAngVel = PI rad/s)

```java
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;

// Simple: limit translational speed only
Action slowDrive = actions.driveTo(target,
    new TranslationalVelConstraint(20.0),       // max 20 in/s (default is 50)
    null                                         // use default accel
);

// Both velocity and acceleration limits
Action precisionDrive = actions.driveTo(target,
    new TranslationalVelConstraint(15.0),        // max 15 in/s
    new ProfileAccelConstraint(-10.0, 15.0)      // gentler accel: -10 to +15 in/s^2
);

// Limit both translational and angular velocity
Action extraSlow = actions.driveTo(target,
    new MinVelConstraint(Arrays.asList(
        new TranslationalVelConstraint(10.0),    // max 10 in/s
        new AngularVelConstraint(Math.PI / 4)    // max PI/4 rad/s
    )),
    new ProfileAccelConstraint(-10.0, 10.0)
);

// Same works for driveThrough and driveToReversed
Action slowMulti = actions.driveThrough(
    new TranslationalVelConstraint(20.0),
    new ProfileAccelConstraint(-15.0, 20.0),
    waypoint1, waypoint2, waypoint3
);

Action slowReverse = actions.driveToReversed(target,
    new TranslationalVelConstraint(15.0),
    null
);
```

### Turn Skip Thresholds

If the robot is already facing close to the correct direction, the initial/final turns are skipped to save time. These thresholds are tunable via FTC Dashboard under `Lebot2_TankDriveActions`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `INITIAL_TURN_SKIP_TOLERANCE` | 2.0 degrees | Skip initial turn if heading error is within this |
| `FINAL_TURN_SKIP_TOLERANCE` | 1.0 degrees | Skip final turn if heading error is within this |

### Important Notes

- **Build just-in-time:** `driveTo()` reads the current pose when called. Build the action right before you run it, not in advance. See [Define vs Evaluate](#define-vs-evaluate).
- **Constraints only apply to the spline segment.** Turns always use the drivetrain's default turn behavior.
- **driveToReversed tangent math:** The spline end tangent is `targetHeading - PI` (not `targetHeading`). This is because `setReversed(true)` means the robot heading = path tangent + PI. The wrapper handles this for you.
- **Missions use this internally.** NavigateToFire, BallGroup, and OpenSesame all use TankDriveActions. You don't need to build raw trajectories for mission code.

---

## Quick Reference

| Scenario | Approach |
|----------|----------|
| Tuning/testing | `Actions.runBlocking()` |
| Competition auto | Manual `action.run()` in loop |
| Driver wants control | Abort and null the action |
| Collision imminent | `emergencyReverse()` or abort |
| Need subsystem updates | Non-blocking pattern required |
| Sequence of moves | `SequentialAction` |
| Parallel operations | `ParallelAction` |
| Interrupt with new target | Barge with fresh trajectory |
| Resume after abort | Rebuild from current pose to saved target |
| Drive backward | `.setReversed(true)` |
| **Navigate to a pose (tank drive)** | **`TankDriveActions.driveTo()`** |
| **Navigate through waypoints** | **`TankDriveActions.driveThrough()`** |
| **Navigate backwards to a pose** | **`TankDriveActions.driveToReversed()`** |
| **Slow down for accuracy** | **Pass `VelConstraint`/`AccelConstraint` to above** |

## Related Files

- `TankDriveActions.java` - Turn-Spline-Turn wrapper for tank drive navigation
- `TankDrivePinpoint.java` - Drive class with action builders
- `Robot.java` - Behavior state machine integration
- `PinpointConfiguration.md` - Localizer setup for accurate pose tracking
