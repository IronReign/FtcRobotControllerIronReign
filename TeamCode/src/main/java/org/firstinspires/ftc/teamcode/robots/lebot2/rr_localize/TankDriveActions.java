package org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

/**
 * Utility class for building Turn-Spline-Turn navigation actions for tank drives.
 *
 * Tank drives (differential drives) cannot independently control heading during motion -
 * the heading is always the tangent to the path. This leads to accumulated heading errors,
 * especially when starting with misaligned heading.
 *
 * SOLUTION: Bracket spline trajectories with explicit turn corrections:
 * 1. Initial Turn - Face toward target waypoint before driving
 * 2. Spline - Drive curved path to waypoint
 * 3. Final Turn - Correct to desired final heading
 *
 * This pattern ensures accurate starting heading (which Ramsete can maintain)
 * and accurate final heading (explicit correction).
 *
 * USAGE:
 * <pre>
 *   TankDriveActions actions = new TankDriveActions(driveTrain);
 *
 *   // Default speed
 *   Action driveAction = actions.driveTo(new Pose2d(48, 24, Math.toRadians(90)));
 *
 *   // Slower speed for accuracy
 *   Action slowAction = actions.driveTo(target,
 *       new TranslationalVelConstraint(20.0),
 *       new ProfileAccelConstraint(-15.0, 20.0));
 *
 *   driveTrain.runAction(driveAction);
 * </pre>
 */
@Config(value = "Lebot2_TankDriveActions")
public class TankDriveActions {

    // Dashboard-tunable turn skip thresholds (degrees)
    public static double INITIAL_TURN_SKIP_TOLERANCE = 2.0;  // Skip initial turn if within this
    public static double FINAL_TURN_SKIP_TOLERANCE = 1.0;    // Skip final turn if within this

    private final TankDrivePinpoint driveTrain;

    // The final target position from the last driveTo/driveThrough/driveToReversed call.
    // Use getLastTargetPosition() to retrieve this for visualization.
    private Vector2d lastTargetPosition = null;

    /**
     * Create a TankDriveActions utility for the given drivetrain.
     *
     * @param driveTrain The TankDrivePinpoint instance to build actions for
     */
    public TankDriveActions(TankDrivePinpoint driveTrain) {
        this.driveTrain = driveTrain;
    }

    /**
     * Get the final target position from the last driveTo/driveThrough/driveToReversed call.
     * This is the actual FieldMap waypoint position, not extracted from the RoadRunner action tree.
     *
     * @return The target position, or null if no action has been built yet
     */
    public Vector2d getLastTargetPosition() {
        return lastTargetPosition;
    }

    // ==================== driveTo ====================

    /**
     * Drive to a target pose using Turn-Spline-Turn pattern with default constraints.
     *
     * @param target Target pose (position + heading)
     * @return Action sequence: turnTo(bearing) -> spline -> turnTo(finalHeading)
     */
    public Action driveTo(Pose2d target) {
        return driveTo(target, null, null);
    }

    /**
     * Drive to a target pose using Turn-Spline-Turn pattern with custom constraints.
     *
     * Executes:
     * 1. Turn to face target position (if needed)
     * 2. Spline to target position arriving straight (end tangent = bearing to target)
     * 3. Turn to final heading (if different from arrival heading)
     *
     * The spline does NOT try to match the final heading -- it arrives "straight" at the
     * target position. This avoids aggressive end-of-spline corrections that cause overshoot
     * on tank drives. The explicit final turn handles heading accurately.
     *
     * @param target Target pose (position + heading)
     * @param velConstraint Velocity constraint override for spline segment, or null for default
     * @param accelConstraint Acceleration constraint override for spline segment, or null for default
     * @return Action sequence: turnTo(bearing) -> spline -> turnTo(finalHeading)
     */
    public Action driveTo(Pose2d target, VelConstraint velConstraint, AccelConstraint accelConstraint) {
        lastTargetPosition = target.position;
        Pose2d current = driveTrain.getPose();
        double bearingToTarget = bearingTo(current.position, target.position);

        // All three segments are lazy -- built from actual pose at runtime
        Action initialTurn = new LazyTurnAction(bearingToTarget, INITIAL_TURN_SKIP_TOLERANCE);
        Action splineDrive = new LazySplineAction(target.position, false, velConstraint, accelConstraint);
        Action finalTurn = new LazyTurnAction(target.heading.toDouble(), FINAL_TURN_SKIP_TOLERANCE);

        return new SequentialAction(initialTurn, splineDrive, finalTurn);
    }

    // ==================== driveThrough ====================

    /**
     * Drive through multiple waypoints using Turn-Spline-Turn pattern with default constraints.
     *
     * @param waypoints Sequence of poses to drive through
     * @return Action sequence: turnTo(first) -> spline(all) -> turnTo(lastHeading)
     */
    public Action driveThrough(Pose2d... waypoints) {
        return driveThrough(null, null, waypoints);
    }

    /**
     * Drive through multiple waypoints using Turn-Spline-Turn pattern with custom constraints.
     *
     * Executes:
     * 1. Turn to face first waypoint (if needed)
     * 2. Spline through all waypoints (tangent at each = bearing to next, last = bearing from prev)
     * 3. Turn to final heading (if different from arrival heading)
     *
     * All spline tangents are set to the direction of travel between waypoints.
     * The spline never tries to match a desired heading -- that's the final turn's job.
     *
     * @param velConstraint Velocity constraint override for spline segments, or null for default
     * @param accelConstraint Acceleration constraint override for spline segments, or null for default
     * @param waypoints Sequence of poses to drive through
     * @return Action sequence: turnTo(first) -> spline(all) -> turnTo(lastHeading)
     */
    public Action driveThrough(VelConstraint velConstraint, AccelConstraint accelConstraint, Pose2d... waypoints) {
        if (waypoints.length > 0) {
            lastTargetPosition = waypoints[waypoints.length - 1].position;
        }
        if (waypoints.length == 0) {
            return new InstantAction(() -> {});
        }
        if (waypoints.length == 1) {
            return driveTo(waypoints[0], velConstraint, accelConstraint);
        }

        Pose2d current = driveTrain.getPose();
        Pose2d last = waypoints[waypoints.length - 1];
        double bearingToFirst = bearingTo(current.position, waypoints[0].position);

        // Extract positions for lazy spline builder
        Vector2d[] positions = new Vector2d[waypoints.length];
        for (int i = 0; i < waypoints.length; i++) {
            positions[i] = waypoints[i].position;
        }

        // All three segments are lazy -- built from actual pose at runtime
        Action initialTurn = new LazyTurnAction(bearingToFirst, INITIAL_TURN_SKIP_TOLERANCE);
        Action splineDrive = new LazySplineAction(positions, velConstraint, accelConstraint);
        Action finalTurn = new LazyTurnAction(last.heading.toDouble(), FINAL_TURN_SKIP_TOLERANCE);

        return new SequentialAction(initialTurn, splineDrive, finalTurn);
    }

    // ==================== driveToReversed ====================

    /**
     * Drive to a target pose in reverse using Turn-Spline-Turn pattern with default constraints.
     *
     * @param target Target pose (position + heading)
     * @return Action sequence: turnTo(bearing+180) -> reversed spline -> turnTo(finalHeading)
     */
    public Action driveToReversed(Pose2d target) {
        return driveToReversed(target, null, null);
    }

    /**
     * Drive to a target pose in reverse using Turn-Spline-Turn pattern with custom constraints.
     *
     * For reverse trajectories, the robot faces AWAY from the target and drives backwards.
     * The spline arrives "straight" (end tangent = bearing to target) without trying to
     * match the desired final heading. The explicit final turn handles heading.
     *
     * Executes:
     * 1. Turn to face away from target (if needed)
     * 2. Reversed spline to target position (arriving straight)
     * 3. Turn to final heading (if needed)
     *
     * @param target Target pose (position + heading)
     * @param velConstraint Velocity constraint override for spline segment, or null for default
     * @param accelConstraint Acceleration constraint override for spline segment, or null for default
     * @return Action sequence: turnTo(bearing+180) -> reversed spline -> turnTo(finalHeading)
     */
    public Action driveToReversed(Pose2d target, VelConstraint velConstraint, AccelConstraint accelConstraint) {
        lastTargetPosition = target.position;
        Pose2d current = driveTrain.getPose();
        double bearingToTarget = bearingTo(current.position, target.position);
        double bearingAwayFromTarget = normalizeAngle(bearingToTarget + Math.PI);

        // All three segments are lazy -- built from actual pose at runtime
        Action initialTurn = new LazyTurnAction(bearingAwayFromTarget, INITIAL_TURN_SKIP_TOLERANCE);
        Action splineDrive = new LazySplineAction(target.position, true, velConstraint, accelConstraint);
        Action finalTurn = new LazyTurnAction(target.heading.toDouble(), FINAL_TURN_SKIP_TOLERANCE);

        return new SequentialAction(initialTurn, splineDrive, finalTurn);
    }

    // ==================== LazyTurnAction ====================

    /**
     * An Action that defers building the RoadRunner turnTo until its first run() call.
     *
     * RoadRunner's turnTo pre-computes a motion profile at build() time, baking in the
     * expected start heading. If the robot's actual heading differs (due to spline tracking
     * error, overshoot, etc.), the pre-computed turn executes the wrong correction.
     *
     * LazyTurnAction solves this by waiting until it actually runs (after the previous action
     * in a SequentialAction completes) to read the robot's real pose and build an accurate turn.
     */
    private class LazyTurnAction implements Action {
        private final double targetHeadingRad;
        private final double skipToleranceDeg;
        private Action innerAction = null;
        private boolean complete = false;

        LazyTurnAction(double targetHeadingRad, double skipToleranceDeg) {
            this.targetHeadingRad = targetHeadingRad;
            this.skipToleranceDeg = skipToleranceDeg;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (complete) return false;

            if (innerAction == null) {
                // First call -- build from actual current pose
                Pose2d currentPose = driveTrain.getPose();
                double headingError = Math.toDegrees(Math.abs(
                        normalizeAngle(targetHeadingRad - currentPose.heading.toDouble())));

                if (headingError <= skipToleranceDeg) {
                    complete = true;
                    return false;  // Within tolerance, skip
                }

                // Build real RoadRunner turnTo from actual pose
                innerAction = driveTrain.actionBuilder(currentPose)
                        .turnTo(targetHeadingRad)
                        .build();
            }

            return innerAction.run(packet);
        }
    }

    // ==================== LazySplineAction ====================

    /**
     * An Action that defers building the RoadRunner spline until its first run() call.
     *
     * The spline start position is baked in at build() time. If the robot drifts during the
     * preceding turn (especially large turns like 90°+), the pre-built spline starts from the
     * wrong position, causing Ramsete to fight a position error that compounds into overshoot.
     *
     * LazySplineAction waits until it actually runs (after the initial turn completes) to read
     * the robot's real pose and build a spline from the correct start position.
     *
     * Supports single-target (driveTo/driveToReversed) and multi-target (driveThrough) modes.
     */
    private class LazySplineAction implements Action {
        private final Vector2d[] targets;
        private final boolean reversed;
        private final VelConstraint velConstraint;
        private final AccelConstraint accelConstraint;
        private Action innerAction = null;

        /** Single target (forward or reversed). */
        LazySplineAction(Vector2d target, boolean reversed,
                         VelConstraint velConstraint, AccelConstraint accelConstraint) {
            this.targets = new Vector2d[]{target};
            this.reversed = reversed;
            this.velConstraint = velConstraint;
            this.accelConstraint = accelConstraint;
        }

        /** Multiple waypoints (forward only). */
        LazySplineAction(Vector2d[] targets,
                         VelConstraint velConstraint, AccelConstraint accelConstraint) {
            this.targets = targets;
            this.reversed = false;
            this.velConstraint = velConstraint;
            this.accelConstraint = accelConstraint;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (innerAction == null) {
                // First call -- build from actual current pose (post-turn)
                Pose2d currentPose = driveTrain.getPose();

                if (targets.length == 1) {
                    innerAction = buildSingleTarget(currentPose, targets[0]);
                } else {
                    innerAction = buildMultiTarget(currentPose, targets);
                }
            }
            return innerAction.run(packet);
        }

        private Action buildSingleTarget(Pose2d currentPose, Vector2d target) {
            double bearing = bearingTo(currentPose.position, target);

            TrajectoryActionBuilder builder;
            if (reversed) {
                // Robot heading is facing away from target; path tangent is toward target
                builder = driveTrain.actionBuilder(currentPose).setReversed(true);
            } else {
                builder = driveTrain.actionBuilder(currentPose);
            }

            // End tangent = bearing from actual position (arrive straight)
            if (velConstraint != null || accelConstraint != null) {
                return builder.splineTo(target, bearing, velConstraint, accelConstraint).build();
            } else {
                return builder.splineTo(target, bearing).build();
            }
        }

        private Action buildMultiTarget(Pose2d currentPose, Vector2d[] waypoints) {
            TrajectoryActionBuilder builder = driveTrain.actionBuilder(currentPose);
            double bearingToFirst = bearingTo(currentPose.position, waypoints[0]);

            for (int i = 0; i < waypoints.length; i++) {
                double tangent;
                if (i < waypoints.length - 1) {
                    tangent = bearingTo(waypoints[i], waypoints[i + 1]);
                } else {
                    tangent = (i > 0)
                            ? bearingTo(waypoints[i - 1], waypoints[i])
                            : bearingToFirst;
                }
                if (velConstraint != null || accelConstraint != null) {
                    builder = builder.splineTo(waypoints[i], tangent, velConstraint, accelConstraint);
                } else {
                    builder = builder.splineTo(waypoints[i], tangent);
                }
            }
            return builder.build();
        }
    }

    // ==================== HELPER METHODS ====================

    /**
     * Calculate bearing (angle) from one position to another.
     *
     * @param from Starting position
     * @param to Target position
     * @return Bearing in radians (atan2 convention: 0 = +X, PI/2 = +Y)
     */
    private double bearingTo(Vector2d from, Vector2d to) {
        return Math.atan2(to.y - from.y, to.x - from.x);
    }

    /**
     * Normalize angle to [-PI, PI] range.
     *
     * @param angle Angle in radians
     * @return Normalized angle in radians
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
