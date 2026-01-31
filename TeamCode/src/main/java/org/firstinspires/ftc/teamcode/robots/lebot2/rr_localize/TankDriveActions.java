package org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize;

import com.acmerobotics.dashboard.config.Config;
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

    /**
     * Create a TankDriveActions utility for the given drivetrain.
     *
     * @param driveTrain The TankDrivePinpoint instance to build actions for
     */
    public TankDriveActions(TankDrivePinpoint driveTrain) {
        this.driveTrain = driveTrain;
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
     * 2. Spline to target position with arrival tangent = target heading
     * 3. Turn to final heading (if different from arrival tangent)
     *
     * @param target Target pose (position + heading)
     * @param velConstraint Velocity constraint override for spline segment, or null for default
     * @param accelConstraint Acceleration constraint override for spline segment, or null for default
     * @return Action sequence: turnTo(bearing) -> spline -> turnTo(finalHeading)
     */
    public Action driveTo(Pose2d target, VelConstraint velConstraint, AccelConstraint accelConstraint) {
        Pose2d current = driveTrain.getPose();

        // Calculate bearing from current position to target position
        double bearingToTarget = bearingTo(current.position, target.position);

        // Check if initial turn is needed
        double bearingError = Math.toDegrees(Math.abs(normalizeAngle(bearingToTarget - current.heading.toDouble())));
        boolean needsInitialTurn = bearingError > INITIAL_TURN_SKIP_TOLERANCE;

        // Build intended start pose (current position, facing target)
        Pose2d intendedStartPose = new Pose2d(current.position, bearingToTarget);

        // For forward driving, robot heading = path tangent at end of spline
        // splineTo end tangent = target.heading, so robot ends at target.heading
        double splineEndTangent = target.heading.toDouble();
        double arrivalError = Math.toDegrees(Math.abs(normalizeAngle(target.heading.toDouble() - splineEndTangent)));
        boolean needsFinalTurn = arrivalError > FINAL_TURN_SKIP_TOLERANCE;

        // Build the action sequence
        Action initialTurn = needsInitialTurn
                ? driveTrain.actionBuilder(current).turnTo(bearingToTarget).build()
                : new InstantAction(() -> {});

        TrajectoryActionBuilder splineBuilder = driveTrain.actionBuilder(intendedStartPose);
        Action splineDrive;
        if (velConstraint != null || accelConstraint != null) {
            splineDrive = splineBuilder
                    .splineTo(target.position, target.heading.toDouble(), velConstraint, accelConstraint)
                    .build();
        } else {
            splineDrive = splineBuilder
                    .splineTo(target.position, target.heading.toDouble())
                    .build();
        }

        Action finalTurn = needsFinalTurn
                ? driveTrain.actionBuilder(new Pose2d(target.position, target.heading))
                        .turnTo(target.heading.toDouble())
                        .build()
                : new InstantAction(() -> {});

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
     * 2. Spline through all waypoints (tangent at each = bearing to next, last = its heading)
     * 3. Turn to final heading (if different from last spline tangent)
     *
     * @param velConstraint Velocity constraint override for spline segments, or null for default
     * @param accelConstraint Acceleration constraint override for spline segments, or null for default
     * @param waypoints Sequence of poses to drive through
     * @return Action sequence: turnTo(first) -> spline(all) -> turnTo(lastHeading)
     */
    public Action driveThrough(VelConstraint velConstraint, AccelConstraint accelConstraint, Pose2d... waypoints) {
        if (waypoints.length == 0) {
            return new InstantAction(() -> {});
        }
        if (waypoints.length == 1) {
            return driveTo(waypoints[0], velConstraint, accelConstraint);
        }

        Pose2d current = driveTrain.getPose();
        Pose2d first = waypoints[0];
        Pose2d last = waypoints[waypoints.length - 1];

        // Bearing to first waypoint
        double bearingToFirst = bearingTo(current.position, first.position);

        // Check if initial turn is needed
        double bearingError = Math.toDegrees(Math.abs(normalizeAngle(bearingToFirst - current.heading.toDouble())));
        boolean needsInitialTurn = bearingError > INITIAL_TURN_SKIP_TOLERANCE;

        // Intended start pose (current position, facing first waypoint)
        Pose2d intendedStartPose = new Pose2d(current.position, bearingToFirst);

        // Build spline through all waypoints
        TrajectoryActionBuilder builder = driveTrain.actionBuilder(intendedStartPose);
        for (int i = 0; i < waypoints.length; i++) {
            Pose2d wp = waypoints[i];
            double tangent;
            if (i < waypoints.length - 1) {
                // Tangent points toward next waypoint
                tangent = bearingTo(wp.position, waypoints[i + 1].position);
            } else {
                // Last waypoint: tangent = its heading
                tangent = wp.heading.toDouble();
            }
            if (velConstraint != null || accelConstraint != null) {
                builder = builder.splineTo(wp.position, tangent, velConstraint, accelConstraint);
            } else {
                builder = builder.splineTo(wp.position, tangent);
            }
        }

        // For forward driving, robot heading = path tangent = last waypoint heading
        double splineEndTangent = last.heading.toDouble();
        double arrivalError = Math.toDegrees(Math.abs(normalizeAngle(last.heading.toDouble() - splineEndTangent)));
        boolean needsFinalTurn = arrivalError > FINAL_TURN_SKIP_TOLERANCE;

        // Build the action sequence
        Action initialTurn = needsInitialTurn
                ? driveTrain.actionBuilder(current).turnTo(bearingToFirst).build()
                : new InstantAction(() -> {});

        Action splineDrive = builder.build();

        Action finalTurn = needsFinalTurn
                ? driveTrain.actionBuilder(new Pose2d(last.position, last.heading))
                        .turnTo(last.heading.toDouble())
                        .build()
                : new InstantAction(() -> {});

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
     *
     * Executes:
     * 1. Turn to face away from target (if needed)
     * 2. Reversed spline to target position
     * 3. Turn to final heading (if needed)
     *
     * @param target Target pose (position + heading)
     * @param velConstraint Velocity constraint override for spline segment, or null for default
     * @param accelConstraint Acceleration constraint override for spline segment, or null for default
     * @return Action sequence: turnTo(bearing+180) -> reversed spline -> turnTo(finalHeading)
     */
    public Action driveToReversed(Pose2d target, VelConstraint velConstraint, AccelConstraint accelConstraint) {
        Pose2d current = driveTrain.getPose();

        // Calculate bearing from current position to target position
        double bearingToTarget = bearingTo(current.position, target.position);

        // Face AWAY from target (will drive backwards)
        double bearingAwayFromTarget = normalizeAngle(bearingToTarget + Math.PI);

        // Check if initial turn is needed
        double bearingError = Math.toDegrees(Math.abs(normalizeAngle(bearingAwayFromTarget - current.heading.toDouble())));
        boolean needsInitialTurn = bearingError > INITIAL_TURN_SKIP_TOLERANCE;

        // Intended start pose (current position, facing away from target)
        Pose2d intendedStartPose = new Pose2d(current.position, bearingAwayFromTarget);

        // For reversed spline: splineTo tangent is the PATH tangent direction.
        // With setReversed(true), robot heading = path tangent + PI.
        // To end with robot heading = target.heading, path tangent must = target.heading - PI.
        double endTangent = normalizeAngle(target.heading.toDouble() - Math.PI);

        // After reversed spline, robot heading = endTangent + PI = target.heading
        double robotHeadingAtEnd = normalizeAngle(endTangent + Math.PI);
        Pose2d intendedEndPose = new Pose2d(target.position, robotHeadingAtEnd);

        // Check if final turn is needed (compare target heading to actual robot heading after spline)
        double arrivalError = Math.toDegrees(Math.abs(normalizeAngle(target.heading.toDouble() - robotHeadingAtEnd)));
        boolean needsFinalTurn = arrivalError > FINAL_TURN_SKIP_TOLERANCE;

        // Build the action sequence
        Action initialTurn = needsInitialTurn
                ? driveTrain.actionBuilder(current).turnTo(bearingAwayFromTarget).build()
                : new InstantAction(() -> {});

        TrajectoryActionBuilder splineBuilder = driveTrain.actionBuilder(intendedStartPose)
                .setReversed(true);
        Action splineDrive;
        if (velConstraint != null || accelConstraint != null) {
            splineDrive = splineBuilder
                    .splineTo(target.position, endTangent, velConstraint, accelConstraint)
                    .build();
        } else {
            splineDrive = splineBuilder
                    .splineTo(target.position, endTangent)
                    .build();
        }

        Action finalTurn = needsFinalTurn
                ? driveTrain.actionBuilder(intendedEndPose)
                        .turnTo(target.heading.toDouble())
                        .build()
                : new InstantAction(() -> {});

        return new SequentialAction(initialTurn, splineDrive, finalTurn);
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
