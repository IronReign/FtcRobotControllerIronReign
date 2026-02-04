package org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.ArrayList;
import java.util.List;

/**
 * Position-based drive controller for tank drives.
 *
 * Replaces RoadRunner spline trajectories with pure position-feedback driving:
 * - Distance PID controls drive speed based on remaining distance to target
 * - Heading PID steers the robot toward (or away from) the target
 * - Drive power is scaled by cos(headingError): heading way off → only turn, aligned → full drive
 * - Slew rate limiter smooths acceleration/deceleration to prevent wheelies
 *
 * Architecture (Turn-Drive-Turn):
 *   SequentialAction [
 *       LazyTurnAction(bearing to target),               // face toward target
 *       PositionDriveAction(target.position, reversed),  // drive to position
 *       LazyTurnAction(target.heading)                   // correct final heading
 *   ]
 *
 * LazyTurnAction uses RR TurnAction (with feedforward + motion profiling) for accurate turns.
 * PositionDriveAction uses pure PID feedback for accurate straight-line driving.
 */
@Config(value = "Lebot2_TankDriveActions")
public class TankDriveActions {

    // Dashboard-tunable turn skip thresholds (degrees)
    public static double INITIAL_TURN_SKIP_TOLERANCE = 2.0;  // Skip initial turn if within this
    public static double FINAL_TURN_SKIP_TOLERANCE = 2.0;    // Skip final turn if within this

    // Position drive PID coefficients (Dashboard-tunable)
    public static PIDCoefficients DISTANCE_PID = new PIDCoefficients(0.04, 0.04, 2.0);
    public static double DIST_PID_I_CUTIN = 8.0;  // need to be within 8" for integration to enable
    public static PIDCoefficients HEADING_DRIVE_PID = new PIDCoefficients(0.03, 0.03, 0.001);
    public static double HEAD_PID_I_CUTIN = 5.0; // need to be withing 5 degrees for integration to go active
    // Position drive thresholds
    public static double POSITION_TOLERANCE = 1.5;  // inches — completion threshold
    public static double MAX_DRIVE_POWER = 1;     // cap to prevent wheelies      //.8 <--ORIGINAL

    // Slew rate limits (power change per tick, ~50ms loop)
    public static double ACCEL_SLEW_RATE = 0.05;           // max power increase per tick
    public static double DECEL_SLEW_RATE_FORWARD = 0.08;   // max power decrease per tick (forward)
    public static double DECEL_SLEW_RATE_REVERSE = 0.04;   // gentler decel in reverse (anti-wheelie)

    // Settling timeout — safety net if PID oscillates and never converges
    public static double SETTLING_TIMEOUT_MS = 500;


    private final TankDrivePinpoint driveTrain;

    // The final target position from the last driveTo/driveThrough/driveToReversed call.
    private Vector2d lastTargetPosition = null;

    public TankDriveActions(TankDrivePinpoint driveTrain) {
        this.driveTrain = driveTrain;
    }

    /**
     * Get the final target position from the last driveTo/driveThrough/driveToReversed call.
     */
    public Vector2d getLastTargetPosition() {
        return lastTargetPosition;
    }

    // ==================== driveTo ====================

    /**
     * Drive to a target pose with default constraints.
     */
    public Action driveTo(Pose2d target) {
        return driveTo(target, MAX_DRIVE_POWER);
    }

    /**
     * Drive to a target pose with a custom max power.
     * Use lower values for slower driving (e.g., during intake).
     */
    public Action driveTo(Pose2d target, double maxPower) {
        lastTargetPosition = target.position;

        Action initialTurn = new LazyBearingTurnAction(target.position, false, INITIAL_TURN_SKIP_TOLERANCE);
        Action drive = new PositionDriveAction(target.position, false, maxPower);
        Action finalTurn = new LazyTurnAction(target.heading.toDouble(), FINAL_TURN_SKIP_TOLERANCE);
        return new SequentialAction(initialTurn, drive, finalTurn);
    }

    /**
     * Drive to a target pose using Turn-Drive-Turn pattern.
     *
     * 1. Initial turn — face toward target (RR TurnAction via LazyTurnAction)
     * 2. Drive — position-feedback to target (PositionDriveAction)
     * 3. Final turn — correct to desired heading (RR TurnAction via LazyTurnAction)
     *
     * @param target Target pose (position + heading)
     * @param velConstraint Accepted for API compatibility, currently unused
     * @param accelConstraint Accepted for API compatibility, currently unused
     * @return Action sequence: initialTurn -> PositionDriveAction -> finalTurn
     */
    public Action driveTo(Pose2d target, VelConstraint velConstraint, AccelConstraint accelConstraint) {
        lastTargetPosition = target.position;

        Action initialTurn = new LazyBearingTurnAction(target.position, false, INITIAL_TURN_SKIP_TOLERANCE);
        Action drive = new PositionDriveAction(target.position, false, MAX_DRIVE_POWER);
        Action finalTurn = new LazyTurnAction(target.heading.toDouble(), FINAL_TURN_SKIP_TOLERANCE);
        return new SequentialAction(initialTurn, drive, finalTurn);
    }

    // ==================== driveThrough ====================

    /**
     * Drive through multiple waypoints with default constraints.
     */
    public Action driveThrough(Pose2d... waypoints) {
        return driveThrough(null, null, waypoints);
    }

    /**
     * Drive through multiple waypoints using Turn-Drive pattern per waypoint.
     *
     * For each waypoint: turn to face it, then drive to it.
     * Final LazyTurnAction corrects to the last waypoint's heading.
     *
     * @param velConstraint Accepted for API compatibility, currently unused
     * @param accelConstraint Accepted for API compatibility, currently unused
     * @param waypoints Sequence of poses to drive through
     * @return Action sequence: [turn -> drive per waypoint] -> finalTurn
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

        List<Action> actions = new ArrayList<>();

        // Initial turn toward first waypoint (computed lazily at runtime)
        actions.add(new LazyBearingTurnAction(waypoints[0].position, false, INITIAL_TURN_SKIP_TOLERANCE));

        for (int i = 0; i < waypoints.length; i++) {
            // Drive to this waypoint
            actions.add(new PositionDriveAction(waypoints[i].position, false, MAX_DRIVE_POWER));

            // Turn toward next waypoint (computed lazily from actual arrival position)
            if (i < waypoints.length - 1) {
                actions.add(new LazyBearingTurnAction(waypoints[i + 1].position, false, INITIAL_TURN_SKIP_TOLERANCE));
            }
        }

        // Final heading correction
        Pose2d last = waypoints[waypoints.length - 1];
        actions.add(new LazyTurnAction(last.heading.toDouble(), FINAL_TURN_SKIP_TOLERANCE));

        return new SequentialAction(actions.toArray(new Action[0]));
    }

    // ==================== driveToReversed ====================

    /**
     * Drive to a target pose in reverse with default constraints.
     */
    public Action driveToReversed(Pose2d target) {
        return driveToReversed(target, null, null);
    }

    /**
     * Drive to a target pose in reverse using Turn-Drive-Turn pattern.
     *
     * 1. Initial turn — face away from target (RR TurnAction via LazyTurnAction)
     * 2. Drive — reversed position-feedback to target (PositionDriveAction)
     * 3. Final turn — correct to desired heading (RR TurnAction via LazyTurnAction)
     *
     * @param target Target pose (position + heading)
     * @param velConstraint Accepted for API compatibility, currently unused
     * @param accelConstraint Accepted for API compatibility, currently unused
     * @return Action sequence: initialTurn -> PositionDriveAction(reversed) -> finalTurn
     */
    public Action driveToReversed(Pose2d target, VelConstraint velConstraint, AccelConstraint accelConstraint) {
        lastTargetPosition = target.position;

        Action initialTurn = new LazyBearingTurnAction(target.position, true, INITIAL_TURN_SKIP_TOLERANCE);
        Action drive = new PositionDriveAction(target.position, true, MAX_DRIVE_POWER);
        Action finalTurn = new LazyTurnAction(target.heading.toDouble(), FINAL_TURN_SKIP_TOLERANCE);
        return new SequentialAction(initialTurn, drive, finalTurn);
    }

    // ==================== PositionDriveAction ====================

    /**
     * Pure position-feedback drive action using distance + heading PID.
     *
     * Drive power = distancePID(distance) * cos(headingError)
     * Steer power = headingPID(headingError)
     * Tank mixing: left = drive + steer, right = drive - steer
     *
     * The cos(headingError) scaling means:
     * - 90° off target → cos=0 → robot only turns, no forward drive
     * - Perfectly aligned → cos=1 → full forward drive
     * This eliminates the need for a separate initial turn action.
     *
     * A slew rate limiter smooths drive power changes to prevent wheelies.
     *
     * For reversed driving: heading targets bearing + PI, drive power is negated.
     */
    private class PositionDriveAction implements Action {
        private final Vector2d target;
        private final boolean reversed;
        private final double maxPower;

        private PIDController distancePID;
        private PIDController headingPID;
        private boolean initialized = false;
        private double previousDrivePower = 0.0;
        private double lockedHeading = 0.0;  // Fixed heading reference, set at init
        private long settlingStartTime = -1;  // Timestamp when robot first crosses finish line

        PositionDriveAction(Vector2d target, boolean reversed, double maxPower) {
            this.target = target;
            this.reversed = reversed;
            this.maxPower = maxPower;
        }

        private void initialize() {
            distancePID = new PIDController(DISTANCE_PID);
            distancePID.setInputRange(-50, 50);  // Saturates well before 50"; allows negative for overshoot
            distancePID.setOutputRange(-1.0, 1.0);
            distancePID.setInput(0);
            distancePID.enable();

            headingPID = new PIDController(HEADING_DRIVE_PID);
            headingPID.setInputRange(-180, 180);
            headingPID.setOutputRange(-1.0, 1.0);
            headingPID.setContinuous(true);
            headingPID.setSetpoint(0);  // target is 0 heading error
            headingPID.enable();

            // Switch to FLOAT so PID has full authority over deceleration.
            // BRAKE mode shorts the windings at near-zero power, causing abrupt stops
            // that tip the robot (wheelie) — especially in reverse.
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // Use the robot's current heading as the locked reference.
            // The initial LazyBearingTurnAction already aligned the robot toward
            // (or away from) the target using the pre-turn position. If the Pinpoint
            // reports position drift during the turn, recomputing the bearing here
            // would give a different (wrong) angle. Trusting the physical heading
            // the robot is actually on avoids this ~12-15° systematic error.
            Pose2d currentPose = driveTrain.getPose();
            lockedHeading = currentPose.heading.toDouble();

            // Diagnostic: log init state — compare bearing vs actual heading
            double bearing = bearingTo(currentPose.position, target);
            double expectedHeading = reversed ? normalizeAngle(bearing + Math.PI) : bearing;
            double bearingDelta = Math.toDegrees(normalizeAngle(expectedHeading - lockedHeading));
            System.out.println(String.format(
                    "PosDrive INIT: pos=(%.1f,%.1f) hdg=%.1f° bearing=%.1f° bearingDelta=%.1f° dist=%.1f tgt=(%.1f,%.1f) rev=%b",
                    currentPose.position.x, currentPose.position.y,
                    Math.toDegrees(lockedHeading),
                    Math.toDegrees(reversed ? normalizeAngle(bearing + Math.PI) : bearing),
                    bearingDelta,
                    Math.sqrt(Math.pow(target.x - currentPose.position.x, 2) + Math.pow(target.y - currentPose.position.y, 2)),
                    target.x, target.y,
                    reversed));

            previousDrivePower = 0.0;
            settlingStartTime = -1;
            initialized = true;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialize();
            }

            // Update PID coefficients from Dashboard each tick
            distancePID.setPID(DISTANCE_PID);
            distancePID.setIntegralCutIn(DIST_PID_I_CUTIN);
            headingPID.setPID(HEADING_DRIVE_PID);
            headingPID.setIntegralCutIn(HEAD_PID_I_CUTIN);

            Pose2d currentPose = driveTrain.getPose();
            double currentHeading = currentPose.heading.toDouble();

            // Compute vector from robot to target
            double dx = target.x - currentPose.position.x;
            double dy = target.y - currentPose.position.y;
            double distance = Math.sqrt(dx * dx + dy * dy);

            // Along-track distance: projection of robot-to-target vector onto drive direction.
            // Positive = target is ahead, negative = robot has overshot past the target.
            double driveDirection = reversed ? normalizeAngle(lockedHeading + Math.PI) : lockedHeading;
            double alongTrack = dx * Math.cos(driveDirection) + dy * Math.sin(driveDirection);

            // Distance PID: setpoint is along-track distance, input stays 0.
            // error = alongTrack. Positive → drive forward, negative → reverse to correct overshoot.
            distancePID.setSetpoint(alongTrack);
            double rawDrivePower = distancePID.performPID();

            // Primary termination: along-track error within tolerance
            if (Math.abs(alongTrack) < POSITION_TOLERANCE) {
                driveTrain.setMotorPowers(0, 0);
                setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                previousDrivePower = 0.0;
                return false;
            }

            // Start settling timer when robot first crosses the finish line (overshoot).
            // This is the safety net — if the PID oscillates and never converges,
            // the timeout will terminate the action.
            if (alongTrack <= 0 && settlingStartTime < 0) {
                settlingStartTime = System.currentTimeMillis();
            }
            if (settlingStartTime > 0 &&
                    System.currentTimeMillis() - settlingStartTime > SETTLING_TIMEOUT_MS) {
                driveTrain.setMotorPowers(0, 0);
                setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                previousDrivePower = 0.0;
                return false;
            }


            // Heading error relative to locked heading (set at init from initial bearing)
            double headingError = normalizeAngle(lockedHeading - currentHeading);

            // Scale by cos(headingError) — no forward drive when heading is way off
            double drivePower = rawDrivePower * Math.cos(headingError);

            // For reversed: negate drive power (motors run backward)
            if (reversed) {
                drivePower = -drivePower;
            }

            // Cap drive power
            drivePower = Math.max(-maxPower, Math.min(maxPower, drivePower));

            // Slew rate limiter — bypassed entirely when correcting overshoot
            if (alongTrack > 0) {
                drivePower = applySlewRate(drivePower, previousDrivePower);
            }
            previousDrivePower = drivePower;

            // Heading PID: input is heading error in degrees, setpoint is 0
            headingPID.setInput(Math.toDegrees(headingError));
            double steerPower = headingPID.performPID();

            // Tank mixing
            double leftPower = drivePower + steerPower;
            double rightPower = drivePower - steerPower;

            // Normalize if exceeding [-1, 1]
            double maxMag = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxMag > 1.0) {
                leftPower /= maxMag;
                rightPower /= maxMag;
            }

            driveTrain.setMotorPowers(leftPower, rightPower);

            // Telemetry for Dashboard
            packet.put("posDrive_dist", distance);
            packet.put("posDrive_along", alongTrack);
            packet.put("posDrive_headErr", Math.toDegrees(headingError));
            packet.put("posDrive_drive", drivePower);
            packet.put("posDrive_steer", steerPower);
            packet.put("posDrive_left", leftPower);
            packet.put("posDrive_right", rightPower);
            packet.put("posDrive_settling", settlingStartTime > 0);

            return true;
        }

        /**
         * Apply slew rate limiting to drive power for smooth accel/decel.
         * Forward deceleration is unslewed — the PID has full authority to stop.
         * Only acceleration and reverse deceleration are slew-limited (wheelie prevention).
         */
        private double applySlewRate(double desired, double previous) {
            double delta = desired - previous;

            if (Math.abs(desired) > Math.abs(previous)) {
                // Accelerating — cap increase rate
                if (Math.abs(delta) > ACCEL_SLEW_RATE) {
                    return previous + Math.signum(delta) * ACCEL_SLEW_RATE;
                }
            } else if (reversed) {
                // Decelerating in reverse — slew to prevent wheelies on hard stops
                if (Math.abs(delta) > DECEL_SLEW_RATE_REVERSE) {
                    return previous + Math.signum(delta) * DECEL_SLEW_RATE_REVERSE;
                }
            }
            // Forward deceleration: no slew — PID has full authority
            return desired;
        }

        private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
            for (DcMotorEx m : driveTrain.leftMotors) {
                m.setZeroPowerBehavior(behavior);
            }
            for (DcMotorEx m : driveTrain.rightMotors) {
                m.setZeroPowerBehavior(behavior);
            }
        }
    }

    // ==================== LazyTurnAction ====================

    /**
     * An Action that defers building the RoadRunner turnTo until its first run() call.
     *
     * RoadRunner's turnTo pre-computes a motion profile at build() time, baking in the
     * expected start heading. LazyTurnAction waits until it actually runs to read the
     * robot's real pose and build an accurate turn.
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
                Pose2d currentPose = driveTrain.getPose();
                double headingError = Math.toDegrees(Math.abs(
                        normalizeAngle(targetHeadingRad - currentPose.heading.toDouble())));

                if (headingError <= skipToleranceDeg) {
                    complete = true;
                    return false;
                }

                innerAction = driveTrain.actionBuilder(currentPose)
                        .turnTo(targetHeadingRad)
                        .build();
            }

            return innerAction.run(packet);
        }
    }

    // ==================== LazyBearingTurnAction ====================

    /**
     * An Action that computes the bearing to a target position at runtime, then
     * builds and executes an RR turnTo to face that bearing.
     *
     * Unlike LazyTurnAction (which takes a fixed target heading), this computes
     * the heading from the robot's actual position at execution time. This is
     * critical because the robot may not be exactly where it was when the action
     * sequence was built.
     *
     * For reversed driving, the robot faces away from the target (bearing + PI).
     */
    private class LazyBearingTurnAction implements Action {
        private final Vector2d targetPosition;
        private final boolean reversed;
        private final double skipToleranceDeg;
        private Action innerAction = null;
        private boolean complete = false;

        LazyBearingTurnAction(Vector2d targetPosition, boolean reversed, double skipToleranceDeg) {
            this.targetPosition = targetPosition;
            this.reversed = reversed;
            this.skipToleranceDeg = skipToleranceDeg;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (complete) return false;

            if (innerAction == null) {
                Pose2d currentPose = driveTrain.getPose();
                double bearing = bearingTo(currentPose.position, targetPosition);
                double targetHeading = reversed ? normalizeAngle(bearing + Math.PI) : bearing;

                double headingError = Math.toDegrees(Math.abs(
                        normalizeAngle(targetHeading - currentPose.heading.toDouble())));

                System.out.println(String.format(
                        "BearingTurn INIT: pos=(%.1f,%.1f) hdg=%.1f° bearing=%.1f° target=%.1f° err=%.1f° tgt=(%.1f,%.1f) rev=%b skip=%b",
                        currentPose.position.x, currentPose.position.y,
                        Math.toDegrees(currentPose.heading.toDouble()),
                        Math.toDegrees(bearing),
                        Math.toDegrees(targetHeading),
                        headingError,
                        targetPosition.x, targetPosition.y,
                        reversed,
                        headingError <= skipToleranceDeg));

                if (headingError <= skipToleranceDeg) {
                    complete = true;
                    return false;
                }

                innerAction = driveTrain.actionBuilder(currentPose)
                        .turnTo(targetHeading)
                        .build();
            }

            return innerAction.run(packet);
        }
    }

    // ==================== HELPER METHODS ====================

    /**
     * Calculate bearing (angle) from one position to another.
     */
    private double bearingTo(Vector2d from, Vector2d to) {
        return Math.atan2(to.y - from.y, to.x - from.x);
    }

    /**
     * Normalize angle to [-PI, PI] range.
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
