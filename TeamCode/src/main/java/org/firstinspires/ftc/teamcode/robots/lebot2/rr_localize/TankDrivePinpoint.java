package org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize;

//import static org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.PinpointLocalizer.goBILDA_4_BAR_POD;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.RamseteController;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.robots.lebot2.FieldMap;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.drivetrain.DriveTrainBase;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Vision;
import org.firstinspires.ftc.teamcode.rrQuickStart.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.rrQuickStart.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.rrQuickStart.messages.TankCommandMessage;
import org.firstinspires.ftc.teamcode.rrQuickStart.Drawing;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

/**
 * TankDrive using Pinpoint odometry, implementing DriveTrainBase for Subsystem integration.
 *
 * This class serves dual purposes:
 * 1. RoadRunner trajectory following (Actions, kinematics, feedforward)
 * 2. Subsystem integration (three-phase updates, teleop driving, PID turns)
 *
 * THREE-PHASE UPDATE:
 * - readSensors(): Refresh Pinpoint localizer (I2C bulk read)
 * - calc(): Update pose estimate, run turn PID if active
 * - act(): No-op (RoadRunner Actions write motors directly)
 *
 * TUNING COMPATIBILITY:
 * Public fields (leftMotors, rightMotors, localizer, PARAMS) are preserved
 * for TuningOpModes compatibility.
 */
@Config(value = "Lebot2_TankDrivePinpoint")
public final class TankDrivePinpoint implements DriveTrainBase {

    // ==================== ROADRUNNER PARAMS (for tuning compatibility) ====================

    public static class Params {
        // drive model parameters
        public double inPerTick = 1/(19.89436789f * 25.4);  //23.75/11995; // Pinpoint reports in inches, so 1:1 mapping
        public double trackWidthTicks = 4777.861530266601; // (unused legacy)
        public double trackWidthInches = 14.0; // Track width for TankKinematics - tune via Dashboard

        // feedforward parameters (in tick units)
        public double kS = 1.164863007541458;
        public double kV = 0.0003462964757510799;
        public double kA = .0001;

        // path profile parameters (in inches)
        public double maxWheelVel = 25;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double ramseteZeta = 0.7; // in the range (0, 1)
        public double ramseteBBar = 2.0; // positive

        // turn controller gains
        public double turnGain = 25.0;
        public double turnVelGain = 3.0;
        public double turnIGain = 1.0;           // Integral gain for steady-state accuracy
        public double turnICutIn = 5.0;         // Only integrate when error < this (degrees)
        public double turnFeedforwardScale = 0.0; // Scale feedforward (0=pure feedback, 1=full feedforward)

        // turn completion (position-based after profile ends)
        public double turnCompleteTolerance = 1.5;      // Heading error tolerance (degrees)
        public double turnCompleteVelTolerance = 0.2;   // Angular velocity tolerance (rad/s)
        public double turnCompleteTimeout = 2.0;        // Max seconds to settle after profile (safety)
    }

    public static Params PARAMS = new Params();

    // ==================== TURN PID PARAMS ====================
    // TODO: These PID constants need tuning for vision-based targeting.
    // Current P is too weak for small errors - needs higher P or proper I term.
    // The I=0.04 exists but may need adjustment for steady-state error.

    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0.03, 0.04, 0.0);
    public static double HEADING_TOLERANCE = 1.5; // degrees

    // ==================== VISION PID PARAMS ====================
    public static PIDCoefficients VISION_PID = new PIDCoefficients(0.022, 0.01, 0.013);
    public static double VISION_OFFSET = -2; // offset from center of target in LLResult x units
    public static double VISION_TOLERANCE = 3; // degrees of tx
    public static double VISION_INTEGRAL_CUTIN = 3.0; // degrees
    public static double VISION_ALPHA = .5; // EMA alpha for vision PID

    // Distance tracking PID (for maintaining distance to target)
    public static PIDCoefficients DISTANCE_PID = new PIDCoefficients(0.02, 0.0, 0.01);
    public static double DISTANCE_TOLERANCE = 2.0; // inches
    public static double TARGET_DISTANCE_INCHES = 30.0; // Target distance to maintain
    public static double DISTANCE_MAX_SPEED = 0.4; // Max speed for distance control


    // ==================== ROADRUNNER INFRASTRUCTURE ====================

    public TankKinematics kinematics;  // Non-final to allow Dashboard tuning of trackWidthInches
    private double lastTrackWidth = 0;  // Track changes to PARAMS.trackWidthInches

    public final TurnConstraints defaultTurnConstraints;
    public final VelConstraint defaultVelConstraint;
    public final AccelConstraint defaultAccelConstraint;

    // Public for TuningOpModes compatibility
    public final List<DcMotorEx> leftMotors, rightMotors;
    public final VoltageSensor voltageSensor;
    public final Localizer localizer;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter tankCommandWriter = new DownsampledWriter("TANK_COMMAND", 50_000_000);

    // ==================== SUBSYSTEM STATE ====================

    // Drive behavior - what is currently controlling the drive motors
    // Set implicitly by control methods (drive, runAction, turnToHeading)
    public enum Behavior {
        MANUAL,     // Joystick control (default)
        TRAJECTORY, // RoadRunner trajectory following
        PID_TURN    // PID-based turn to heading/target
    }
    private Behavior behavior = Behavior.MANUAL;

    // Turn state machine (used when behavior == PID_TURN)
    public enum TurnState {
        IDLE,
        TURNING_TO_HEADING,
        CENTERING_ON_TARGET,    // Continuous tx from Vision (heading only)
        TRACKING_TARGET         // Combined heading centering + distance control
    }
    private TurnState turnState = TurnState.IDLE;
    private double turnTarget = 0;
    private double turnMaxSpeed = 1.0;

    // Vision reference for continuous target tracking
    private Vision vision = null;
    public static double CENTERING_MAX_SPEED = 0.5; // Default max speed for centering

    // RoadRunner action tracking
    private Action currentAction = null;
    private Vector2d currentActionTarget = null;  // Cached target for visualization
    private List<Vector2d> trajectoryWaypoints = new ArrayList<>();  // All waypoints for visualization

    // PID controllers for turns and tracking
    private final PIDController headingPID;
    private final PIDController visionPID;
    private final PIDController distancePID;

    // Cached values from readSensors()
    private double cachedHeading = 0;
    private Pose2d cachedPose = new Pose2d(0, 0, 0);

    // ==================== CONSTRUCTOR ====================

    public TankDrivePinpoint(HardwareMap hardwareMap, Pose2d pose) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        // NOTE: Bulk caching mode is NOT set here - Robot.java handles it
        // This allows TuningOpModes to set their own mode if needed

        // Initialize kinematics with track width
        // Track width tunable via Dashboard (PARAMS.trackWidthInches)
        kinematics = new TankKinematics(PARAMS.trackWidthInches);
        lastTrackWidth = PARAMS.trackWidthInches;
        // Initialize constraints
        defaultTurnConstraints = new TurnConstraints(
                PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
        defaultVelConstraint = new MinVelConstraint(Arrays.asList(
                kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                new AngularVelConstraint(PARAMS.maxAngVel)
        ));
        defaultAccelConstraint = new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

        // Lebot2 motor configuration - differential drive with rear-mounted motors
        leftMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "leftRear"));
        rightMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "rightRear"));

        for (DcMotorEx m : leftMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        for (DcMotorEx m : rightMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Lebot2: left motor reversed so positive power = forward
        rightMotors.get(0).setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Use Pinpoint for localization
        localizer = new PinpointLocalizer(hardwareMap, PARAMS.inPerTick, pose);

        // Initialize heading PID for turns
        headingPID = new PIDController(HEADING_PID);
        headingPID.setInputRange(-180, 180);
        headingPID.setOutputRange(-1, 1);
        headingPID.setIntegralCutIn(4);
        headingPID.setContinuous(true);
        headingPID.setTolerance(HEADING_TOLERANCE);
        headingPID.enable();

        // Initialize vision PID for target centering
        visionPID = new PIDController(VISION_PID);
        visionPID.setInputRange(-20, 20);
        visionPID.setOutputRange(-1, 1);
        visionPID.setIntegralCutIn(VISION_INTEGRAL_CUTIN);
        visionPID.setContinuous(false);
        visionPID.setTolerance(VISION_TOLERANCE/360*40); // degrees to percentage of input range
        visionPID.setEmaAlpha(VISION_ALPHA);
        visionPID.enable();

        // Initialize distance PID for target distance tracking
        distancePID = new PIDController(DISTANCE_PID);
        distancePID.setInputRange(0, 100);  // inches
        distancePID.setOutputRange(-DISTANCE_MAX_SPEED, DISTANCE_MAX_SPEED);
        distancePID.setContinuous(false);
        distancePID.setTolerance(DISTANCE_TOLERANCE / 100.0);  // percentage of input range
        distancePID.enable();

        cachedPose = pose;
        cachedHeading = Math.toDegrees(pose.heading.toDouble());

        FlightRecorder.write("TANK_PINPOINT_PARAMS", PARAMS);
    }

    // ==================== THREE-PHASE SUBSYSTEM METHODS ====================

    @Override
    public void readSensors() {
        // PHASE 1: Refresh Pinpoint localizer (triggers I2C bulk read)
        if (localizer instanceof PinpointLocalizer) {
            ((PinpointLocalizer) localizer).refresh();
        }

        // Check if trackWidthInches changed via Dashboard - rebuild kinematics if so
        if (PARAMS.trackWidthInches != lastTrackWidth) {
            kinematics = new TankKinematics(PARAMS.trackWidthInches);
            lastTrackWidth = PARAMS.trackWidthInches;
        }
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        // PHASE 2: Update pose estimate and run turn state machine

        // Get pose from localizer (uses cached Pinpoint data)
        PoseVelocity2d vel = localizer.update();
        cachedPose = localizer.getPose();
        cachedHeading = Math.toDegrees(cachedPose.heading.toDouble());

        // Update pose history for visualization
        poseHistory.add(cachedPose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }
        estimatedPoseWriter.write(new PoseMessage(cachedPose));

        // Service RoadRunner actions (for turnToHeading and other trajectory-based turns)
        if (behavior == Behavior.TRAJECTORY && currentAction != null) {
            TelemetryPacket packet = new TelemetryPacket();
            updateAction(packet);
        }

        // Run turn state machine (for legacy PID turns and vision centering)
        switch (turnState) {
            case IDLE:
                // Nothing to do
                break;

            case TURNING_TO_HEADING:
                executeTurnToHeading();
                break;

            case CENTERING_ON_TARGET:
                executeCenteringOnTarget();
                break;

            case TRACKING_TARGET:
                executeTrackingTarget();
                break;
        }

        // Draw robot and field elements on overlay if provided
        if (fieldOverlay != null) {
            // Draw field waypoints (optional, controlled by FieldMap.DRAW_WAYPOINTS)
            FieldMap.drawWaypoints(fieldOverlay);

            // Draw trajectory waypoints for spline visualization
            if (!trajectoryWaypoints.isEmpty()) {
                // Draw intermediate waypoints in yellow
                for (int i = 0; i < trajectoryWaypoints.size() - 1; i++) {
                    Vector2d wp = trajectoryWaypoints.get(i);
                    fieldOverlay.setFill("#FFFF0080");  // Yellow with 50% alpha
                    fieldOverlay.setStroke("#FFFF00");  // Yellow outline
                    fieldOverlay.setStrokeWidth(2);
                    fieldOverlay.fillCircle(wp.x, wp.y, FieldMap.WAYPOINT_RADIUS * 0.75);
                    fieldOverlay.strokeCircle(wp.x, wp.y, FieldMap.WAYPOINT_RADIUS * 0.75);
                }
                // Draw final target in green
                Vector2d finalWp = trajectoryWaypoints.get(trajectoryWaypoints.size() - 1);
                fieldOverlay.setFill("#00FF0080");  // Green with 50% alpha
                fieldOverlay.setStroke("#00FF00");  // Green outline
                fieldOverlay.setStrokeWidth(2);
                fieldOverlay.fillCircle(finalWp.x, finalWp.y, FieldMap.WAYPOINT_RADIUS);
                fieldOverlay.strokeCircle(finalWp.x, finalWp.y, FieldMap.WAYPOINT_RADIUS);
            } else if (currentActionTarget != null) {
                // Fallback: draw single target in green (for non-spline trajectories)
                fieldOverlay.setFill("#00FF0080");  // Green with 50% alpha
                fieldOverlay.setStroke("#00FF00");  // Green outline
                fieldOverlay.setStrokeWidth(2);
                fieldOverlay.fillCircle(currentActionTarget.x, currentActionTarget.y, FieldMap.WAYPOINT_RADIUS);
                fieldOverlay.strokeCircle(currentActionTarget.x, currentActionTarget.y, FieldMap.WAYPOINT_RADIUS);
            }

            drawPoseHistory(fieldOverlay);
            fieldOverlay.setStroke("#3F51B5");
            Drawing.drawRobot(fieldOverlay, cachedPose);
        }
    }

    @Override
    public void act() {
        // PHASE 3: No-op for drive motors
        // RoadRunner Actions write motors directly during trajectory following
        // Turn PID writes motors in calc() for immediate response
    }

    // ==================== TURN METHODS ====================

    private void executeTurnToHeading() {
        headingPID.setInput(cachedHeading);
        headingPID.setSetpoint(turnTarget);
        headingPID.setOutputRange(-turnMaxSpeed, turnMaxSpeed);
        headingPID.setPID(HEADING_PID);

        double correction = headingPID.performPID();

        if (headingPID.onTarget()) {
            setMotorPowers(0, 0);
            turnState = TurnState.IDLE;
            behavior = Behavior.MANUAL;
        } else {
            // Apply turn power (positive = clockwise)
            setMotorPowers(correction, -correction);
        }
    }

    @Override
    public void turnToHeading(double headingDegrees, double maxSpeed) {
        // Use RoadRunner's turnTo action for accurate, well-tuned turns
        Action turnAction = actionBuilder(cachedPose)
                .turnTo(Math.toRadians(headingDegrees))
                .build();
        runAction(turnAction);
    }

    /**
     * Legacy PID-based turn to heading. Kept for reference/testing.
     * Note: PID constants are poorly tuned - prefer turnToHeading() which uses RoadRunner.
     */
    public void turnToHeadingPID(double headingDegrees, double maxSpeed) {
        turnTarget = headingDegrees;
        turnMaxSpeed = maxSpeed;
        turnState = TurnState.TURNING_TO_HEADING;
        behavior = Behavior.PID_TURN;
        headingPID.enable();
    }

    @Override
    public void turnToTarget(double tx, double maxSpeed) {
        // Deprecated — use centerOnTarget() for vision-based aiming
        centerOnTarget();
    }

    @Override
    public boolean isTurnComplete() {
        // For PID turns: check turnState
        // For RoadRunner turns: check if action completed (behavior back to MANUAL)
        return turnState == TurnState.IDLE && behavior != Behavior.TRAJECTORY;
    }

    @Override
    public void cancelTurn() {
        turnState = TurnState.IDLE;
        if (behavior == Behavior.PID_TURN) {
            behavior = Behavior.MANUAL;
        }
        setMotorPowers(0, 0);
    }

    @Override
    public void setVision(Vision vision) {
        this.vision = vision;
    }

    @Override
    public void centerOnTarget() {
        // Only start if we have Vision reference and target is visible
        if (vision == null) {
            return; // No vision reference - can't center
        }
        if (!vision.hasTarget()) {
            return; // No target visible - do nothing
        }

        // Start centering - will query Vision each loop for fresh tx
        turnMaxSpeed = CENTERING_MAX_SPEED;
        turnState = TurnState.CENTERING_ON_TARGET;
        behavior = Behavior.PID_TURN;
        visionPID.enable();
    }

    private void executeCenteringOnTarget() {
        // Query Vision for fresh tx each loop
        if (vision == null || !vision.hasTarget()) {
            // Lost target or no vision - stop and return to manual
            setMotorPowers(0, 0);
            turnState = TurnState.IDLE;
            behavior = Behavior.MANUAL;
            return;
        }

        double tx = vision.getTx();

        // Negate tx so positive tx (target right) produces positive correction (turn right)
        visionPID.setInput(tx);
        visionPID.setSetpoint(VISION_OFFSET);
        visionPID.setOutputRange(-turnMaxSpeed, turnMaxSpeed);
        visionPID.setPID(VISION_PID);

        double correction = visionPID.performPID();

        if (visionPID.lockedOnTarget()) {
            setMotorPowers(0, 0);
            turnState = TurnState.IDLE;
            behavior = Behavior.MANUAL;
        } else {
            setMotorPowers(-correction, correction);
        }
    }

    /**
     * Start combined target tracking - centers on target AND maintains distance.
     * Uses vision tx for heading control and target area for distance estimation.
     * Never self-terminates - use stopTracking() to end.
     */
    public void trackTarget() {
        if (vision == null) {
            return;
        }

        turnState = TurnState.TRACKING_TARGET;
        behavior = Behavior.PID_TURN;
        visionPID.enable();
        distancePID.enable();
    }

    /**
     * Stop target tracking and return to manual control.
     */
    public void stopTracking() {
        if (turnState == TurnState.TRACKING_TARGET) {
            setMotorPowers(0, 0);
            turnState = TurnState.IDLE;
            behavior = Behavior.MANUAL;
        }
    }

    /**
     * Check if currently tracking a target.
     */
    public boolean isTracking() {
        return turnState == TurnState.TRACKING_TARGET;
    }

    private void executeTrackingTarget() {
        // Check if we still have a target
        if (vision == null || !vision.hasTarget()) {
            // Lost target - stop motors but stay in tracking mode (will resume when target reappears)
            setMotorPowers(0, 0);
            return;
        }

        // --- Heading control (same as centerOnTarget) ---
        double tx = vision.getTx();
        visionPID.setInput(tx);
        visionPID.setSetpoint(VISION_OFFSET);
        visionPID.setOutputRange(-CENTERING_MAX_SPEED, CENTERING_MAX_SPEED);
        visionPID.setPID(VISION_PID);
        double turnCorrection = visionPID.performPID();

        // --- Distance control ---
        double currentDistance = vision.getTargetDistanceInches();
        distancePID.setInput(currentDistance);
        distancePID.setSetpoint(TARGET_DISTANCE_INCHES);
        distancePID.setOutputRange(-DISTANCE_MAX_SPEED, DISTANCE_MAX_SPEED);
        distancePID.setPID(DISTANCE_PID);
        double driveCorrection = distancePID.performPID();

        // Combine: driveCorrection for forward/back, turnCorrection for differential
        // Positive driveCorrection = too far, need to drive forward
        // Negative driveCorrection = too close, need to drive backward
        double leftPower = driveCorrection - turnCorrection;
        double rightPower = driveCorrection + turnCorrection;

        // Clamp to [-1, 1]
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        setMotorPowers(leftPower, rightPower);
    }

    // ==================== TELEOP DRIVING ====================

    @Override
    public void drive(double throttle, double strafe, double turn) {
        boolean separate = false;
        if(separate){
            setMotorPowers(throttle, strafe);
            return;
        }else {

            // Tank drive ignores strafe

            // Check if joystick input should interrupt current mode
            boolean hasInput = Math.abs(throttle) > 0.1 || Math.abs(turn) > 0.1;

            if (hasInput) {
                // Joystick input interrupts RR_ACTION and PID_TURN
                if (behavior == Behavior.TRAJECTORY) {
                    cancelAction();
                }
                if (behavior == Behavior.PID_TURN) {
                    cancelTurn();
                }
                behavior = Behavior.MANUAL;
            }

            // Only apply joystick input in MANUAL mode
            if (behavior != Behavior.MANUAL) {
                return;
            }

            double leftPower = throttle + turn;
            double rightPower = throttle - turn;

            // Normalize if over 1.0
            double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }

            setMotorPowers(leftPower, rightPower);
        }
    }

    // ==================== BEHAVIOR MANAGEMENT ====================

    /**
     * Get current drive behavior.
     */
    public Behavior getBehavior() {
        return behavior;
    }

    /**
     * Check if drive is in manual mode.
     */
    public boolean isManualMode() {
        return behavior == Behavior.MANUAL;
    }

    /**
     * Check if a RoadRunner action is currently running.
     */
    public boolean isActionRunning() {
        return behavior == Behavior.TRAJECTORY && currentAction != null;
    }

    /**
     * Run a RoadRunner action. Sets behavior to TRAJECTORY.
     *
     * @param action The action to run
     */
    public void runAction(Action action) {
        currentAction = action;
        behavior = Behavior.TRAJECTORY;

        // Cache the target position for visualization
        currentActionTarget = extractTargetFromAction(action);
    }

    /**
     * Run a RoadRunner action with an explicit target position for visualization.
     * Use this instead of runAction(Action) when you know the target position
     * (e.g., from TankDriveActions.getLastTargetPosition()) to avoid relying on
     * extraction from the action tree.
     *
     * @param action The action to run
     * @param targetPosition The target position for the green disk visualization
     */
    public void runAction(Action action, Vector2d targetPosition) {
        currentAction = action;
        behavior = Behavior.TRAJECTORY;
        currentActionTarget = targetPosition;
        trajectoryWaypoints.clear();
    }

    /**
     * Run a RoadRunner action with multiple waypoints for visualization.
     * Use this for spline trajectories where you want to see all intermediate targets.
     *
     * @param action The action to run
     * @param waypoints All waypoints along the trajectory (intermediate, row start, row end)
     */
    public void runAction(Action action, List<Vector2d> waypoints) {
        currentAction = action;
        behavior = Behavior.TRAJECTORY;
        trajectoryWaypoints.clear();
        trajectoryWaypoints.addAll(waypoints);
        // Set currentActionTarget to the final waypoint for backward compatibility
        if (!waypoints.isEmpty()) {
            currentActionTarget = waypoints.get(waypoints.size() - 1);
        }
    }

    /**
     * Extract the end position from a RoadRunner action for visualization.
     * Handles FollowTrajectoryAction, TurnAction, and SequentialAction (recursively).
     */
    private Vector2d extractTargetFromAction(Action action) {
        if (action instanceof FollowTrajectoryAction) {
            FollowTrajectoryAction followAction = (FollowTrajectoryAction) action;
            double pathLength = followAction.timeTrajectory.path.length();
            Pose2d endPose = followAction.timeTrajectory.path.get(pathLength, 1).value();
            return endPose.position;
        }
        if (action instanceof TurnAction) {
            TurnAction turnAction = (TurnAction) action;
            return turnAction.turn.beginPose.position;
        }
        // Handle SequentialAction - find the last trajectory/turn action
        if (action instanceof com.acmerobotics.roadrunner.SequentialAction) {
            com.acmerobotics.roadrunner.SequentialAction seqAction =
                    (com.acmerobotics.roadrunner.SequentialAction) action;
            // SequentialAction has a public 'actions' field (List<Action>)
            java.util.List<Action> actions = seqAction.getInitialActions();
            // Find the last action that has a position target
            for (int i = actions.size() - 1; i >= 0; i--) {
                Vector2d target = extractTargetFromAction(actions.get(i));
                if (target != null) {
                    return target;
                }
            }
        }
        return null;
    }

    /**
     * Cancel any running RoadRunner action.
     */
    public void cancelAction() {
        currentAction = null;
        currentActionTarget = null;
        trajectoryWaypoints.clear();
        if (behavior == Behavior.TRAJECTORY) {
            behavior = Behavior.MANUAL;
            setMotorPowers(0, 0);
        }
    }

    /**
     * Update the current RoadRunner action.
     * Call this in the main loop when an action is running.
     *
     * @param packet TelemetryPacket for logging
     * @return true if action is still running, false if complete
     */
    public boolean updateAction(TelemetryPacket packet) {
        if (currentAction == null) {
            return false;
        }

        boolean running = currentAction.run(packet);
        if (!running) {
            currentAction = null;
            currentActionTarget = null;
            trajectoryWaypoints.clear();
            behavior = Behavior.MANUAL;
        }
        return running;
    }

    /**
     * Set drive powers using RoadRunner's PoseVelocity2d format.
     * Used by RoadRunner Actions during trajectory following.
     */
    public void setDrivePowers(PoseVelocity2d powers) {
        TankKinematics.WheelVelocities<Time> wheelVels = new TankKinematics(2).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        for (DcMotorEx m : leftMotors) {
            m.setPower(wheelVels.left.get(0) / maxPowerMag);
        }
        for (DcMotorEx m : rightMotors) {
            m.setPower(wheelVels.right.get(0) / maxPowerMag);
        }
    }

    void setMotorPowers(double left, double right) {
        for (DcMotorEx m : leftMotors) {
            m.setPower(left);
        }
        for (DcMotorEx m : rightMotors) {
            m.setPower(right);
        }
    }

    // ==================== POSE METHODS ====================

    @Override
    public Pose2d getPose() {
        return cachedPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        if (localizer instanceof PinpointLocalizer) {
            ((PinpointLocalizer) localizer).setPose(pose);
        }
        cachedPose = pose;
        cachedHeading = Math.toDegrees(pose.heading.toDouble());
    }

    @Override
    public void setPose(Object position) {
        // TODO: Convert position constant to Pose2d and set
        // This will be implemented when we have the Constants class
    }

    @Override
    public double getHeadingDegrees() {
        return cachedHeading;
    }

    @Override
    public int getLeftTicks() {
        return leftMotors.get(0).getCurrentPosition();
    }

    @Override
    public void resetEncoders() {
        for (DcMotorEx m : leftMotors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        for (DcMotorEx m : rightMotors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // ==================== ROADRUNNER POSE ESTIMATE ====================

    /**
     * Update pose estimate from localizer.
     * Called by RoadRunner Actions during trajectory following.
     * Also called by standalone tuning opmodes like LocalizationTest.
     */
    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = localizer.update();
        cachedPose = localizer.getPose();
        cachedHeading = Math.toDegrees(cachedPose.heading.toDouble());

        poseHistory.add(cachedPose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(cachedPose));

        return vel;
    }

    // ==================== ROADRUNNER ACTIONS ====================

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                setMotorPowers(0, 0);
                return false;
            }

            DualNum<Time> x = timeTrajectory.profile.get(t);

            Pose2dDual<Arclength> txWorldTarget = timeTrajectory.path.get(x.value(), 3);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new RamseteController(kinematics.trackWidth, PARAMS.ramseteZeta, PARAMS.ramseteBBar)
                    .compute(x, txWorldTarget, localizer.getPose());
            driveCommandWriter.write(new DriveCommandMessage(command));

            TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftPower = feedforward.compute(wheelVels.left) / voltage;
            double rightPower = feedforward.compute(wheelVels.right) / voltage;
            tankCommandWriter.write(new TankCommandMessage(voltage, leftPower, rightPower));

            setMotorPowers(leftPower, rightPower);

            p.put("x", localizer.getPose().position.x);
            p.put("y", localizer.getPose().position.y);
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(localizer.getPose());
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;
        private double lastTs = -1;
        private double turnIntegral = 0.0;  // Accumulated heading error for I term
        private double settlingStartTs = -1;  // When settling phase started
        private Rotation2d finalTargetHeading = null;  // Target heading after profile ends

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double now = Actions.now();
            double t;
            double dt;
            if (beginTs < 0) {
                beginTs = now;
                lastTs = now;
                t = 0;
                dt = 0;
                turnIntegral = 0;  // Reset integral at start of turn
            } else {
                t = now - beginTs;
                dt = now - lastTs;
                lastTs = now;
            }

            PoseVelocity2d robotVelRobot = updatePoseEstimate();
            Rotation2d currentHeading = localizer.getPose().heading;

            // Determine target heading and feedforward velocity
            Rotation2d targetHeading;
            double ffVel;
            Pose2d targetPose;  // For visualization
            boolean inSettlingPhase = (t >= turn.duration);

            if (inSettlingPhase) {
                // Settling phase: profile done, hold final target
                if (finalTargetHeading == null) {
                    // Capture final target heading from end of profile
                    finalTargetHeading = turn.get(turn.duration).heading.value();
                    settlingStartTs = now;
                }
                targetHeading = finalTargetHeading;
                ffVel = 0;  // No feedforward during settling
                targetPose = new Pose2d(turn.beginPose.position, targetHeading);

                // Check completion conditions
                double headingErrorDeg = Math.toDegrees(targetHeading.minus(currentHeading));
                double settlingTime = now - settlingStartTs;

                boolean positionSettled = Math.abs(headingErrorDeg) < PARAMS.turnCompleteTolerance;
                boolean velocitySettled = Math.abs(robotVelRobot.angVel) < PARAMS.turnCompleteVelTolerance;
                boolean timedOut = settlingTime > PARAMS.turnCompleteTimeout;

                if ((positionSettled && velocitySettled) || timedOut) {
                    setMotorPowers(0, 0);
                    return false;
                }
            } else {
                // Profile phase: follow moving target
                Pose2dDual<Time> txWorldTarget = turn.get(t);
                targetPose = txWorldTarget.value();
                targetPoseWriter.write(new PoseMessage(targetPose));
                targetHeading = txWorldTarget.heading.value();
                ffVel = txWorldTarget.heading.velocity().value();
            }

            // Calculate heading error (radians)
            double headingError = targetHeading.minus(currentHeading);
            double headingErrorDeg = Math.toDegrees(headingError);

            // Accumulate integral only when error is below cut-in threshold
            if (Math.abs(headingErrorDeg) < PARAMS.turnICutIn) {
                turnIntegral += headingError * dt;
            } else {
                turnIntegral = 0;  // Reset if error grows beyond cut-in (anti-windup)
            }

            // Velocity error (target is 0 during settling)
            double velError = (inSettlingPhase ? 0 : ffVel) - robotVelRobot.angVel;

            // Scaled feedforward + PID feedback
            double ffTerm = ffVel * PARAMS.turnFeedforwardScale;
            double pTerm = PARAMS.turnGain * headingError;
            double iTerm = PARAMS.turnIGain * turnIntegral;
            double dTerm = PARAMS.turnVelGain * velError;

            PoseVelocity2dDual<Time> command = new PoseVelocity2dDual<>(
                    Vector2dDual.constant(new Vector2d(0, 0), 3),
                    DualNum.constant(ffTerm + pTerm + iTerm + dTerm, 3)
            );
            driveCommandWriter.write(new DriveCommandMessage(command));

            TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftPower = feedforward.compute(wheelVels.left) / voltage;
            double rightPower = feedforward.compute(wheelVels.right) / voltage;
            tankCommandWriter.write(new TankCommandMessage(voltage, leftPower, rightPower));

            setMotorPowers(leftPower, rightPower);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, targetPose);

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;
            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    // ==================== SUBSYSTEM LIFECYCLE ====================

    @Override
    public void stop() {
        setMotorPowers(0, 0);
        turnState = TurnState.IDLE;
        currentAction = null;
        behavior = Behavior.MANUAL;
    }

    @Override
    public void resetStates() {
        turnState = TurnState.IDLE;
        currentAction = null;
        behavior = Behavior.MANUAL;
    }

    // ==================== TELEMETRY ====================

    @Override
    public String getTelemetryName() {
        return "TankDrivePinpoint";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        // Position in inches and meters for easy comparison with Vision
        double xIn = cachedPose.position.x;
        double yIn = cachedPose.position.y;
        double xM = xIn / 39.3701;
        double yM = yIn / 39.3701;

        telemetry.put("Pose (in)", String.format("(%.1f, %.1f)", xIn, yIn));
        telemetry.put("Pose (m)", String.format("(%.2f, %.2f)", xM, yM));
        telemetry.put("Heading", String.format("%.1f°", cachedHeading));
        telemetry.put("Drive Mode", behavior);

        if (debug) {
            telemetry.put("Turn State", turnState);
            telemetry.put("Turn Target", String.format("%.1f", turnTarget));
            telemetry.put("Heading PID Error", String.format("%.2f", headingPID.getError()));
            telemetry.put("Vision PID Error", String.format("%.2f", visionPID.getError()));
            telemetry.put("Left Power", String.format("%.2f", leftMotors.get(0).getPower()));
            telemetry.put("Right Power", String.format("%.2f", rightMotors.get(0).getPower()));
            telemetry.put("Action Running", currentAction != null);
        }

        return telemetry;
    }
}
