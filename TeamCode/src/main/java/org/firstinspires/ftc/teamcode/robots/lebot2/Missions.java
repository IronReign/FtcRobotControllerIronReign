package org.firstinspires.ftc.teamcode.robots.lebot2;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.CompositeVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.TankDriveActions;
import org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.TankDrivePinpoint;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Launcher;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Vision;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.util.CsvLogKeeper;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

/**
 * Missions - Layer 4 autonomous mission coordinator.
 *
 * Missions are composite autonomous behaviors that coordinate Robot behaviors,
 * subsystem behaviors, and RoadRunner trajectories to accomplish high-level goals.
 *
 * ARCHITECTURE:
 * - Missions sit above Robot.Behavior (Layer 3) and below Autonomous strategy
 * - They coordinate through Robot behaviors, not directly controlling hardware
 * - Each mission is a state machine that runs non-blocking
 * - Usable from both Autonomous and TeleOp
 *
 * AVAILABLE MISSIONS:
 * - LAUNCH_PRELOADS: Fire any balls loaded at start (delegates to Robot.LAUNCH_ALL)
 * - BALL_GROUP: Navigate to a ball group, intake balls
 * - OPEN_SESAME: Navigate to classifier release pose, back into lever
 * - GO_BALL_CLUSTER: (future) Vision-based ball cluster pickup
 *
 * USAGE:
 * - Call start*() to begin a mission
 * - Robot.update() calls missions.calc() when active
 * - Check isComplete() or isActive() for status
 * - Call abort() for emergency stop
 */
@Config(value = "Lebot2_Missions")
public class Missions implements TelemetryProvider {

    // Reference to robot (for accessing subsystems and behaviors)
    private final Robot robot;

    // Turn-Spline-Turn action builder for tank drive navigation
    private TankDriveActions actions;

    // ==================== MISSION ENUMS ====================

    public enum Mission {
        NONE,               // No mission active
        NAVIGATE_TO_FIRE,   // Navigate to fire position (can be reversed)
        LAUNCH_PRELOADS,    // Fire preloaded balls
        BALL_GROUP,         // Navigate to ball row, intake through row
        OPEN_SESAME,        // Navigate to gate, back into release lever
        GO_BALL_CLUSTER,    // (future) Vision/statistical ball pickup
        // Tuning missions (for TEST mode)
        TUNING_ROTATION,    // Turn 90° CW 4x, CCW 4x, report heading error
        TUNING_SQUARE,      // Drive 24", turn 90° CW, repeat 4x, report position error
        TUNING_STRAIGHT,    // Drive 48" forward, back, report error
        TUNING_TURN,        // Single turns at various angles
        TUNING_RAMSETE,         // Trajectory following with deliberate heading disturbance
        TUNING_STRAIGHT_POS,    // Drive 48" forward, back using PositionDriveAction
        TUNING_SQUARE_POS,      // Drive 24", turn 90° CW, repeat 4x using PositionDriveAction
        TUNING_DRIFT,           // In-place turns to measure Pinpoint position drift
        TUNING_VISION,          // Continuous vision centering (never self-terminates)
        CHECK_HEALTH            // Pre-match health check
    }

    public enum MissionState {
        IDLE,       // No mission or mission not started
        RUNNING,    // Mission in progress
        COMPLETE,   // Mission finished successfully
        FAILED,     // Mission encountered an error (timeout, etc.)
        ABORTED     // Mission was manually aborted
    }

    // Current mission tracking
    private Mission currentMission = Mission.NONE;
    private MissionState missionState = MissionState.IDLE;

    // ==================== TIMEOUT CONFIGURATION ====================

    public static double NAVIGATION_TIMEOUT_SECONDS = 10.0;
    public static double INTAKE_TIMEOUT_SECONDS = 5.0;
    public static double LAUNCH_TIMEOUT_SECONDS = 8.0;
    public static double PRESS_TIMEOUT_SECONDS = 2.0;
    public static double INTAKE_DRIVE_POWER = 0.23;  // Max power while driving through ball rows

    private ElapsedTime missionTimer = new ElapsedTime();

    // ==================== BALL GROUP CONFIGURATION ====================

    // Order to approach ball groups (configurable during match setup)
    private int[] ballGroupOrder = {0, 1, 2};
    private int currentGroupIndex = 0;
    private int targetGroupIndex = 0;  // Which group we're currently collecting

    // How many balls to collect per group (typically 3)
    public static int BALLS_PER_GROUP = 3;

    // ==================== MISSION STATE MACHINES ====================

    // LaunchPreloads state
    private enum LaunchPreloadsState {
        IDLE,
        WAITING_FOR_LAUNCH,
        WAITING_DUMMY      // For dummy mode timing
    }
    private LaunchPreloadsState launchPreloadsState = LaunchPreloadsState.IDLE;
    private ElapsedTime launchDummyTimer = new ElapsedTime();

    // TODO: Set DUMMY_LAUNCH_MODE to false once the real launch behavior is fixed
    // (paddle/firing mechanism needs physical fixes before real launch works)
    public static boolean DUMMY_LAUNCH_MODE = false;
    public static double DUMMY_LAUNCH_DURATION = 2.0;  // seconds to wait in dummy mode

    // OpenSesame state
    private enum OpenSesameState {
        IDLE,
        NAVIGATING,
        PRESSING
    }
    private OpenSesameState openSesameState = OpenSesameState.IDLE;
    private ElapsedTime pressTimer = new ElapsedTime();
    public static double PRESS_DURATION_SECONDS = 0.5;

    // BallGroup state
    private enum BallGroupState {
        IDLE,
        NAVIGATING_TO_ROW_START,
        INTAKING_THROUGH_ROW,
        COMPLETE
    }
    private BallGroupState ballGroupState = BallGroupState.IDLE;
    private int ballCountAtStart = 0;
    private Pose2d targetRowEnd = null;  // End of current ball row

    // NavigateToFire state
    private enum NavigateToFireState {
        IDLE,
        NAVIGATING
    }
    private NavigateToFireState navToFireState = NavigateToFireState.IDLE;
    private boolean navReversed = false;  // Whether to drive in reverse
    private int targetFirePosition = 1;   // Which fire position (1-4)

    // RoadRunner action tracking
    private Action currentAction = null;

    // ==================== TUNING MISSION STATE MACHINES ====================

    // Tuning configuration (Dashboard tunable)
    public static double TUNING_DRIVE_DISTANCE = 24.0;      // inches
    public static double TUNING_PAUSE_SECONDS = 1.0;        // pause between moves
    public static double TUNING_STRAIGHT_DISTANCE = 48.0;   // inches for straight test
    public static int TUNING_ROTATION_REPS = 4;             // repetitions per direction

    // TuningRotation state - turns 90° CW 4x, then CCW 4x
    private enum TuningRotationState {
        IDLE,
        TURNING_CW,
        PAUSE_CW,
        TURNING_CCW,
        PAUSE_CCW,
        COMPLETE
    }
    private TuningRotationState tuningRotationState = TuningRotationState.IDLE;
    private int tuningRotationCount = 0;
    private Pose2d tuningStartPose = null;
    private ElapsedTime tuningPauseTimer = new ElapsedTime();

    // TuningSquare state - drive 24", turn 90° CW, repeat 4x
    private enum TuningSquareState {
        IDLE,
        DRIVING,
        TURNING,
        PAUSE,
        COMPLETE
    }
    private TuningSquareState tuningSquareState = TuningSquareState.IDLE;
    private int tuningSquareCount = 0;

    // TuningStraight state - drive forward, then back
    private enum TuningStraightState {
        IDLE,
        DRIVING_FORWARD,
        PAUSE_FORWARD,
        DRIVING_BACK,
        COMPLETE
    }
    private TuningStraightState tuningStraightState = TuningStraightState.IDLE;

    // TuningTurn state - single turns at various angles
    private enum TuningTurnState {
        IDLE,
        TURNING,
        PAUSE,
        COMPLETE
    }
    private TuningTurnState tuningTurnState = TuningTurnState.IDLE;
    private double[] tuningTurnAngles = {45, 90, 180, -90, -45};
    private int tuningTurnIndex = 0;

    // TuningRamsete state - trajectory following with deliberate heading disturbance
    private enum TuningRamseteState {
        IDLE,
        TURNING_TO_DISTURB,     // Deliberately turn off-heading
        PAUSE_BEFORE_DRIVE,     // Brief pause to stabilize
        DRIVING_TRAJECTORY,     // Run trajectory, observe Ramsete correction
        PAUSE_AFTER_DRIVE,      // Pause before return
        RETURNING_TO_START,     // Drive back to starting position
        COMPLETE
    }
    private TuningRamseteState tuningRamseteState = TuningRamseteState.IDLE;
    private int tuningRamseteRep = 0;
    private Pose2d tuningRamseteStartPose = null;
    private double tuningRamseteTargetHeading = 0;  // Where trajectory wants to go

    // Ramsete tuning parameters (Dashboard tunable)
    public static double RAMSETE_DISTURBANCE_ANGLE = 15.0;   // Degrees to turn off-heading
    public static double RAMSETE_DRIVE_DISTANCE = 48.0;      // Trajectory distance (inches)
    public static int RAMSETE_REPS = 3;                      // Number of repetitions

    // TuningStraightPos state - position-based straight drive test
    private enum TuningStraightPosState {
        IDLE,
        DRIVING_FORWARD,
        PAUSE_FORWARD,
        DRIVING_BACK,
        COMPLETE
    }
    private TuningStraightPosState tuningStraightPosState = TuningStraightPosState.IDLE;

    // TuningSquarePos state - position-based square drive test
    private enum TuningSquarePosState {
        IDLE,
        DRIVING,
        PAUSE,
        COMPLETE
    }
    private TuningSquarePosState tuningSquarePosState = TuningSquarePosState.IDLE;
    private int tuningSquarePosCount = 0;

    // TuningDrift state - in-place turns to measure Pinpoint position drift
    private enum TuningDriftState {
        IDLE,
        TURNING_CW,
        PAUSE_CW,
        TURNING_CCW,
        PAUSE_CCW,
        COMPLETE
    }
    private TuningDriftState tuningDriftState = TuningDriftState.IDLE;
    private int tuningDriftCount = 0;
    private Pose2d preTurnPose = null;          // Position just before each turn
    private double cumulativeDriftX = 0.0;
    private double cumulativeDriftY = 0.0;

    // Drift tuning parameters (Dashboard tunable)
    public static double DRIFT_TURN_ANGLE = 90.0;   // Degrees per turn
    public static int DRIFT_REPS = 4;                // Turns per direction (4x90° = 360°)

    // HealthCheck state - pre-match system verification
    private enum HealthCheckState {
        IDLE,
        CHECK_BATTERY,
        CHECK_PINPOINT,
        START_TURNS,
        WAITING_TURNS,
        EVALUATE_TURNS,
        START_FLYWHEEL,
        WAITING_FLYWHEEL,
        EVALUATE_FLYWHEEL,
        START_LAUNCH,
        WAITING_LAUNCH,
        EVALUATE_LAUNCH,
        WAITING_DRIVER_CONFIRM,
        START_INTAKE,
        WAITING_INTAKE,
        EVALUATE_INTAKE,
        CHECK_VISION,
        SHOW_RESULTS,
        COMPLETE
    }
    private HealthCheckState healthCheckState = HealthCheckState.IDLE;

    // Health check configuration (Dashboard tunable)
    public static double HEALTH_BATTERY_MIN = 12.8;
    public static double HEALTH_HEADING_TOLERANCE = 5.0;       // degrees after 4x90° turns
    public static double HEALTH_POSITION_TOLERANCE = 3.0;      // inches drift after turns
    public static double HEALTH_FLYWHEEL_SPEED = 750;          // deg/s reduced target
    public static double HEALTH_FLYWHEEL_AMP_DIFF = 0.5;       // max amp difference between motors
    public static double HEALTH_FLYWHEEL_ENCODER_RATIO = 0.7;  // min ratio of slower/faster encoder
    public static double HEALTH_INTAKE_MIN_AMPS = 0.1;         // minimum to confirm motor running
    public static double HEALTH_CHECK_TIMEOUT = 5.0;           // seconds per phase timeout

    // Health check result tracking
    private LinkedHashMap<String, Boolean> healthResults = new LinkedHashMap<>();
    private LinkedHashMap<String, String> healthDetails = new LinkedHashMap<>();
    private Pose2d healthPreTurnPose = null;
    private int healthTurnCount = 0;
    private double healthSavedLaunchSpeed = 0;
    private int healthFlywheelStartEncoder = 0;
    private int healthHelperStartEncoder = 0;
    private double healthFlywheelAmps = 0;
    private double healthHelperAmps = 0;
    private double healthBeltAmps = 0;
    private double healthIntakeAmps = 0;
    private ElapsedTime healthPhaseTimer = new ElapsedTime();
    private Gamepad healthGamepad = null;

    // ==================== LOGGING ====================

    public static boolean LOGGING_ENABLED = true;  // Dashboard tunable
    private CsvLogKeeper missionLog = null;
    private long logStartTime = 0;

    // ==================== CONSTRUCTOR ====================

    public Missions(Robot robot) {
        this.robot = robot;
        // Initialize Turn-Spline-Turn action builder
        if (robot.driveTrain instanceof TankDrivePinpoint) {
            this.actions = new TankDriveActions((TankDrivePinpoint) robot.driveTrain);
        }
    }

    /**
     * Initialize logging with timestamped filename.
     * Call once at autonomous init.
     */
    public void initLogging() {
        if (LOGGING_ENABLED && missionLog == null) {
            String timestamp = new SimpleDateFormat("yyMMddHHmm", Locale.US).format(new Date());
            missionLog = new CsvLogKeeper("mission_" + timestamp, 12,
                "elapsedMs,mission,missionState,subState,poseX,poseY,heading,targetX,targetY,targetHdg,event,detail");
            logStartTime = System.currentTimeMillis();
            log("LOG_INIT", null, null);
        }
    }

    /**
     * Close the log file.
     */
    public void closeLogging() {
        if (missionLog != null) {
            log("LOG_CLOSE", null, null);
            missionLog.CloseLog();
            missionLog = null;
        }
    }

    /**
     * Log an event with optional target pose.
     */
    private void log(String event, String detail, Pose2d targetPose) {
        if (!LOGGING_ENABLED || missionLog == null) {
            return;
        }

        Pose2d pose = robot.driveTrain.getPose();
        String subState = getSubStateName();

        ArrayList<Object> row = new ArrayList<>();
        row.add(System.currentTimeMillis() - logStartTime);
        row.add(currentMission.name());
        row.add(missionState.name());
        row.add(subState);
        row.add(String.format("%.1f", pose.position.x));
        row.add(String.format("%.1f", pose.position.y));
        row.add(String.format("%.1f", Math.toDegrees(pose.heading.toDouble())));
        row.add(targetPose != null ? String.format("%.1f", targetPose.position.x) : "");
        row.add(targetPose != null ? String.format("%.1f", targetPose.position.y) : "");
        row.add(targetPose != null ? String.format("%.1f", Math.toDegrees(targetPose.heading.toDouble())) : "");
        row.add(event);
        row.add(detail != null ? detail : "");
        missionLog.UpdateLog(row);
    }

    /**
     * Get current sub-state name for logging.
     */
    private String getSubStateName() {
        switch (currentMission) {
            case NAVIGATE_TO_FIRE: return navToFireState.name();
            case LAUNCH_PRELOADS: return launchPreloadsState.name();
            case BALL_GROUP: return ballGroupState.name();
            case OPEN_SESAME: return openSesameState.name();
            case TUNING_ROTATION: return tuningRotationState.name();
            case TUNING_SQUARE: return tuningSquareState.name();
            case TUNING_STRAIGHT: return tuningStraightState.name();
            case TUNING_TURN: return tuningTurnState.name();
            case TUNING_RAMSETE: return tuningRamseteState.name();
            case TUNING_STRAIGHT_POS: return tuningStraightPosState.name();
            case TUNING_SQUARE_POS: return tuningSquarePosState.name();
            case TUNING_DRIFT: return tuningDriftState.name();
            case TUNING_VISION: return "CENTERING";
            case CHECK_HEALTH: return healthCheckState.name();
            default: return "NONE";
        }
    }

    /**
     * Get current sub-state name (for Autonomous logging).
     */
    public String getSubStateForLogging() {
        return getSubStateName();
    }

    // ==================== MISSION CONTROL ====================

    /**
     * Prepare for a new mission by clearing any terminal state.
     * Called automatically by startXxx() methods - callers don't need to call clearState().
     *
     * @return true if ready to start, false if a mission is currently RUNNING
     */
    private boolean prepareForNewMission() {
        if (missionState == MissionState.RUNNING) {
            return false;  // Don't interrupt running mission
        }
        // Auto-clear terminal states (COMPLETE, FAILED, ABORTED)
        if (missionState != MissionState.IDLE) {
            missionState = MissionState.IDLE;
            currentMission = Mission.NONE;
        }
        return true;
    }

    /**
     * Start the NavigateToFire mission with auto-direction.
     * Calculates whether forward or reverse requires less turning and uses that.
     * Also spins up the launcher during navigation.
     *
     * @param firePosition Which fire position (1-4)
     */
    public void startNavigateToFire(int firePosition) {
        if (!prepareForNewMission()) {
            return;
        }

        // Calculate optimal direction based on current heading
        Pose2d currentPose = robot.driveTrain.getPose();
        Pose2d firePose = getFirePose(Math.max(1, Math.min(4, firePosition)));
        boolean useReverse = shouldDriveReversed(currentPose, firePose);

        startNavigateToFire(firePosition, useReverse);
    }

    /**
     * Start the NavigateToFire mission with explicit direction.
     * Navigates to a fire position, optionally in reverse (for backing up from gate).
     * Also spins up the launcher during navigation.
     *
     * @param firePosition Which fire position (1-4)
     * @param reversed Whether to drive in reverse (true = back up, false = drive forward)
     */
    public void startNavigateToFire(int firePosition, boolean reversed) {
        if (!prepareForNewMission()) {
            return;
        }
        targetFirePosition = Math.max(1, Math.min(4, firePosition));
        navReversed = reversed;
        currentMission = Mission.NAVIGATE_TO_FIRE;
        missionState = MissionState.RUNNING;
        navToFireState = NavigateToFireState.IDLE;
        missionTimer.reset();
    }

    /**
     * Calculate whether driving in reverse would require less turning.
     * Compares angular distance to face target vs face away from target.
     */
    private boolean shouldDriveReversed(Pose2d currentPose, Pose2d targetPose) {
        double bearingToTarget = FieldMap.bearingTo(currentPose, targetPose);
        double currentHeading = currentPose.heading.toDouble();

        // Angular distance to face target (for forward driving)
        double forwardTurn = Math.abs(normalizeAngle(bearingToTarget - currentHeading));

        // Angular distance to face away from target (for reverse driving)
        double reverseBearing = normalizeAngle(bearingToTarget + Math.PI);
        double reverseTurn = Math.abs(normalizeAngle(reverseBearing - currentHeading));

        return reverseTurn < forwardTurn;
    }

    /**
     * Normalize angle to [-PI, PI] range.
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Start the LaunchPreloads mission.
     * Fires any balls that were preloaded at match start.
     */
    public void startLaunchPreloads() {
        if (!prepareForNewMission()) {
            return;
        }
        currentMission = Mission.LAUNCH_PRELOADS;
        missionState = MissionState.RUNNING;
        launchPreloadsState = LaunchPreloadsState.IDLE;
        missionTimer.reset();
    }

    /**
     * Start the BallGroup mission.
     * Navigates to specified ball group and intakes balls.
     *
     * @param groupIndex Which group to collect (0, 1, or 2)
     */
    public void startBallGroup(int groupIndex) {
        if (!prepareForNewMission()) {
            return;
        }
        targetGroupIndex = Math.max(0, Math.min(2, groupIndex));
        currentMission = Mission.BALL_GROUP;
        missionState = MissionState.RUNNING;
        ballGroupState = BallGroupState.IDLE;
        ballCountAtStart = robot.loader.getBallCount();
        missionTimer.reset();
    }

    /**
     * Start the OpenSesame mission.
     * Navigates to classifier release pose and backs into the lever.
     */
    public void startOpenSesame() {
        if (!prepareForNewMission()) {
            return;
        }
        currentMission = Mission.OPEN_SESAME;
        missionState = MissionState.RUNNING;
        openSesameState = OpenSesameState.IDLE;
        missionTimer.reset();
    }

    // ==================== TUNING MISSION START METHODS ====================

    /**
     * Start the TuningRotation mission.
     * Turns 90° CW 4x, then 90° CCW 4x, with pauses between.
     * Reports accumulated heading error at completion.
     */
    public void startTuningRotation() {
        if (!prepareForNewMission()) {
            return;
        }
        currentMission = Mission.TUNING_ROTATION;
        missionState = MissionState.RUNNING;
        tuningRotationState = TuningRotationState.IDLE;
        tuningRotationCount = 0;
        tuningStartPose = robot.driveTrain.getPose();
        missionTimer.reset();
        log("TUNING_ROTATION_START", null, tuningStartPose);
    }

    /**
     * Start the TuningSquare mission.
     * Drives 24" forward, turns 90° CW, repeats 4x to complete a square.
     * Reports position error from start at completion.
     */
    public void startTuningSquare() {
        if (!prepareForNewMission()) {
            return;
        }
        currentMission = Mission.TUNING_SQUARE;
        missionState = MissionState.RUNNING;
        tuningSquareState = TuningSquareState.IDLE;
        tuningSquareCount = 0;
        tuningStartPose = robot.driveTrain.getPose();
        missionTimer.reset();
        log("TUNING_SQUARE_START", null, tuningStartPose);
    }

    /**
     * Start the TuningStraight mission.
     * Drives 48" forward, pauses, drives 48" back to start.
     * Reports position error from start at completion.
     */
    public void startTuningStraight() {
        if (!prepareForNewMission()) {
            return;
        }
        currentMission = Mission.TUNING_STRAIGHT;
        missionState = MissionState.RUNNING;
        tuningStraightState = TuningStraightState.IDLE;
        tuningStartPose = robot.driveTrain.getPose();
        missionTimer.reset();
        log("TUNING_STRAIGHT_START", null, tuningStartPose);
    }

    /**
     * Start the TuningTurn mission.
     * Executes single turns at 45°, 90°, 180°, -90°, -45° with pauses.
     * Reports error for each turn.
     */
    public void startTuningTurn() {
        if (!prepareForNewMission()) {
            return;
        }
        currentMission = Mission.TUNING_TURN;
        missionState = MissionState.RUNNING;
        tuningTurnState = TuningTurnState.IDLE;
        tuningTurnIndex = 0;
        tuningStartPose = robot.driveTrain.getPose();
        missionTimer.reset();
        log("TUNING_TURN_START", null, tuningStartPose);
    }

    public void startTuningRamsete() {
        if (!prepareForNewMission()) {
            return;
        }
        currentMission = Mission.TUNING_RAMSETE;
        missionState = MissionState.RUNNING;
        tuningRamseteState = TuningRamseteState.IDLE;
        tuningRamseteRep = 0;
        tuningRamseteStartPose = robot.driveTrain.getPose();
        tuningRamseteTargetHeading = Math.toDegrees(tuningRamseteStartPose.heading.toDouble());
        missionTimer.reset();
        log("TUNING_RAMSETE_START", String.format("disturbance=%.1f° distance=%.1f\"",
                RAMSETE_DISTURBANCE_ANGLE, RAMSETE_DRIVE_DISTANCE), tuningRamseteStartPose);
    }

    /**
     * Start the TuningStraightPos mission (position-based).
     * Same as TuningStraight but uses PositionDriveAction instead of RoadRunner trajectories.
     */
    public void startTuningStraightPos() {
        if (!prepareForNewMission()) {
            return;
        }
        currentMission = Mission.TUNING_STRAIGHT_POS;
        missionState = MissionState.RUNNING;
        tuningStraightPosState = TuningStraightPosState.IDLE;
        tuningStartPose = robot.driveTrain.getPose();
        missionTimer.reset();
        log("TUNING_STRAIGHT_POS_START", null, tuningStartPose);
    }

    /**
     * Start the TuningSquarePos mission (position-based).
     * Same as TuningSquare but uses PositionDriveAction instead of RoadRunner trajectories.
     * Drive segments use actions.driveTo(), turns use LazyTurnAction (via the final heading).
     */
    public void startTuningSquarePos() {
        if (!prepareForNewMission()) {
            return;
        }
        currentMission = Mission.TUNING_SQUARE_POS;
        missionState = MissionState.RUNNING;
        tuningSquarePosState = TuningSquarePosState.IDLE;
        tuningSquarePosCount = 0;
        tuningStartPose = robot.driveTrain.getPose();
        missionTimer.reset();
        log("TUNING_SQUARE_POS_START", null, tuningStartPose);
    }

    public void startTuningDrift() {
        if (!prepareForNewMission()) {
            return;
        }
        currentMission = Mission.TUNING_DRIFT;
        missionState = MissionState.RUNNING;
        tuningDriftState = TuningDriftState.IDLE;
        tuningDriftCount = 0;
        cumulativeDriftX = 0.0;
        cumulativeDriftY = 0.0;
        tuningStartPose = robot.driveTrain.getPose();
        missionTimer.reset();
        log("TUNING_DRIFT_START", String.format("angle=%.0f reps=%d", DRIFT_TURN_ANGLE, DRIFT_REPS), tuningStartPose);
    }

    /**
     * Start the vision centering tuning mission.
     * Runs centerOnTarget() continuously — never self-terminates.
     * Use Back button to stop. Tune VISION_PID gains on Dashboard while running.
     */
    public void startTuningVision() {
        if (!prepareForNewMission()) {
            return;
        }
        currentMission = Mission.TUNING_VISION;
        missionState = MissionState.RUNNING;
        missionTimer.reset();
        robot.vision.setPipeline(Vision.Pipeline.DECODE);
        robot.driveTrain.centerOnTarget();
        log("TUNING_VISION_START", null, robot.driveTrain.getPose());
    }

    /**
     * Start the pre-match health check mission.
     * Sequences through battery, pinpoint, turns, flywheel, launch, intake, and vision checks.
     * @param gamepad Gamepad for driver confirmation input during star direction check
     */
    public void startCheckHealth(Gamepad gamepad) {
        if (!prepareForNewMission()) {
            return;
        }
        currentMission = Mission.CHECK_HEALTH;
        missionState = MissionState.RUNNING;
        healthCheckState = HealthCheckState.IDLE;
        healthResults.clear();
        healthDetails.clear();
        healthGamepad = gamepad;
        healthTurnCount = 0;
        missionTimer.reset();
        log("HEALTH_CHECK_START", null, null);
    }

    /**
     * Abort the current mission immediately.
     * After aborting, the same mission can be restarted from the current position.
     *
     * Usage pattern for collision avoidance:
     *   robot.missions.abort();                        // Stop immediately
     *   // ... handle obstacle ...
     *   robot.missions.startNavigateToFire(1, false);  // Restart (auto-clears ABORTED)
     */
    public void abort() {
        // Cancel any running trajectory
        if (robot.driveTrain instanceof TankDrivePinpoint) {
            ((TankDrivePinpoint) robot.driveTrain).cancelAction();
        }
        robot.driveTrain.drive(0, 0, 0);

        // Stop launcher if it was spinning up
        robot.launcher.stop();

        // Reset to manual control
        robot.setBehavior(Robot.Behavior.MANUAL);
        robot.intake.off();

        // Set aborted state (distinguishable from IDLE and FAILED)
        missionState = MissionState.ABORTED;
        resetMissionStates();
    }

    // ==================== CONFIGURATION ====================

    /**
     * Set the order to approach ball groups.
     * Called during match setup based on starting position and partner agreements.
     *
     * @param order Array of group indices (e.g., {0, 1, 2} or {2, 1, 0})
     */
    public void setBallGroupOrder(int[] order) {
        if (order != null && order.length == 3) {
            this.ballGroupOrder = order.clone();
        }
    }

    /**
     * Get the next ball group to collect based on configured order.
     *
     * @return Group index, or -1 if all groups collected
     */
    public int getNextBallGroup() {
        if (currentGroupIndex >= ballGroupOrder.length) {
            return -1;
        }
        return ballGroupOrder[currentGroupIndex];
    }

    /**
     * Advance to the next ball group in the order.
     * Call after successfully completing a BallGroup mission.
     */
    public void advanceToNextGroup() {
        currentGroupIndex++;
    }

    /**
     * Reset ball group progress (e.g., at start of match).
     */
    public void resetGroupProgress() {
        currentGroupIndex = 0;
    }

    // ==================== STATE QUERIES ====================

    /**
     * Check if a mission is currently active.
     */
    public boolean isActive() {
        return missionState == MissionState.RUNNING;
    }

    /**
     * Check if the current mission completed successfully.
     */
    public boolean isComplete() {
        return missionState == MissionState.COMPLETE;
    }

    /**
     * Check if the current mission failed (timeout or error).
     */
    public boolean isFailed() {
        return missionState == MissionState.FAILED;
    }

    /**
     * Check if the current mission was manually aborted.
     */
    public boolean isAborted() {
        return missionState == MissionState.ABORTED;
    }

    /**
     * Check if mission ended (complete, failed, or aborted).
     * Useful for simple "is it done?" checks.
     */
    public boolean isDone() {
        return missionState == MissionState.COMPLETE
            || missionState == MissionState.FAILED
            || missionState == MissionState.ABORTED;
    }

    /**
     * Get the current mission.
     */
    public Mission getCurrentMission() {
        return currentMission;
    }

    /**
     * Get the current mission state.
     */
    public MissionState getMissionState() {
        return missionState;
    }

    /**
     * Manually clear terminal state (complete/failed/aborted).
     * Usually not needed - startXxx() methods auto-clear before starting.
     * Kept public for edge cases or explicit state management.
     */
    public void clearState() {
        if (missionState == MissionState.COMPLETE
            || missionState == MissionState.FAILED
            || missionState == MissionState.ABORTED) {
            missionState = MissionState.IDLE;
            currentMission = Mission.NONE;
        }
    }

    // ==================== MAIN UPDATE ====================

    /**
     * Update the current mission.
     * Called from Robot.update() when a mission is active.
     *
     * @param fieldOverlay Canvas for field visualization
     */
    public void calc(Canvas fieldOverlay) {
        if (missionState != MissionState.RUNNING) {
            return;
        }

        switch (currentMission) {
            case NAVIGATE_TO_FIRE:
                updateNavigateToFireMission();
                break;

            case LAUNCH_PRELOADS:
                updateLaunchPreloadsMission();
                break;

            case BALL_GROUP:
                updateBallGroupMission();
                break;

            case OPEN_SESAME:
                updateOpenSesameMission();
                break;

            case GO_BALL_CLUSTER:
                // Future implementation
                missionState = MissionState.COMPLETE;
                break;

            // Tuning missions
            case TUNING_ROTATION:
                updateTuningRotationMission();
                break;

            case TUNING_SQUARE:
                updateTuningSquareMission();
                break;

            case TUNING_STRAIGHT:
                updateTuningStraightMission();
                break;

            case TUNING_TURN:
                updateTuningTurnMission();
                break;

            case TUNING_RAMSETE:
                updateTuningRamseteMission();
                break;

            case TUNING_STRAIGHT_POS:
                updateTuningStraightPosMission();
                break;

            case TUNING_SQUARE_POS:
                updateTuningSquarePosMission();
                break;

            case TUNING_DRIFT:
                updateTuningDriftMission();
                break;

            case TUNING_VISION:
                updateTuningVisionMission();
                break;

            case CHECK_HEALTH:
                updateCheckHealthMission();
                break;

            case NONE:
            default:
                missionState = MissionState.IDLE;
                break;
        }
    }

    // ==================== MISSION IMPLEMENTATIONS ====================

    private void updateNavigateToFireMission() {
        TankDrivePinpoint driveTrain = (TankDrivePinpoint) robot.driveTrain;

        // Check for timeout
        if (missionTimer.seconds() > NAVIGATION_TIMEOUT_SECONDS) {
            log("NAV_TIMEOUT", String.format("%.1fs", missionTimer.seconds()), null);
            robot.driveTrain.drive(0, 0, 0);
            robot.launcher.stop();
            missionState = MissionState.FAILED;
            navToFireState = NavigateToFireState.IDLE;
            return;
        }

        switch (navToFireState) {
            case IDLE:
                TankDriveActions.MAX_DRIVE_POWER = 1;
                // Start spinning up launcher during navigation
                robot.launcher.setBehavior(Launcher.Behavior.SPINNING);

                // Get target fire position from FieldMap
                Pose2d firePose = getFirePose(targetFirePosition);

                log("NAV_START", "reversed=" + navReversed + ",fire=" + targetFirePosition, firePose);

                // Build Turn-Spline-Turn trajectory using TankDriveActions
                // This ensures accurate heading at start and end of navigation
                Action trajectory;
                if (navReversed) {
                    trajectory = actions.driveToReversed(firePose);
                } else {
                    trajectory = actions.driveTo(firePose);
                }
                driveTrain.runAction(trajectory, actions.getLastTargetPosition());
                navToFireState = NavigateToFireState.NAVIGATING;
                log("NAV_TRAJECTORY_BUILT", null, firePose);
                break;

            case NAVIGATING:
                if (!driveTrain.isActionRunning()) {
                    // Arrived at fire position
                    log("NAV_COMPLETE", null, null);
                    missionState = MissionState.COMPLETE;
                    navToFireState = NavigateToFireState.IDLE;
                }
                break;
        }
    }

    private Pose2d getFirePose(int firePosition) {
        switch (firePosition) {
            case 1: return FieldMap.getPose(FieldMap.FIRE_1, Robot.isRedAlliance);
            case 2: return FieldMap.getPose(FieldMap.FIRE_2, Robot.isRedAlliance);
            case 3: return FieldMap.getPose(FieldMap.FIRE_3, Robot.isRedAlliance);
            case 4: return FieldMap.getPose(FieldMap.FIRE_4, Robot.isRedAlliance);
            default: return FieldMap.getPose(FieldMap.FIRE_1, Robot.isRedAlliance);
        }
    }

    private void updateLaunchPreloadsMission() {
        // Check for timeout
        if (missionTimer.seconds() > LAUNCH_TIMEOUT_SECONDS) {
            log("LAUNCH_TIMEOUT", String.format("%.1fs", missionTimer.seconds()), null);
            robot.setBehavior(Robot.Behavior.MANUAL);
            missionState = MissionState.FAILED;
            launchPreloadsState = LaunchPreloadsState.IDLE;
            return;
        }

        switch (launchPreloadsState) {
            case IDLE:
                // Check if we have balls to launch
//                if (robot.loader.isEmpty()) {
//                    log("LAUNCH_EMPTY", null, null);
//                    missionState = MissionState.COMPLETE;
//                    return;
//                }

                if (DUMMY_LAUNCH_MODE) {
                    // Dummy mode: just wait and pretend to launch
                    log("LAUNCH_START_DUMMY", "balls=" + robot.loader.getBallCount(), null);
                    launchDummyTimer.reset();
                    launchPreloadsState = LaunchPreloadsState.WAITING_DUMMY;
                } else {
                    // Real mode: Start the LAUNCH_ALL robot behavior
                    log("LAUNCH_START", "balls=" + robot.loader.getBallCount(), null);
                    robot.setBehavior(Robot.Behavior.LAUNCH_ALL);
                    launchPreloadsState = LaunchPreloadsState.WAITING_FOR_LAUNCH;
                }
                break;

            case WAITING_DUMMY:
                // Dummy mode: wait for configured duration then complete
                if (launchDummyTimer.seconds() >= DUMMY_LAUNCH_DURATION) {
                    log("LAUNCH_DONE_DUMMY", null, null);
                    missionState = MissionState.COMPLETE;
                    launchPreloadsState = LaunchPreloadsState.IDLE;
                }
                break;

            case WAITING_FOR_LAUNCH:
                // Real mode: Wait for Robot.Behavior.LAUNCH_ALL to complete
                // Robot returns to MANUAL when launch sequence is done
                if (robot.getBehavior() == Robot.Behavior.MANUAL) {
                    log("LAUNCH_DONE", null, null);
                    missionState = MissionState.COMPLETE;
                    launchPreloadsState = LaunchPreloadsState.IDLE;
                }
                break;
        }
    }

    private void updateOpenSesameMission() {
        TankDrivePinpoint driveTrain = (TankDrivePinpoint) robot.driveTrain;

        // Check for timeout
        if (missionTimer.seconds() > NAVIGATION_TIMEOUT_SECONDS + PRESS_TIMEOUT_SECONDS) {
            robot.driveTrain.drive(0, 0, 0);
            missionState = MissionState.FAILED;
            openSesameState = OpenSesameState.IDLE;
            return;
        }

        switch (openSesameState) {
            case IDLE:
                // Get gate pose from FieldMap
                Pose2d gatePose = FieldMap.getPose(FieldMap.GATE, Robot.isRedAlliance);

                // Build Turn-Spline-Turn trajectory using TankDriveActions
                Action trajectory = actions.driveTo(gatePose);
                driveTrain.runAction(trajectory, actions.getLastTargetPosition());
                openSesameState = OpenSesameState.NAVIGATING;
                break;

            case NAVIGATING:
                if (!driveTrain.isActionRunning()) {
                    // Arrived at release pose
                    openSesameState = OpenSesameState.PRESSING;
                    pressTimer.reset();
                }
                break;

            case PRESSING:
                // Drive backward into lever
                robot.driveTrain.drive(-0.3, 0, 0);
                if (pressTimer.seconds() > PRESS_DURATION_SECONDS) {
                    robot.driveTrain.drive(0, 0, 0);
                    missionState = MissionState.COMPLETE;
                    openSesameState = OpenSesameState.IDLE;
                }
                break;
        }
    }

    private void updateBallGroupMission() {
        TankDrivePinpoint driveTrain = (TankDrivePinpoint) robot.driveTrain;

        // Check for timeout
        if (missionTimer.seconds() > NAVIGATION_TIMEOUT_SECONDS + INTAKE_TIMEOUT_SECONDS) {
            log("BALLGROUP_TIMEOUT", String.format("%.1fs", missionTimer.seconds()), null);
            robot.driveTrain.drive(0, 0, 0);
            robot.intake.off();
            robot.loader.releaseBeltFromIntake();
            missionState = MissionState.FAILED;
            ballGroupState = BallGroupState.IDLE;
            return;
        }

        switch (ballGroupState) {
            case IDLE:
                // Get row start and end poses from FieldMap
                Pose2d rowStart = getRowStartPose(targetGroupIndex);
                targetRowEnd = getRowEndPose(targetGroupIndex);

                log("BALLGROUP_START", "row=" + targetGroupIndex, rowStart);

                if (TankDriveActions.USE_SPLINES) {
                    // Single chained trajectory: spline approach → intake trigger → slow drive through
                    Action intakeAction = new InstantAction(() -> {
                        robot.intake.loadAll();
                        robot.loader.requestBeltForIntake();
                        log("BALLGROUP_INTAKE_START_SPLINE", null, targetRowEnd);
                    });
                    Action trajectory = actions.buildRowTrajectory(
                            rowStart, targetRowEnd, TankDriveActions.INTAKE_VEL_INCHES_SEC,
                            intakeAction
                    );
                    driveTrain.runAction(trajectory, actions.getLastTrajectoryWaypoints());
                    ballGroupState = BallGroupState.INTAKING_THROUGH_ROW;  // Skip to final phase
                } else {
                    // Existing turn-drive-turn approach
                    Action trajectory = actions.driveTo(rowStart);
                    driveTrain.runAction(trajectory, actions.getLastTargetPosition());
                    ballGroupState = BallGroupState.NAVIGATING_TO_ROW_START;
                }
                break;

            case NAVIGATING_TO_ROW_START:
                if (!driveTrain.isActionRunning()) {
                    TankDriveActions.MAX_DRIVE_POWER = .25;
                    // Arrived at row start, begin intake run through row
                    log("BALLGROUP_AT_ROW_START", null, null);
                    robot.intake.loadAll();
                    robot.loader.requestBeltForIntake();

                    // Drive through row to row end at reduced speed for intake
                    log("BALLGROUP_INTAKE_START", null, targetRowEnd);
                    Action intakeTrajectory = actions.driveTo(targetRowEnd, INTAKE_DRIVE_POWER);
                    driveTrain.runAction(intakeTrajectory, actions.getLastTargetPosition());
                    ballGroupState = BallGroupState.INTAKING_THROUGH_ROW;
                }
                break;

            case INTAKING_THROUGH_ROW:
                // Drive through the full row while intaking — stop only when trajectory completes
                if (!driveTrain.isActionRunning()) {
                    TankDriveActions.MAX_DRIVE_POWER = 1;
                    int ballsCollected = robot.loader.getBallCount() - ballCountAtStart;
                    log("BALLGROUP_INTAKE_DONE", "trajectory_done,balls=" + ballsCollected, null);
                    robot.driveTrain.drive(0, 0, 0);
                    robot.intake.off();
                    robot.loader.releaseBeltFromIntake();
                    ballGroupState = BallGroupState.COMPLETE;
                }
                break;

            case COMPLETE:
                log("BALLGROUP_COMPLETE", null, null);
                missionState = MissionState.COMPLETE;
                ballGroupState = BallGroupState.IDLE;
                break;
        }
    }

    private Pose2d getRowStartPose(int rowIndex) {
        switch (rowIndex) {
            case 0: return FieldMap.getPose(FieldMap.BALL_ROW_1_START, Robot.isRedAlliance);
            case 1: return FieldMap.getPose(FieldMap.BALL_ROW_2_START, Robot.isRedAlliance);
            case 2: return FieldMap.getPose(FieldMap.BALL_ROW_3_START, Robot.isRedAlliance);
            default: return FieldMap.getPose(FieldMap.BALL_ROW_1_START, Robot.isRedAlliance);
        }
    }

    private Pose2d getRowEndPose(int rowIndex) {
        switch (rowIndex) {
            case 0: return FieldMap.getPose(FieldMap.BALL_ROW_1_END, Robot.isRedAlliance);
            case 1: return FieldMap.getPose(FieldMap.BALL_ROW_2_END, Robot.isRedAlliance);
            case 2: return FieldMap.getPose(FieldMap.BALL_ROW_3_END, Robot.isRedAlliance);
            default: return FieldMap.getPose(FieldMap.BALL_ROW_1_END, Robot.isRedAlliance);
        }
    }

    private void resetMissionStates() {
        navToFireState = NavigateToFireState.IDLE;
        launchPreloadsState = LaunchPreloadsState.IDLE;
        openSesameState = OpenSesameState.IDLE;
        ballGroupState = BallGroupState.IDLE;
        tuningRotationState = TuningRotationState.IDLE;
        tuningSquareState = TuningSquareState.IDLE;
        tuningStraightState = TuningStraightState.IDLE;
        tuningTurnState = TuningTurnState.IDLE;
        tuningDriftState = TuningDriftState.IDLE;
        healthCheckState = HealthCheckState.IDLE;
        currentAction = null;
    }

    // ==================== TUNING MISSION IMPLEMENTATIONS ====================

    private void updateTuningRotationMission() {
        TankDrivePinpoint driveTrain = (TankDrivePinpoint) robot.driveTrain;

        switch (tuningRotationState) {
            case IDLE:
                // Start first CW turn
                double currentHeading = Math.toDegrees(driveTrain.getPose().heading.toDouble());
                double targetHeading = currentHeading - 90;
                driveTrain.turnToHeading(targetHeading, 0.7);
                tuningRotationState = TuningRotationState.TURNING_CW;
                tuningRotationCount = 0;
                log("ROTATION_CW_START", "count=" + tuningRotationCount, null);
                break;

            case TURNING_CW:
                if (driveTrain.isTurnComplete()) {
                    tuningRotationCount++;
                    Pose2d pose = driveTrain.getPose();
                    log("ROTATION_CW_DONE", "count=" + tuningRotationCount, pose);

                    if (tuningRotationCount >= TUNING_ROTATION_REPS) {
                        // Done with CW, switch to CCW
                        tuningRotationCount = 0;
                        tuningPauseTimer.reset();
                        tuningRotationState = TuningRotationState.PAUSE_CW;
                    } else {
                        // More CW turns
                        double heading = Math.toDegrees(pose.heading.toDouble());
                        driveTrain.turnToHeading(heading - 90, 0.7);
                    }
                }
                break;

            case PAUSE_CW:
                if (tuningPauseTimer.seconds() > TUNING_PAUSE_SECONDS) {
                    // Start CCW turns
                    double heading = Math.toDegrees(driveTrain.getPose().heading.toDouble());
                    driveTrain.turnToHeading(heading + 90, 0.7);
                    tuningRotationState = TuningRotationState.TURNING_CCW;
                    log("ROTATION_CCW_START", "count=" + tuningRotationCount, null);
                }
                break;

            case TURNING_CCW:
                if (driveTrain.isTurnComplete()) {
                    tuningRotationCount++;
                    Pose2d pose = driveTrain.getPose();
                    log("ROTATION_CCW_DONE", "count=" + tuningRotationCount, pose);

                    if (tuningRotationCount >= TUNING_ROTATION_REPS) {
                        // Done with all rotations
                        tuningPauseTimer.reset();
                        tuningRotationState = TuningRotationState.PAUSE_CCW;
                    } else {
                        // More CCW turns
                        double heading = Math.toDegrees(pose.heading.toDouble());
                        driveTrain.turnToHeading(heading + 90, 0.7);
                    }
                }
                break;

            case PAUSE_CCW:
                if (tuningPauseTimer.seconds() > TUNING_PAUSE_SECONDS) {
                    tuningRotationState = TuningRotationState.COMPLETE;
                }
                break;

            case COMPLETE:
                // Report final error
                Pose2d finalPose = driveTrain.getPose();
                double headingError = Math.toDegrees(finalPose.heading.toDouble()) -
                                     Math.toDegrees(tuningStartPose.heading.toDouble());
                log("ROTATION_COMPLETE", String.format("headingError=%.2f°", headingError), finalPose);
                missionState = MissionState.COMPLETE;
                break;
        }
    }

    private void updateTuningSquareMission() {
        TankDrivePinpoint driveTrain = (TankDrivePinpoint) robot.driveTrain;

        switch (tuningSquareState) {
            case IDLE:
                // Start first drive segment
                Pose2d startPose = driveTrain.getPose();
                double heading = startPose.heading.toDouble();
                Vector2d target = new Vector2d(
                    startPose.position.x + TUNING_DRIVE_DISTANCE * Math.cos(heading),
                    startPose.position.y + TUNING_DRIVE_DISTANCE * Math.sin(heading)
                );
                Action driveAction = driveTrain.actionBuilder(startPose)
                    .lineToX(target.x)
                    .build();
                driveTrain.runAction(driveAction);
                tuningSquareState = TuningSquareState.DRIVING;
                tuningSquareCount = 0;
                log("SQUARE_DRIVE_START", "count=" + tuningSquareCount, null);
                break;

            case DRIVING:
                if (!driveTrain.isActionRunning()) {
                    Pose2d pose = driveTrain.getPose();
                    log("SQUARE_DRIVE_DONE", "count=" + tuningSquareCount, pose);
                    // Turn 90° CW
                    double currentHeading = Math.toDegrees(pose.heading.toDouble());
                    driveTrain.turnToHeading(currentHeading - 90, 0.7);
                    tuningSquareState = TuningSquareState.TURNING;
                }
                break;

            case TURNING:
                if (driveTrain.isTurnComplete()) {
                    tuningSquareCount++;
                    Pose2d pose = driveTrain.getPose();
                    log("SQUARE_TURN_DONE", "count=" + tuningSquareCount, pose);

                    if (tuningSquareCount >= 4) {
                        // Completed square
                        tuningSquareState = TuningSquareState.COMPLETE;
                    } else {
                        // Pause before next drive
                        tuningPauseTimer.reset();
                        tuningSquareState = TuningSquareState.PAUSE;
                    }
                }
                break;

            case PAUSE:
                if (tuningPauseTimer.seconds() > TUNING_PAUSE_SECONDS) {
                    // Start next drive segment
                    Pose2d pose = driveTrain.getPose();
                    double hdg = pose.heading.toDouble();
                    Vector2d tgt = new Vector2d(
                        pose.position.x + TUNING_DRIVE_DISTANCE * Math.cos(hdg),
                        pose.position.y + TUNING_DRIVE_DISTANCE * Math.sin(hdg)
                    );
                    Action action = driveTrain.actionBuilder(pose)
                        .lineToX(tgt.x)
                        .build();
                    driveTrain.runAction(action);
                    tuningSquareState = TuningSquareState.DRIVING;
                    log("SQUARE_DRIVE_START", "count=" + tuningSquareCount, null);
                }
                break;

            case COMPLETE:
                // Report position error from start
                Pose2d finalPose = driveTrain.getPose();
                double xErr = finalPose.position.x - tuningStartPose.position.x;
                double yErr = finalPose.position.y - tuningStartPose.position.y;
                double posErr = Math.sqrt(xErr*xErr + yErr*yErr);
                double hdgErr = Math.toDegrees(finalPose.heading.toDouble()) -
                               Math.toDegrees(tuningStartPose.heading.toDouble());
                log("SQUARE_COMPLETE", String.format("posErr=%.2f\" hdgErr=%.2f°", posErr, hdgErr), finalPose);
                missionState = MissionState.COMPLETE;
                break;
        }
    }

    private void updateTuningStraightMission() {
        TankDrivePinpoint driveTrain = (TankDrivePinpoint) robot.driveTrain;

        switch (tuningStraightState) {
            case IDLE:
                // Drive forward
                Pose2d startPose = driveTrain.getPose();
                double heading = startPose.heading.toDouble();
                Vector2d target = new Vector2d(
                    startPose.position.x + TUNING_STRAIGHT_DISTANCE * Math.cos(heading),
                    startPose.position.y + TUNING_STRAIGHT_DISTANCE * Math.sin(heading)
                );
                Action driveAction = driveTrain.actionBuilder(startPose)
                    .lineToX(target.x)
                    .build();
                driveTrain.runAction(driveAction);
                tuningStraightState = TuningStraightState.DRIVING_FORWARD;
                log("STRAIGHT_FORWARD_START", null, null);
                break;

            case DRIVING_FORWARD:
                if (!driveTrain.isActionRunning()) {
                    Pose2d pose = driveTrain.getPose();
                    log("STRAIGHT_FORWARD_DONE", null, pose);
                    tuningPauseTimer.reset();
                    tuningStraightState = TuningStraightState.PAUSE_FORWARD;
                }
                break;

            case PAUSE_FORWARD:
                if (tuningPauseTimer.seconds() > TUNING_PAUSE_SECONDS) {
                    // Drive back
                    Pose2d pose = driveTrain.getPose();
                    Action backAction = driveTrain.actionBuilder(pose)
                        .lineToX(tuningStartPose.position.x)
                        .build();
                    driveTrain.runAction(backAction);
                    tuningStraightState = TuningStraightState.DRIVING_BACK;
                    log("STRAIGHT_BACK_START", null, null);
                }
                break;

            case DRIVING_BACK:
                if (!driveTrain.isActionRunning()) {
                    tuningStraightState = TuningStraightState.COMPLETE;
                }
                break;

            case COMPLETE:
                // Report position error from start
                Pose2d finalPose = driveTrain.getPose();
                double xErr = finalPose.position.x - tuningStartPose.position.x;
                double yErr = finalPose.position.y - tuningStartPose.position.y;
                double posErr = Math.sqrt(xErr*xErr + yErr*yErr);
                double hdgErr = Math.toDegrees(finalPose.heading.toDouble()) -
                               Math.toDegrees(tuningStartPose.heading.toDouble());
                log("STRAIGHT_COMPLETE", String.format("posErr=%.2f\" hdgErr=%.2f°", posErr, hdgErr), finalPose);
                missionState = MissionState.COMPLETE;
                break;
        }
    }

    private void updateTuningTurnMission() {
        TankDrivePinpoint driveTrain = (TankDrivePinpoint) robot.driveTrain;

        switch (tuningTurnState) {
            case IDLE:
                if (tuningTurnIndex >= tuningTurnAngles.length) {
                    tuningTurnState = TuningTurnState.COMPLETE;
                    break;
                }
                // Start turn to next angle
                double currentHeading = Math.toDegrees(driveTrain.getPose().heading.toDouble());
                double targetAngle = tuningTurnAngles[tuningTurnIndex];
                double targetHeading = currentHeading + targetAngle;
                driveTrain.turnToHeading(targetHeading, 0.7);
                tuningTurnState = TuningTurnState.TURNING;
                log("TURN_START", String.format("target=%.0f°", targetAngle), null);
                break;

            case TURNING:
                if (driveTrain.isTurnComplete()) {
                    Pose2d pose = driveTrain.getPose();
                    log("TURN_DONE", String.format("angle=%.0f°", tuningTurnAngles[tuningTurnIndex]), pose);
                    tuningTurnIndex++;
                    tuningPauseTimer.reset();
                    tuningTurnState = TuningTurnState.PAUSE;
                }
                break;

            case PAUSE:
                if (tuningPauseTimer.seconds() > TUNING_PAUSE_SECONDS) {
                    tuningTurnState = TuningTurnState.IDLE;  // Will check for more turns
                }
                break;

            case COMPLETE:
                Pose2d finalPose = driveTrain.getPose();
                log("TURN_SEQUENCE_COMPLETE", null, finalPose);
                missionState = MissionState.COMPLETE;
                break;
        }
    }

    private void updateTuningRamseteMission() {
        TankDrivePinpoint driveTrain = (TankDrivePinpoint) robot.driveTrain;

        switch (tuningRamseteState) {
            case IDLE:
                if (tuningRamseteRep >= RAMSETE_REPS) {
                    tuningRamseteState = TuningRamseteState.COMPLETE;
                    break;
                }
                // Record current pose as start of this rep
                tuningRamseteStartPose = driveTrain.getPose();
                tuningRamseteTargetHeading = Math.toDegrees(tuningRamseteStartPose.heading.toDouble());

                // Deliberately turn off-heading to create disturbance
                // Alternate CW/CCW each rep
                double disturbance = (tuningRamseteRep % 2 == 0) ? RAMSETE_DISTURBANCE_ANGLE : -RAMSETE_DISTURBANCE_ANGLE;
                double disturbedHeading = tuningRamseteTargetHeading + disturbance;
                driveTrain.turnToHeading(disturbedHeading, 0.7);
                tuningRamseteState = TuningRamseteState.TURNING_TO_DISTURB;
                log("RAMSETE_DISTURB_START", String.format("rep=%d disturbance=%.1f°", tuningRamseteRep, disturbance), null);
                break;

            case TURNING_TO_DISTURB:
                if (driveTrain.isTurnComplete()) {
                    Pose2d pose = driveTrain.getPose();
                    double actualDisturbance = Math.toDegrees(pose.heading.toDouble()) - tuningRamseteTargetHeading;
                    log("RAMSETE_DISTURB_DONE", String.format("actualDisturbance=%.1f°", actualDisturbance), pose);
                    tuningPauseTimer.reset();
                    tuningRamseteState = TuningRamseteState.PAUSE_BEFORE_DRIVE;
                }
                break;

            case PAUSE_BEFORE_DRIVE:
                if (tuningPauseTimer.seconds() > 0.5) {  // Brief pause to stabilize
                    // Build trajectory from ORIGINAL pose (correct heading), not current pose
                    // This creates immediate heading error that Ramsete must correct
                    Pose2d currentPose = driveTrain.getPose();
                    double targetHeadingRad = Math.toRadians(tuningRamseteTargetHeading);
                    Vector2d targetPos = new Vector2d(
                            tuningRamseteStartPose.position.x + RAMSETE_DRIVE_DISTANCE * Math.cos(targetHeadingRad),
                            tuningRamseteStartPose.position.y + RAMSETE_DRIVE_DISTANCE * Math.sin(targetHeadingRad)
                    );

                    // Key: use tuningRamseteStartPose (original heading) not currentPose (disturbed)
                    // Path tangent = original heading, but robot facing disturbed heading
                    // Ramsete sees the error and must correct
                    Action trajectory = driveTrain.actionBuilder(tuningRamseteStartPose)
                            .splineTo(targetPos, targetHeadingRad)
                            .build();
                    driveTrain.runAction(trajectory);

                    double headingError = Math.toDegrees(currentPose.heading.toDouble()) - tuningRamseteTargetHeading;
                    log("RAMSETE_DRIVE_START", String.format("headingError=%.1f° target=(%.1f,%.1f)",
                            headingError, targetPos.x, targetPos.y), currentPose);
                    tuningRamseteState = TuningRamseteState.DRIVING_TRAJECTORY;
                }
                break;

            case DRIVING_TRAJECTORY:
                // Log heading error periodically during trajectory
                if (!driveTrain.isActionRunning()) {
                    Pose2d pose = driveTrain.getPose();
                    double finalHeadingError = Math.toDegrees(pose.heading.toDouble()) - tuningRamseteTargetHeading;
                    double xError = pose.position.x - (tuningRamseteStartPose.position.x + RAMSETE_DRIVE_DISTANCE * Math.cos(Math.toRadians(tuningRamseteTargetHeading)));
                    double yError = pose.position.y - (tuningRamseteStartPose.position.y + RAMSETE_DRIVE_DISTANCE * Math.sin(Math.toRadians(tuningRamseteTargetHeading)));
                    double posError = Math.sqrt(xError * xError + yError * yError);
                    log("RAMSETE_DRIVE_DONE", String.format("rep=%d hdgErr=%.1f° posErr=%.1f\"", tuningRamseteRep, finalHeadingError, posError), pose);
                    tuningRamseteRep++;
                    tuningPauseTimer.reset();
                    tuningRamseteState = TuningRamseteState.PAUSE_AFTER_DRIVE;
                }
                break;

            case PAUSE_AFTER_DRIVE:
                if (tuningPauseTimer.seconds() > TUNING_PAUSE_SECONDS) {
                    // Return to start position
                    // Build from INTENDED end pose (target position + original heading)
                    // This creates heading error that Ramsete must correct on return
                    Pose2d currentPose = driveTrain.getPose();
                    double targetHeadingRad = Math.toRadians(tuningRamseteTargetHeading);
                    Vector2d targetPos = new Vector2d(
                            tuningRamseteStartPose.position.x + RAMSETE_DRIVE_DISTANCE * Math.cos(targetHeadingRad),
                            tuningRamseteStartPose.position.y + RAMSETE_DRIVE_DISTANCE * Math.sin(targetHeadingRad)
                    );
                    // Intended pose at end of forward drive
                    Pose2d intendedEndPose = new Pose2d(targetPos, targetHeadingRad);

                    // For reversed spline, end tangent points opposite to travel direction
                    double returnTangent = targetHeadingRad + Math.PI;
                    Action returnTrajectory = driveTrain.actionBuilder(intendedEndPose)
                            .setReversed(true)  // Drive backwards
                            .splineTo(tuningRamseteStartPose.position, returnTangent)
                            .build();
                    driveTrain.runAction(returnTrajectory);
                    double headingError = Math.toDegrees(currentPose.heading.toDouble()) - tuningRamseteTargetHeading;
                    log("RAMSETE_RETURN_START", String.format("rep=%d/%d hdgErr=%.1f°",
                            tuningRamseteRep, RAMSETE_REPS, headingError), currentPose);
                    tuningRamseteState = TuningRamseteState.RETURNING_TO_START;
                }
                break;

            case RETURNING_TO_START:
                if (!driveTrain.isActionRunning()) {
                    Pose2d pose = driveTrain.getPose();
                    log("RAMSETE_RETURN_DONE", null, pose);

                    // Check if all reps complete AFTER returning
                    if (tuningRamseteRep >= RAMSETE_REPS) {
                        tuningRamseteState = TuningRamseteState.COMPLETE;
                    } else {
                        tuningPauseTimer.reset();
                        tuningRamseteState = TuningRamseteState.IDLE;  // Ready for next rep
                    }
                }
                break;

            case COMPLETE:
                Pose2d completePose = driveTrain.getPose();
                log("RAMSETE_COMPLETE", String.format("totalReps=%d", tuningRamseteRep), completePose);
                missionState = MissionState.COMPLETE;
                break;
        }
    }

    // ==================== POSITION-BASED TUNING MISSIONS ====================

    private void updateTuningStraightPosMission() {
        TankDrivePinpoint driveTrain = (TankDrivePinpoint) robot.driveTrain;

        switch (tuningStraightPosState) {
            case IDLE:
                // Drive forward using PositionDriveAction
                Pose2d startPose = driveTrain.getPose();
                double heading = startPose.heading.toDouble();
                Vector2d target = new Vector2d(
                    startPose.position.x + TUNING_STRAIGHT_DISTANCE * Math.cos(heading),
                    startPose.position.y + TUNING_STRAIGHT_DISTANCE * Math.sin(heading)
                );
                // Build a Pose2d with same heading so LazyTurnAction skips (no heading change)
                Pose2d targetPose = new Pose2d(target, heading);
                Action driveAction = actions.driveTo(targetPose);
                driveTrain.runAction(driveAction, actions.getLastTargetPosition());
                tuningStraightPosState = TuningStraightPosState.DRIVING_FORWARD;
                log("STRAIGHT_POS_FORWARD_START", null, null);
                break;

            case DRIVING_FORWARD:
                if (!driveTrain.isActionRunning()) {
                    Pose2d pose = driveTrain.getPose();
                    log("STRAIGHT_POS_FORWARD_DONE", null, pose);
                    tuningPauseTimer.reset();
                    tuningStraightPosState = TuningStraightPosState.PAUSE_FORWARD;
                }
                break;

            case PAUSE_FORWARD:
                if (tuningPauseTimer.seconds() > TUNING_PAUSE_SECONDS) {
                    // Drive back using reversed PositionDriveAction
                    Pose2d pose = driveTrain.getPose();
                    double hdg = pose.heading.toDouble();
                    Pose2d backTarget = new Pose2d(tuningStartPose.position, hdg);
                    Action backAction = actions.driveToReversed(backTarget);
                    driveTrain.runAction(backAction, actions.getLastTargetPosition());
                    tuningStraightPosState = TuningStraightPosState.DRIVING_BACK;
                    log("STRAIGHT_POS_BACK_START", null, null);
                }
                break;

            case DRIVING_BACK:
                if (!driveTrain.isActionRunning()) {
                    tuningStraightPosState = TuningStraightPosState.COMPLETE;
                }
                break;

            case COMPLETE:
                Pose2d finalPose = driveTrain.getPose();
                double xErr = finalPose.position.x - tuningStartPose.position.x;
                double yErr = finalPose.position.y - tuningStartPose.position.y;
                double posErr = Math.sqrt(xErr*xErr + yErr*yErr);
                double hdgErr = Math.toDegrees(finalPose.heading.toDouble()) -
                               Math.toDegrees(tuningStartPose.heading.toDouble());
                log("STRAIGHT_POS_COMPLETE", String.format("posErr=%.2f\" hdgErr=%.2f°", posErr, hdgErr), finalPose);
                missionState = MissionState.COMPLETE;
                break;
        }
    }

    private void updateTuningSquarePosMission() {
        TankDrivePinpoint driveTrain = (TankDrivePinpoint) robot.driveTrain;

        switch (tuningSquarePosState) {
            case IDLE:
                // Drive to next square corner using PositionDriveAction
                Pose2d startPose = driveTrain.getPose();
                double heading = startPose.heading.toDouble();
                Vector2d target = new Vector2d(
                    startPose.position.x + TUNING_DRIVE_DISTANCE * Math.cos(heading),
                    startPose.position.y + TUNING_DRIVE_DISTANCE * Math.sin(heading)
                );
                // Target heading is 90° CW from current (for the turn after arrival)
                double nextHeading = heading - Math.toRadians(90);
                Pose2d targetPose = new Pose2d(target, nextHeading);
                // driveTo will: PositionDriveAction to position, then LazyTurnAction to nextHeading
                Action driveAction = actions.driveTo(targetPose);
                driveTrain.runAction(driveAction, actions.getLastTargetPosition());
                tuningSquarePosState = TuningSquarePosState.DRIVING;
                log("SQUARE_POS_DRIVE_START", "count=" + tuningSquarePosCount, null);
                break;

            case DRIVING:
                if (!driveTrain.isActionRunning()) {
                    tuningSquarePosCount++;
                    Pose2d pose = driveTrain.getPose();
                    log("SQUARE_POS_SEGMENT_DONE", "count=" + tuningSquarePosCount, pose);

                    if (tuningSquarePosCount >= 4) {
                        tuningSquarePosState = TuningSquarePosState.COMPLETE;
                    } else {
                        tuningPauseTimer.reset();
                        tuningSquarePosState = TuningSquarePosState.PAUSE;
                    }
                }
                break;

            case PAUSE:
                if (tuningPauseTimer.seconds() > TUNING_PAUSE_SECONDS) {
                    tuningSquarePosState = TuningSquarePosState.IDLE;  // Will start next segment
                }
                break;

            case COMPLETE:
                Pose2d finalPose = driveTrain.getPose();
                double xErr = finalPose.position.x - tuningStartPose.position.x;
                double yErr = finalPose.position.y - tuningStartPose.position.y;
                double posErr = Math.sqrt(xErr*xErr + yErr*yErr);
                double hdgErr = Math.toDegrees(finalPose.heading.toDouble()) -
                               Math.toDegrees(tuningStartPose.heading.toDouble());
                log("SQUARE_POS_COMPLETE", String.format("posErr=%.2f\" hdgErr=%.2f°", posErr, hdgErr), finalPose);
                missionState = MissionState.COMPLETE;
                break;
        }
    }

    private void updateTuningVisionMission() {
        // If centering completed (on-target or lost target), restart it
        if (robot.driveTrain.isTurnComplete()) {
            robot.driveTrain.centerOnTarget();
        }
        // Never self-terminates — use Back button to stop
    }

    private void updateTuningDriftMission() {
        TankDrivePinpoint driveTrain = (TankDrivePinpoint) robot.driveTrain;

        switch (tuningDriftState) {
            case IDLE:
                // Record pre-turn pose and start first CW turn
                preTurnPose = driveTrain.getPose();
                double currentHeading = Math.toDegrees(preTurnPose.heading.toDouble());
                driveTrain.turnToHeading(currentHeading - DRIFT_TURN_ANGLE, 0.7);
                tuningDriftState = TuningDriftState.TURNING_CW;
                tuningDriftCount = 0;
                log("DRIFT_CW_START", "count=0", null);
                break;

            case TURNING_CW:
                if (driveTrain.isTurnComplete()) {
                    tuningDriftCount++;
                    Pose2d pose = driveTrain.getPose();

                    // Per-turn drift
                    double dx = pose.position.x - preTurnPose.position.x;
                    double dy = pose.position.y - preTurnPose.position.y;
                    double turnDrift = Math.sqrt(dx * dx + dy * dy);
                    cumulativeDriftX += dx;
                    cumulativeDriftY += dy;
                    double totalDrift = Math.sqrt(cumulativeDriftX * cumulativeDriftX + cumulativeDriftY * cumulativeDriftY);

                    log("DRIFT_CW_DONE", String.format(
                            "count=%d dx=%.2f dy=%.2f drift=%.2f cumDx=%.2f cumDy=%.2f cumDrift=%.2f",
                            tuningDriftCount, dx, dy, turnDrift,
                            cumulativeDriftX, cumulativeDriftY, totalDrift), pose);

                    if (tuningDriftCount >= DRIFT_REPS) {
                        // Done with CW, pause before CCW
                        tuningDriftCount = 0;
                        tuningPauseTimer.reset();
                        tuningDriftState = TuningDriftState.PAUSE_CW;
                        log("DRIFT_CW_PHASE_DONE", String.format(
                                "cumDx=%.2f cumDy=%.2f cumDrift=%.2f", cumulativeDriftX, cumulativeDriftY, totalDrift), null);
                    } else {
                        // More CW turns — record new pre-turn pose
                        preTurnPose = pose;
                        double hdg = Math.toDegrees(pose.heading.toDouble());
                        driveTrain.turnToHeading(hdg - DRIFT_TURN_ANGLE, 0.7);
                    }
                }
                break;

            case PAUSE_CW:
                if (tuningPauseTimer.seconds() > TUNING_PAUSE_SECONDS) {
                    // Start CCW turns — record pre-turn pose
                    preTurnPose = driveTrain.getPose();
                    double hdg = Math.toDegrees(preTurnPose.heading.toDouble());
                    driveTrain.turnToHeading(hdg + DRIFT_TURN_ANGLE, 0.7);
                    tuningDriftState = TuningDriftState.TURNING_CCW;
                    log("DRIFT_CCW_START", "count=0", null);
                }
                break;

            case TURNING_CCW:
                if (driveTrain.isTurnComplete()) {
                    tuningDriftCount++;
                    Pose2d pose = driveTrain.getPose();

                    // Per-turn drift
                    double dxCcw = pose.position.x - preTurnPose.position.x;
                    double dyCcw = pose.position.y - preTurnPose.position.y;
                    double turnDriftCcw = Math.sqrt(dxCcw * dxCcw + dyCcw * dyCcw);
                    cumulativeDriftX += dxCcw;
                    cumulativeDriftY += dyCcw;
                    double totalDriftCcw = Math.sqrt(cumulativeDriftX * cumulativeDriftX + cumulativeDriftY * cumulativeDriftY);

                    log("DRIFT_CCW_DONE", String.format(
                            "count=%d dx=%.2f dy=%.2f drift=%.2f cumDx=%.2f cumDy=%.2f cumDrift=%.2f",
                            tuningDriftCount, dxCcw, dyCcw, turnDriftCcw,
                            cumulativeDriftX, cumulativeDriftY, totalDriftCcw), pose);

                    if (tuningDriftCount >= DRIFT_REPS) {
                        // Done with all turns
                        tuningPauseTimer.reset();
                        tuningDriftState = TuningDriftState.PAUSE_CCW;
                        log("DRIFT_CCW_PHASE_DONE", String.format(
                                "cumDx=%.2f cumDy=%.2f cumDrift=%.2f", cumulativeDriftX, cumulativeDriftY, totalDriftCcw), null);
                    } else {
                        // More CCW turns — record new pre-turn pose
                        preTurnPose = pose;
                        double hdgCcw = Math.toDegrees(pose.heading.toDouble());
                        driveTrain.turnToHeading(hdgCcw + DRIFT_TURN_ANGLE, 0.7);
                    }
                }
                break;

            case PAUSE_CCW:
                if (tuningPauseTimer.seconds() > TUNING_PAUSE_SECONDS) {
                    tuningDriftState = TuningDriftState.COMPLETE;
                }
                break;

            case COMPLETE:
                Pose2d finalPose = driveTrain.getPose();
                double totalX = finalPose.position.x - tuningStartPose.position.x;
                double totalY = finalPose.position.y - tuningStartPose.position.y;
                double totalPos = Math.sqrt(totalX * totalX + totalY * totalY);
                double hdgErr = Math.toDegrees(finalPose.heading.toDouble()) -
                               Math.toDegrees(tuningStartPose.heading.toDouble());
                log("DRIFT_COMPLETE", String.format(
                        "totalDx=%.2f totalDy=%.2f totalDrift=%.2f hdgErr=%.2f° cumDx=%.2f cumDy=%.2f",
                        totalX, totalY, totalPos, hdgErr, cumulativeDriftX, cumulativeDriftY), finalPose);
                missionState = MissionState.COMPLETE;
                break;
        }
    }

    // ==================== HEALTH CHECK IMPLEMENTATION ====================

    private void updateCheckHealthMission() {
        TankDrivePinpoint driveTrain = (TankDrivePinpoint) robot.driveTrain;

        switch (healthCheckState) {
            case IDLE:
                healthCheckState = HealthCheckState.CHECK_BATTERY;
                break;

            // ---- Battery ----
            case CHECK_BATTERY: {
                double voltage = robot.getVoltage();
                boolean pass = voltage >= HEALTH_BATTERY_MIN;
                healthResults.put("Battery", pass);
                healthDetails.put("Battery", String.format("%.1fV (min %.1f)", voltage, HEALTH_BATTERY_MIN));
                log("HEALTH_BATTERY", (pass ? "PASS" : "FAIL") + " " + String.format("%.1fV", voltage), null);
                healthCheckState = HealthCheckState.CHECK_PINPOINT;
                break;
            }

            // ---- Pinpoint ----
            case CHECK_PINPOINT: {
                boolean pass = false;
                String detail;
                if (driveTrain.localizer instanceof PinpointLocalizer) {
                    PinpointLocalizer pp = (PinpointLocalizer) driveTrain.localizer;
                    GoBildaPinpointDriver.DeviceStatus status = pp.driver.getDeviceStatus();
                    int parTicks = pp.getPar();
                    pass = (status == GoBildaPinpointDriver.DeviceStatus.READY);
                    detail = "status=" + status + " parTicks=" + parTicks;
                } else {
                    detail = "not PinpointLocalizer";
                }
                healthResults.put("Pinpoint", pass);
                healthDetails.put("Pinpoint", detail);
                log("HEALTH_PINPOINT", (pass ? "PASS" : "FAIL") + " " + detail, null);
                healthCheckState = HealthCheckState.START_TURNS;
                break;
            }

            // ---- Turns (4x 90° CW) ----
            case START_TURNS: {
                healthPreTurnPose = driveTrain.getPose();
                healthTurnCount = 0;
                double heading = Math.toDegrees(healthPreTurnPose.heading.toDouble());
                driveTrain.turnToHeading(heading - 90, 0.7);
                healthCheckState = HealthCheckState.WAITING_TURNS;
                healthPhaseTimer.reset();
                log("HEALTH_TURNS_START", null, healthPreTurnPose);
                break;
            }

            case WAITING_TURNS:
                if (driveTrain.isTurnComplete()) {
                    healthTurnCount++;
                    if (healthTurnCount >= 4) {
                        healthCheckState = HealthCheckState.EVALUATE_TURNS;
                    } else {
                        double heading = Math.toDegrees(driveTrain.getPose().heading.toDouble());
                        driveTrain.turnToHeading(heading - 90, 0.7);
                    }
                } else if (healthPhaseTimer.seconds() > HEALTH_CHECK_TIMEOUT * 4) {
                    // Timeout on turns
                    healthResults.put("Turns", false);
                    healthDetails.put("Turns", "TIMEOUT after " + healthTurnCount + " turns");
                    log("HEALTH_TURNS_TIMEOUT", "completed=" + healthTurnCount, null);
                    healthCheckState = HealthCheckState.START_FLYWHEEL;
                }
                break;

            case EVALUATE_TURNS: {
                Pose2d postPose = driveTrain.getPose();
                double headingError = Math.abs(Math.toDegrees(postPose.heading.toDouble()) -
                        Math.toDegrees(healthPreTurnPose.heading.toDouble()));
                double dx = postPose.position.x - healthPreTurnPose.position.x;
                double dy = postPose.position.y - healthPreTurnPose.position.y;
                double positionDrift = Math.sqrt(dx * dx + dy * dy);

                boolean headingPass = headingError < HEALTH_HEADING_TOLERANCE;
                boolean positionPass = positionDrift < HEALTH_POSITION_TOLERANCE;

                healthResults.put("Turn Heading", headingPass);
                healthDetails.put("Turn Heading", String.format("%.1f° err (max %.1f)", headingError, HEALTH_HEADING_TOLERANCE));
                healthResults.put("Turn Position", positionPass);
                healthDetails.put("Turn Position", String.format("%.1f\" drift (max %.1f)", positionDrift, HEALTH_POSITION_TOLERANCE));
                log("HEALTH_TURNS_DONE", String.format("hdgErr=%.1f posDrift=%.1f", headingError, positionDrift), postPose);
                healthCheckState = HealthCheckState.START_FLYWHEEL;
                break;
            }

            // ---- Flywheel ----
            case START_FLYWHEEL: {
                // Save current speed and set reduced test speed
                healthSavedLaunchSpeed = Launcher.MIN_LAUNCH_SPEED;
                Launcher.MIN_LAUNCH_SPEED = HEALTH_FLYWHEEL_SPEED;
                // Record starting encoder positions
                healthFlywheelStartEncoder = robot.launcher.getFlywheelEncoder();
                healthHelperStartEncoder = robot.launcher.getHelperEncoder();
                robot.launcher.setBehavior(Launcher.Behavior.SPINNING);
                healthPhaseTimer.reset();
                log("HEALTH_FLYWHEEL_START", "target=" + HEALTH_FLYWHEEL_SPEED, null);
                healthCheckState = HealthCheckState.WAITING_FLYWHEEL;
                break;
            }

            case WAITING_FLYWHEEL:
                if (robot.launcher.isReady()) {
                    healthCheckState = HealthCheckState.EVALUATE_FLYWHEEL;
                } else if (healthPhaseTimer.seconds() > HEALTH_CHECK_TIMEOUT) {
                    healthCheckState = HealthCheckState.EVALUATE_FLYWHEEL;
                }
                break;

            case EVALUATE_FLYWHEEL: {
                boolean reachedSpeed = robot.launcher.isReady();
                // Read amps (flywheel motors are DcMotorEx, getCurrent available via debug telemetry)
                healthFlywheelAmps = robot.launcher.getFlywheelCurrent();
                healthHelperAmps = robot.launcher.getHelperCurrent();
                double ampDiff = Math.abs(healthFlywheelAmps - healthHelperAmps);

                int flywheelDelta = Math.abs(robot.launcher.getFlywheelEncoder() - healthFlywheelStartEncoder);
                int helperDelta = Math.abs(robot.launcher.getHelperEncoder() - healthHelperStartEncoder);
                double encoderRatio = (Math.max(flywheelDelta, helperDelta) > 0) ?
                        (double) Math.min(flywheelDelta, helperDelta) / Math.max(flywheelDelta, helperDelta) : 0;

                boolean ampPass = ampDiff < HEALTH_FLYWHEEL_AMP_DIFF && healthFlywheelAmps > 0.05 && healthHelperAmps > 0.05;
                boolean encoderPass = encoderRatio >= HEALTH_FLYWHEEL_ENCODER_RATIO;

                healthResults.put("Flywheel Speed", reachedSpeed);
                healthDetails.put("Flywheel Speed", reachedSpeed ? "reached " + HEALTH_FLYWHEEL_SPEED + " deg/s" : "FAILED to reach speed");
                healthResults.put("Flywheel Amps", ampPass);
                healthDetails.put("Flywheel Amps", String.format("main=%.2fA helper=%.2fA diff=%.2f", healthFlywheelAmps, healthHelperAmps, ampDiff));
                healthResults.put("Flywheel Encoders", encoderPass);
                healthDetails.put("Flywheel Encoders", String.format("ratio=%.2f (main=%d helper=%d)", encoderRatio, flywheelDelta, helperDelta));

                log("HEALTH_FLYWHEEL_DONE", String.format("speed=%s amps=%.2f/%.2f encRatio=%.2f",
                        reachedSpeed, healthFlywheelAmps, healthHelperAmps, encoderRatio), null);
                healthCheckState = HealthCheckState.START_LAUNCH;
                break;
            }

            // ---- Launch (fire to test star + conveyor) ----
            case START_LAUNCH: {
                // Only attempt fire if launcher actually reached READY state
                if (robot.launcher.getState() == Launcher.LaunchState.READY) {
                    robot.loader.enableBeltCurrentRead(true);
                    robot.launcher.fire();
                    healthPhaseTimer.reset();
                    healthBeltAmps = 0;
                    log("HEALTH_LAUNCH_START", null, null);
                    healthCheckState = HealthCheckState.WAITING_LAUNCH;
                } else {
                    // Flywheel never reached speed — skip launch test
                    robot.launcher.setBehavior(Launcher.Behavior.IDLE);
                    Launcher.MIN_LAUNCH_SPEED = healthSavedLaunchSpeed;
                    healthResults.put("Launch", false);
                    healthDetails.put("Launch", "SKIPPED — flywheel not ready");
                    healthResults.put("Conveyor (launch)", false);
                    healthDetails.put("Conveyor (launch)", "SKIPPED — no fire attempted");
                    log("HEALTH_LAUNCH_SKIPPED", "flywheel not ready", null);
                    healthPhaseTimer.reset();
                    healthCheckState = HealthCheckState.WAITING_DRIVER_CONFIRM;
                }
                break;
            }

            case WAITING_LAUNCH: {
                // Sample belt current while launcher is firing
                double currentBeltAmps = robot.loader.getBeltCurrent();
                healthBeltAmps = Math.max(healthBeltAmps, currentBeltAmps);

                Launcher.LaunchState launcherState = robot.launcher.getState();
                boolean done = (launcherState == Launcher.LaunchState.IDLE ||
                        launcherState == Launcher.LaunchState.READY);
                if (done || healthPhaseTimer.seconds() > HEALTH_CHECK_TIMEOUT) {
                    healthCheckState = HealthCheckState.EVALUATE_LAUNCH;
                }
                break;
            }

            case EVALUATE_LAUNCH: {
                robot.loader.enableBeltCurrentRead(false);
                boolean conveyorPass = healthBeltAmps > HEALTH_INTAKE_MIN_AMPS;
                healthResults.put("Conveyor (launch)", conveyorPass);
                healthDetails.put("Conveyor (launch)", String.format("peak=%.2fA (min %.2f)", healthBeltAmps, HEALTH_INTAKE_MIN_AMPS));

                // Shut down flywheel, restore speed
                robot.launcher.setBehavior(Launcher.Behavior.IDLE);
                Launcher.MIN_LAUNCH_SPEED = healthSavedLaunchSpeed;

                log("HEALTH_LAUNCH_DONE", String.format("beltAmps=%.2f", healthBeltAmps), null);
                healthPhaseTimer.reset();
                healthCheckState = HealthCheckState.WAITING_DRIVER_CONFIRM;
                break;
            }

            // ---- Driver confirmation for star direction ----
            case WAITING_DRIVER_CONFIRM:
                // Telemetry will show prompt; driver presses A (pass) or B (fail)
                if (healthGamepad != null) {
                    if (healthGamepad.a) {
                        healthResults.put("Star Direction", true);
                        healthDetails.put("Star Direction", "driver confirmed OK");
                        log("HEALTH_STAR", "PASS", null);
                        healthCheckState = HealthCheckState.START_INTAKE;
                    } else if (healthGamepad.b) {
                        healthResults.put("Star Direction", false);
                        healthDetails.put("Star Direction", "driver reported WRONG direction");
                        log("HEALTH_STAR", "FAIL", null);
                        healthCheckState = HealthCheckState.START_INTAKE;
                    } else if (healthPhaseTimer.seconds() > HEALTH_CHECK_TIMEOUT * 2) {
                        // Timeout — driver didn't respond
                        healthResults.put("Star Direction", false);
                        healthDetails.put("Star Direction", "TIMEOUT — no driver response");
                        log("HEALTH_STAR", "TIMEOUT", null);
                        healthCheckState = HealthCheckState.START_INTAKE;
                    }
                    // Otherwise keep waiting
                } else {
                    // No gamepad, skip
                    healthResults.put("Star Direction", false);
                    healthDetails.put("Star Direction", "no gamepad — skipped");
                    healthCheckState = HealthCheckState.START_INTAKE;
                }
                break;

            // ---- Intake + Conveyor ----
            case START_INTAKE: {
                robot.intake.enableCurrentRead(true);
                robot.loader.enableBeltCurrentRead(true);
                robot.intake.on();
                robot.loader.requestBeltForIntake();
                healthIntakeAmps = 0;
                healthBeltAmps = 0;
                healthPhaseTimer.reset();
                log("HEALTH_INTAKE_START", null, null);
                healthCheckState = HealthCheckState.WAITING_INTAKE;
                break;
            }

            case WAITING_INTAKE:
                // Sample peak current
                healthIntakeAmps = Math.max(healthIntakeAmps, robot.intake.getMotorCurrent());
                healthBeltAmps = Math.max(healthBeltAmps, robot.loader.getBeltCurrent());
                if (healthPhaseTimer.seconds() > 1.0) {
                    healthCheckState = HealthCheckState.EVALUATE_INTAKE;
                }
                break;

            case EVALUATE_INTAKE: {
                robot.intake.off();
                robot.loader.releaseBeltFromIntake();
                robot.intake.enableCurrentRead(false);
                robot.loader.enableBeltCurrentRead(false);

                boolean intakePass = healthIntakeAmps > HEALTH_INTAKE_MIN_AMPS;
                boolean beltPass = healthBeltAmps > HEALTH_INTAKE_MIN_AMPS;
                healthResults.put("Intake Motor", intakePass);
                healthDetails.put("Intake Motor", String.format("peak=%.2fA (min %.2f)", healthIntakeAmps, HEALTH_INTAKE_MIN_AMPS));
                healthResults.put("Conveyor (intake)", beltPass);
                healthDetails.put("Conveyor (intake)", String.format("peak=%.2fA (min %.2f)", healthBeltAmps, HEALTH_INTAKE_MIN_AMPS));
                log("HEALTH_INTAKE_DONE", String.format("intakeAmps=%.2f beltAmps=%.2f", healthIntakeAmps, healthBeltAmps), null);
                healthCheckState = HealthCheckState.CHECK_VISION;
                break;
            }

            // ---- Vision ----
            case CHECK_VISION: {
                boolean hasTarget = robot.vision.hasTarget();
                boolean hasBotPose = robot.vision.hasBotPose();
                // Informational only — not a pass/fail since we can't guarantee a target is visible
                healthResults.put("Vision", true); // Always "pass" — info only
                healthDetails.put("Vision", "target=" + (hasTarget ? "YES" : "no") +
                        " botpose=" + (hasBotPose ? "YES" : "no"));
                log("HEALTH_VISION", "target=" + hasTarget + " botpose=" + hasBotPose, null);
                healthCheckState = HealthCheckState.SHOW_RESULTS;
                break;
            }

            // ---- Results ----
            case SHOW_RESULTS: {
                // Count failures
                int failures = 0;
                for (Boolean pass : healthResults.values()) {
                    if (!pass) failures++;
                }
                String summary = (failures == 0) ? "ALL CHECKS PASSED" : failures + " CHECK(S) FAILED";
                log("HEALTH_COMPLETE", summary, null);
                healthCheckState = HealthCheckState.COMPLETE;
                break;
            }

            case COMPLETE:
                // Stay in COMPLETE — results visible in telemetry until another mission starts
                missionState = MissionState.COMPLETE;
                break;
        }
    }

    // ==================== TELEMETRY ====================

    @Override
    public String getTelemetryName() {
        return "Missions";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Mission", currentMission);
        telemetry.put("State", missionState);

        if (missionState == MissionState.RUNNING) {
            telemetry.put("Time", String.format("%.1fs", missionTimer.seconds()));
        }

        if (debug) {
            telemetry.put("Group Order", java.util.Arrays.toString(ballGroupOrder));
            telemetry.put("Current Group Index", currentGroupIndex);

            if (currentMission == Mission.NAVIGATE_TO_FIRE) {
                telemetry.put("NavToFire State", navToFireState);
                telemetry.put("Fire Position", targetFirePosition);
                telemetry.put("Reversed", navReversed);
            }
            if (currentMission == Mission.BALL_GROUP) {
                telemetry.put("Target Group", targetGroupIndex);
                telemetry.put("BallGroup State", ballGroupState);
            }
            if (currentMission == Mission.OPEN_SESAME) {
                telemetry.put("OpenSesame State", openSesameState);
            }
            if (currentMission == Mission.LAUNCH_PRELOADS) {
                telemetry.put("LaunchPreloads State", launchPreloadsState);
            }
            // Tuning mission telemetry
            if (currentMission == Mission.TUNING_ROTATION) {
                telemetry.put("TuningRotation State", tuningRotationState);
                telemetry.put("Rotation Count", tuningRotationCount);
            }
            if (currentMission == Mission.TUNING_SQUARE) {
                telemetry.put("TuningSquare State", tuningSquareState);
                telemetry.put("Square Count", tuningSquareCount);
            }
            if (currentMission == Mission.TUNING_STRAIGHT) {
                telemetry.put("TuningStraight State", tuningStraightState);
            }
            if (currentMission == Mission.TUNING_TURN) {
                telemetry.put("TuningTurn State", tuningTurnState);
                telemetry.put("Turn Index", tuningTurnIndex + "/" + tuningTurnAngles.length);
            }
            if (currentMission == Mission.TUNING_RAMSETE) {
                telemetry.put("TuningRamsete State", tuningRamseteState);
                telemetry.put("Ramsete Rep", tuningRamseteRep + "/" + RAMSETE_REPS);
                double currentHeading = Math.toDegrees(robot.driveTrain.getPose().heading.toDouble());
                telemetry.put("Heading Error", String.format("%.1f°", currentHeading - tuningRamseteTargetHeading));
            }
            if (currentMission == Mission.TUNING_STRAIGHT_POS) {
                telemetry.put("TuningStraightPos State", tuningStraightPosState);
            }
            if (currentMission == Mission.TUNING_SQUARE_POS) {
                telemetry.put("TuningSquarePos State", tuningSquarePosState);
                telemetry.put("Square Count", tuningSquarePosCount);
            }
            if (currentMission == Mission.TUNING_DRIFT) {
                telemetry.put("Drift State", tuningDriftState);
                telemetry.put("Turn Count", tuningDriftCount);
                telemetry.put("Cumulative dX", String.format("%.2f\"", cumulativeDriftX));
                telemetry.put("Cumulative dY", String.format("%.2f\"", cumulativeDriftY));
                double cumDist = Math.sqrt(cumulativeDriftX * cumulativeDriftX + cumulativeDriftY * cumulativeDriftY);
                telemetry.put("Cumulative Drift", String.format("%.2f\"", cumDist));
            }
            if (currentMission == Mission.TUNING_VISION) {
                telemetry.put("Vision tx", String.format("%.1f°", robot.vision.getTx()));
                telemetry.put("Elapsed", String.format("%.1fs", missionTimer.seconds()));
            }
            if (currentMission == Mission.CHECK_HEALTH) {
                telemetry.put("HealthCheck State", healthCheckState);
            }
        }

        // Health check results always shown (not gated by debug) so driver can see report
        if (currentMission == Mission.CHECK_HEALTH) {
            if (healthCheckState == HealthCheckState.WAITING_DRIVER_CONFIRM) {
                telemetry.put(">>> CONFIRM", "Did star spin correctly? A=YES  B=NO");
            }
            for (Map.Entry<String, String> entry : healthDetails.entrySet()) {
                String prefix = healthResults.getOrDefault(entry.getKey(), false) ? "PASS" : "FAIL";
                telemetry.put(entry.getKey(), prefix + " — " + entry.getValue());
            }
            if (healthCheckState == HealthCheckState.COMPLETE || healthCheckState == HealthCheckState.SHOW_RESULTS) {
                int failures = 0;
                for (Boolean pass : healthResults.values()) {
                    if (!pass) failures++;
                }
                telemetry.put("RESULT", (failures == 0) ? "ALL CHECKS PASSED" : failures + " CHECK(S) FAILED");
            }
        }

        return telemetry;
    }
}
