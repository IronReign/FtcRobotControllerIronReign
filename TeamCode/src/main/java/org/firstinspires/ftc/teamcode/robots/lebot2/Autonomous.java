package org.firstinspires.ftc.teamcode.robots.lebot2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Launcher;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.util.CsvLogKeeper;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

/**
 * Autonomous strategy coordinator for Lebot2 - Goal Wall Start.
 *
 * STRATEGY OVERVIEW:
 * 1. Back up from gate to fire position (while spinning up launcher)
 * 2. Target goal with vision, launch preloaded balls
 * 3. Collect ball row 1, return to fire, target, launch
 * 4. Collect ball row 2, return to fire, target, launch
 * 5. Navigate to gate, release scored balls (Open Sesame)
 * 6. Collect ball row 3 (includes released balls), return to fire, launch
 *
 * ARCHITECTURE:
 * - Autonomous handles STRATEGIC decisions (which missions, in what order)
 * - Missions handles TACTICAL execution (trajectories, timeouts, subsystem coordination)
 * - Time-based branching skips phases if running low on time
 *
 * See docs/AutonomousStrategy.md for detailed documentation.
 */
@Config(value = "Lebot2_Autonomous")
public class Autonomous implements TelemetryProvider {

    private final Robot robot;

    // ==================== CONFIGURATION ====================

    // Strategic configuration (tunable via FTC Dashboard)
    public static boolean START_AT_GOAL_WALL = true;  // true = goal wall, false = audience wall
    public static boolean DO_OPEN_SESAME = true;      // Release gate after rows 1 & 2?
    public static double MIN_TIME_FOR_ROW = 5.0;     // Minimum seconds needed for a ball row cycle

    public static boolean SKIP_LAUNCH=true;

    // Derived strategy parameters (set by init() based on START_AT_GOAL_WALL, also Dashboard-tunable)
    public static int FIRE_POSITION = 1;              // Which fire position to use (1-4)
    public static int ROW_START = 0;                  // First ball row index
    public static int ROW_END = 2;                    // Last ball row index
    public static int ROW_DIRECTION = 1;              // +1 = forward (0→2), -1 = reverse (2→0)
    public static int GATE_BEFORE_ROW = 2;            // Do gate release before this row index
    public static boolean SKIP_INITIAL_BACKUP = false; // Skip backup if already at fire position

    // Selective abort - configurable via gamepad during init_loop (D-pad Right)
    // -1 = all rows, 0 = no rows, 1 = one row, 2 = two rows
    public static int ABORT_AFTER_ROWS = -1;
    // Alternate positions for abort (Dashboard-tunable for field adjustment)
    public static String ALT_POSITION_GOAL = FieldMap.FIRE_2;  // Inside big triangle
    public static String ALT_POSITION_AUDIENCE = FieldMap.BASE;  // Opposing alliance's base

    // Autonomous time limit
    public static final double AUTON_DURATION_SECONDS = 30.0;

    // ==================== STATE MACHINE ====================

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

        // Ball collection cycle
        START_BALL_ROW,
        WAITING_BALL_ROW,
        START_RETURN_TO_FIRE,
        WAITING_RETURN,

        // Gate release (after rows 1 & 2)
        START_GATE,
        WAITING_GATE,

        // Selective abort - navigate to alternate position
        START_ALTERNATE,
        WAITING_ALTERNATE,

        // End
        COMPLETE
    }

    private AutonState state = AutonState.INIT;
    private AutonState previousState = AutonState.INIT;

    // ==================== PROGRESS TRACKING ====================

    private ElapsedTime autonTimer = new ElapsedTime();
    private int currentRow = 0;      // 0, 1, 2 for rows 1, 2, 3
    private int rowsCompleted = 0;   // Count of completed rows (for selective abort)
    private int launchCycles = 0;
    private boolean gateReleased = false;

    // ==================== LOGGING ====================

    public static boolean LOGGING_ENABLED = true;  // Dashboard tunable
    private CsvLogKeeper autonLog = null;
    private long logStartTime = 0;

    // ==================== CONSTRUCTOR ====================

    public Autonomous(Robot robot) {
        this.robot = robot;
    }

    // ==================== INITIALIZATION ====================

    /**
     * Initialize autonomous. Call during init phase before starting.
     */
    public void init() {
        // Reset mission progress
        robot.missions.resetGroupProgress();
        robot.missions.clearState();

        // Set strategy parameters based on starting position
        if (robot.getStartingPosition() != Robot.StartingPosition.AUDIENCE) {
            SKIP_LAUNCH=false;
            FIRE_POSITION = 1;
            Launcher.STAR_FEEDING = 1;
            Launcher.MIN_LAUNCH_SPEED = FieldMap.FIRE_1_DEFAULT_DPS;
            ROW_START = 0;
            ROW_END = 2;
            ROW_DIRECTION = 1;
            GATE_BEFORE_ROW = 2;
            SKIP_INITIAL_BACKUP = false;
        } else {
            SKIP_LAUNCH=true;
            FIRE_POSITION = 4;
            Launcher.STAR_FEEDING = 0.7;
            Launcher.MIN_LAUNCH_SPEED = FieldMap.FIRE_4_DEFAULT_DPS;
            ROW_START = 2;
            ROW_END = 0;
            ROW_DIRECTION = -1;
            GATE_BEFORE_ROW = 0;
            SKIP_INITIAL_BACKUP = true;
        }

        // Reset state
        state = AutonState.INIT;
        previousState = AutonState.INIT;
        currentRow = ROW_START;
        rowsCompleted = 0;
        launchCycles = 0;
        gateReleased = false;

        // Set robot starting pose based on configured start position
        // (FieldMap.IS_AUDIENCE_START is already set by Robot.setStartingPosition during init_loop)
        Pose2d startPose = robot.getStartingPose(robot.getStartingPosition());
        robot.driveTrain.setPose(startPose);

        // Timer will be reset when execute() first runs
        autonTimer.reset();

        // Initialize logging with timestamped filename
        if (LOGGING_ENABLED) {
            String timestamp = new SimpleDateFormat("yyMMddHHmm", Locale.US).format(new Date());
            autonLog = new CsvLogKeeper("auton_" + timestamp, 12,
                "elapsedMs,state,prevState,missionState,missionSubState,row,launches,timeRemain,poseX,poseY,heading,event");
            logStartTime = System.currentTimeMillis();
            log("INIT", null);

            // Initialize mission logging with same timestamp
            robot.missions.initLogging();
        }
    }

    // ==================== MAIN EXECUTE ====================

    /**
     * Execute autonomous - call every loop cycle.
     */
    public void execute() {
        // Time remaining for branching decisions
        double timeRemaining = AUTON_DURATION_SECONDS - autonTimer.seconds();

        // Hard stop — kill everything when auton time expires
        if (timeRemaining <= 0 && state != AutonState.COMPLETE) {
            log("AUTON_TIMEOUT", null);
            robot.stop();
            setState(AutonState.COMPLETE);
            return;
        }

        switch (state) {
            case INIT:
                autonTimer.reset();
                setState(AutonState.START_BACKUP_TO_FIRE);
                break;

            // ========== PHASE 1: Backup to Fire ==========
            case START_BACKUP_TO_FIRE:
                if(!SKIP_LAUNCH) {

                    if (SKIP_INITIAL_BACKUP) {
                        // Already at fire position — just spin up launcher
                        robot.launcher.setBehavior(Launcher.Behavior.SPINNING);
                        setState(AutonState.START_TARGETING);
                    } else {
                        // Back up from gate to fire position, spins up launcher during navigation
                        robot.missions.startNavigateToFire(FIRE_POSITION, true);
                        setState(AutonState.WAITING_BACKUP);
                    }
                }else{
                    // SKIP_LAUNCH: just get off the starting line for LEAVE points
                    // Go to opposing alliance's base (same destination as audience abort)
                    robot.missions.startNavigateTo(ALT_POSITION_AUDIENCE, !Robot.isRedAlliance);
                    setState(AutonState.WAITING_BACKUP);
                }
                break;

            case WAITING_BACKUP:
                if (robot.missions.isComplete()) {
                    log("BACKUP_COMPLETE", null);
                    if(!SKIP_LAUNCH){
                        setState(AutonState.START_TARGETING);
                    }else {
                        setState(AutonState.COMPLETE);
                    }
                } else if (robot.missions.isFailed()) {
                    // Timeout - try to launch anyway from current position
                    log("BACKUP_FAILED", "timeout");
                    setState(AutonState.START_LAUNCH);
                }
                break;

            // ========== PHASE 2-3: Target and Launch ==========
            case START_TARGETING:
                //robot.setBehavior(Robot.Behavior.TARGETING);
                setState(AutonState.WAITING_TARGET);
                break;

            case WAITING_TARGET:
                //if (robot.getBehavior() == Robot.Behavior.MANUAL) {
                    // Targeting complete, apply vision correction
                    log("TARGETING_COMPLETE", null);
                    robot.applyVisionPoseCorrection();
                    setState(AutonState.START_LAUNCH);
                //}
                break;

            case START_LAUNCH:
                //if (!robot.loader.isEmpty()) {
                    log("LAUNCH_START", "balls=" + robot.loader.getBallCount());
                    robot.missions.startLaunchPreloads();
                    setState(AutonState.WAITING_LAUNCH);
//                } else {
//                    // Nothing to launch, power down flywheel and proceed to ball collection
//                    log("LAUNCH_SKIP", "loader_empty");
//                    robot.launcher.setBehavior(Launcher.Behavior.IDLE);
//                    setState(AutonState.START_BALL_ROW);
//                }
                break;

            case WAITING_LAUNCH:
                if (robot.missions.isComplete()) {
                    log("LAUNCH_COMPLETE", null);
                    launchCycles++;
                    robot.applyVisionPoseCorrection();
                    // Power down flywheel during ball collection to conserve power
                    robot.launcher.setBehavior(Launcher.Behavior.IDLE);
                    setState(AutonState.START_BALL_ROW);
                } else if (robot.missions.isFailed()) {
                    log("LAUNCH_FAILED", "timeout");
                    launchCycles++;
                    robot.applyVisionPoseCorrection();
                    robot.launcher.setBehavior(Launcher.Behavior.IDLE);
                    setState(AutonState.START_BALL_ROW);
                }
                break;

            // ========== BALL ROW COLLECTION CYCLE ==========
            case START_BALL_ROW:
                // Selective abort - give way to partner team
                if (ABORT_AFTER_ROWS >= 0 && rowsCompleted >= ABORT_AFTER_ROWS) {
                    log("SELECTIVE_ABORT", "rows_done=" + rowsCompleted);
                    if (FieldMap.IS_AUDIENCE_START) {
                        // Go to OPPOSING alliance's base (pass !isRedAlliance to get opposite side)
                        robot.missions.startNavigateTo(ALT_POSITION_AUDIENCE, !Robot.isRedAlliance);
                    } else {
                        // Go to FIRE_2 (same alliance)
                        robot.missions.startNavigateTo(ALT_POSITION_GOAL, Robot.isRedAlliance);
                    }
                    setState(AutonState.START_ALTERNATE);
                    break;
                }

                // Check if past last row (works for both directions)
                if (ROW_DIRECTION > 0 ? currentRow > ROW_END : currentRow < ROW_END) {
                    log("ALL_ROWS_DONE", null);
                    setState(AutonState.COMPLETE);
                    break;
                }

                // Gate release before designated row
//                if (currentRow == GATE_BEFORE_ROW && DO_OPEN_SESAME && !gateReleased) {
//                    log("GATE_BEFORE_ROW", "row=" + currentRow);
//                    setState(AutonState.START_GATE);
//                    break;
//                }

                // Check if enough time for another row cycle
                if (timeRemaining < MIN_TIME_FOR_ROW) {
                    log("TIME_SKIP_ROW", "remaining=" + String.format("%.1f", timeRemaining));
                    setState(AutonState.COMPLETE);
                    break;
                }

                // Start collecting from current row
                log("BALL_ROW_START", "row=" + currentRow);
                robot.missions.startBallGroup(currentRow);
                setState(AutonState.WAITING_BALL_ROW);
                break;

            case WAITING_BALL_ROW:
                if (robot.missions.isComplete()) {
                    log("BALL_ROW_COMPLETE", "row=" + currentRow);
                    rowsCompleted++;
                    currentRow += ROW_DIRECTION;
                    setState(AutonState.START_RETURN_TO_FIRE);
                } else if (robot.missions.isFailed()) {
                    log("BALL_ROW_FAILED", "row=" + currentRow);
                    rowsCompleted++;
                    currentRow += ROW_DIRECTION;
                    setState(AutonState.START_RETURN_TO_FIRE);
                }
                break;

            case START_RETURN_TO_FIRE:
                // Navigate back to fire position (auto-direction)
                if(currentRow == 2){
                    FIRE_POSITION=2;
                }
                robot.missions.startNavigateToFire(FIRE_POSITION);
                setState(AutonState.WAITING_RETURN);
                break;

            case WAITING_RETURN:
                if (robot.missions.isComplete()) {
                    log("RETURN_COMPLETE", null);
                    setState(AutonState.START_TARGETING);
                } else if (robot.missions.isFailed()) {
                    log("RETURN_FAILED", "timeout");
                    setState(AutonState.START_TARGETING);
                }
                break;

            // ========== GATE RELEASE ==========
            case START_GATE:
                robot.missions.startOpenSesame();
                setState(AutonState.WAITING_GATE);
                break;

            case WAITING_GATE:
                if (robot.missions.isComplete()) {
                    log("GATE_COMPLETE", null);
                    gateReleased = true;
                    setState(AutonState.START_BALL_ROW);
                } else if (robot.missions.isFailed()) {
                    log("GATE_FAILED", "timeout");
                    gateReleased = true;
                    setState(AutonState.START_BALL_ROW);

                }
                break;

            // ========== SELECTIVE ABORT ==========
            case START_ALTERNATE:
                // Navigation already started in abort check
                setState(AutonState.WAITING_ALTERNATE);
                break;

            case WAITING_ALTERNATE:
                if (robot.missions.isComplete() || robot.missions.isFailed()) {
                    log("ALTERNATE_REACHED", null);
                    setState(AutonState.COMPLETE);
                }
                break;

            // ========== COMPLETE ==========
            case COMPLETE:
                // Autonomous done - robot will transition to teleop
                // Close logs on first entry to COMPLETE
                if (previousState != AutonState.COMPLETE) {
                    robot.launcher.setBehavior(Launcher.Behavior.IDLE);
                    closeLog();
                }
                break;
        }
    }

    // ==================== HELPERS ====================

    private void setState(AutonState newState) {
        if (state != newState) {
            previousState = state;
            state = newState;
            log("STATE_CHANGE", previousState + "->" + newState);
        }
    }

    /**
     * Log an event to the CSV file.
     */
    private void log(String event, String detail) {
        if (!LOGGING_ENABLED || autonLog == null) {
            return;
        }

        Pose2d pose = robot.driveTrain.getPose();
        String missionSubState = getMissionSubState();

        ArrayList<Object> row = new ArrayList<>();
        row.add(System.currentTimeMillis() - logStartTime);
        row.add(state.name());
        row.add(previousState.name());
        row.add(robot.missions.getMissionState().name());
        row.add(missionSubState);
        row.add(currentRow);
        row.add(launchCycles);
        row.add(String.format("%.1f", getTimeRemaining()));
        row.add(String.format("%.1f", pose.position.x));
        row.add(String.format("%.1f", pose.position.y));
        row.add(String.format("%.1f", Math.toDegrees(pose.heading.toDouble())));
        row.add(event + (detail != null ? ":" + detail : ""));
        autonLog.UpdateLog(row);
    }

    /**
     * Get the current mission's sub-state for logging.
     */
    private String getMissionSubState() {
        return robot.missions.getSubStateForLogging();
    }

    /**
     * Close the log files. Call when autonomous ends.
     */
    public void closeLog() {
        if (autonLog != null) {
            log("AUTON_END", null);
            autonLog.CloseLog();
            autonLog = null;
        }
        robot.missions.closeLogging();
    }

    /**
     * Get current autonomous state.
     */
    public AutonState getState() {
        return state;
    }

    /**
     * Get time remaining in autonomous period.
     */
    public double getTimeRemaining() {
        return Math.max(0, AUTON_DURATION_SECONDS - autonTimer.seconds());
    }

    // ==================== TELEMETRY ====================

    @Override
    public String getTelemetryName() {
        return "Autonomous";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("State", state);
        telemetry.put("Time Remaining", String.format("%.1fs", getTimeRemaining()));
        telemetry.put("Current Row", currentRow + 1);  // Display as 1-indexed
        telemetry.put("Launch Cycles", launchCycles);
        telemetry.put("Abort After", ABORT_AFTER_ROWS < 0 ? "ALL" : ABORT_AFTER_ROWS + " rows");

        if (debug) {
            telemetry.put("Previous State", previousState);
            telemetry.put("Gate Released", gateReleased);
            telemetry.put("Mission Active", robot.missions.isActive());
            telemetry.put("Mission State", robot.missions.getMissionState());
            telemetry.put("Start Position", START_AT_GOAL_WALL ? "Goal Wall" : "Audience");
        }

        return telemetry;
    }
}
