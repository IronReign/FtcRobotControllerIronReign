package org.firstinspires.ftc.teamcode.robots.lebot2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.lebot2.util.TelemetryProvider;

import java.util.LinkedHashMap;
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
    public static double MIN_TIME_FOR_ROW = 12.0;     // Minimum seconds needed for a ball row cycle

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

        // End
        COMPLETE
    }

    private AutonState state = AutonState.INIT;
    private AutonState previousState = AutonState.INIT;

    // ==================== PROGRESS TRACKING ====================

    private ElapsedTime autonTimer = new ElapsedTime();
    private int currentRow = 0;      // 0, 1, 2 for rows 1, 2, 3
    private int launchCycles = 0;
    private boolean gateReleased = false;

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

        // Reset state
        state = AutonState.INIT;
        previousState = AutonState.INIT;
        currentRow = 0;
        launchCycles = 0;
        gateReleased = false;

        // Timer will be reset when execute() first runs
        autonTimer.reset();
    }

    // ==================== MAIN EXECUTE ====================

    /**
     * Execute autonomous - call every loop cycle.
     */
    public void execute() {
        // Clear any completed/failed/aborted mission state
        robot.missions.clearState();

        // Time remaining for branching decisions
        double timeRemaining = AUTON_DURATION_SECONDS - autonTimer.seconds();

        switch (state) {
            case INIT:
                autonTimer.reset();
                setState(AutonState.START_BACKUP_TO_FIRE);
                break;

            // ========== PHASE 1: Backup to Fire ==========
            case START_BACKUP_TO_FIRE:
                // Back up from gate to FIRE_1, spins up launcher during navigation
                robot.missions.startNavigateToFire(1, true);
                setState(AutonState.WAITING_BACKUP);
                break;

            case WAITING_BACKUP:
                if (robot.missions.isComplete()) {
                    setState(AutonState.START_TARGETING);
                } else if (robot.missions.isFailed()) {
                    // Timeout - try to launch anyway from current position
                    setState(AutonState.START_LAUNCH);
                }
                break;

            // ========== PHASE 2-3: Target and Launch ==========
            case START_TARGETING:
                robot.setBehavior(Robot.Behavior.TARGETING);
                setState(AutonState.WAITING_TARGET);
                break;

            case WAITING_TARGET:
                if (robot.getBehavior() == Robot.Behavior.MANUAL) {
                    // Targeting complete, apply vision correction
                    robot.applyVisionPoseCorrection();
                    setState(AutonState.START_LAUNCH);
                }
                break;

            case START_LAUNCH:
                if (!robot.loader.isEmpty()) {
                    robot.missions.startLaunchPreloads();
                    setState(AutonState.WAITING_LAUNCH);
                } else {
                    // Nothing to launch, proceed to ball collection
                    setState(AutonState.START_BALL_ROW);
                }
                break;

            case WAITING_LAUNCH:
                if (robot.missions.isComplete() || robot.missions.isFailed()) {
                    launchCycles++;
                    // Apply vision correction while stationary at fire position
                    robot.applyVisionPoseCorrection();
                    setState(AutonState.START_BALL_ROW);
                }
                break;

            // ========== BALL ROW COLLECTION CYCLE ==========
            case START_BALL_ROW:
                // Check if all rows done
                if (currentRow >= 3) {
                    setState(AutonState.COMPLETE);
                    break;
                }

                // After rows 1 & 2, do gate release before row 3
                if (currentRow == 2 && DO_OPEN_SESAME && !gateReleased) {
                    setState(AutonState.START_GATE);
                    break;
                }

                // Check if enough time for another row cycle
                if (timeRemaining < MIN_TIME_FOR_ROW) {
                    setState(AutonState.COMPLETE);
                    break;
                }

                // Start collecting from current row
                robot.missions.startBallGroup(currentRow);
                setState(AutonState.WAITING_BALL_ROW);
                break;

            case WAITING_BALL_ROW:
                if (robot.missions.isComplete() || robot.missions.isFailed()) {
                    currentRow++;
                    setState(AutonState.START_RETURN_TO_FIRE);
                }
                break;

            case START_RETURN_TO_FIRE:
                // Navigate back to fire position (auto-direction)
                robot.missions.startNavigateToFire(1);
                setState(AutonState.WAITING_RETURN);
                break;

            case WAITING_RETURN:
                if (robot.missions.isComplete() || robot.missions.isFailed()) {
                    // Target and launch the collected balls
                    setState(AutonState.START_TARGETING);
                }
                break;

            // ========== GATE RELEASE ==========
            case START_GATE:
                robot.missions.startOpenSesame();
                setState(AutonState.WAITING_GATE);
                break;

            case WAITING_GATE:
                if (robot.missions.isComplete() || robot.missions.isFailed()) {
                    gateReleased = true;
                    // Now continue with row 3
                    setState(AutonState.START_BALL_ROW);
                }
                break;

            // ========== COMPLETE ==========
            case COMPLETE:
                // Autonomous done - robot will transition to teleop
                break;
        }
    }

    // ==================== HELPERS ====================

    private void setState(AutonState newState) {
        if (state != newState) {
            previousState = state;
            state = newState;
        }
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
