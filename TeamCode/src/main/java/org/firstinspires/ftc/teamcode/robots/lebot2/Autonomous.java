package org.firstinspires.ftc.teamcode.robots.lebot2;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.robots.lebot2.util.TelemetryProvider;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Autonomous strategy coordinator for Lebot2.
 *
 * ARCHITECTURE:
 * - Autonomous handles STRATEGIC decisions (which missions, in what order)
 * - Missions handles TACTICAL execution (how to execute each mission)
 * - This separation allows match-specific strategy adjustments based on:
 *   - Starting position (audience vs goal wall)
 *   - Partner capabilities (which ball groups they handle)
 *   - Opponent analysis
 *
 * AUTONOMOUS PHASES:
 * 1. Launch preloaded balls (LaunchPreloads mission)
 * 2. Collect ball groups (BallGroup missions, order configurable)
 * 3. Release classifier (OpenSesame mission) after 9 balls scored
 * 4. Collect released ball cluster
 * 5. Repeat targeting/launching as time permits
 */
@Config(value = "Lebot2_Autonomous")
public class Autonomous implements TelemetryProvider {

    private final Robot robot;

    // Strategic configuration (set during match setup)
    public static boolean START_AT_AUDIENCE_WALL = true;  // vs goal wall
    public static int[] BALL_GROUP_ORDER = {0, 1, 2};     // Which groups, in order
    public static boolean DO_OPEN_SESAME = true;          // Release classifier?

    // Autonomous sequence states
    public enum AutonPhase {
        INIT,                   // Initialize, configure missions
        LAUNCH_PRELOADS,        // Fire any preloaded balls
        WAITING_PRELOADS,       // Wait for launch to complete
        BALL_GROUP,             // Collect from next ball group
        WAITING_BALL_GROUP,     // Wait for ball group collection
        TARGET_AND_LAUNCH,      // Target goal and launch collected balls
        WAITING_LAUNCH,         // Wait for launch to complete
        OPEN_SESAME,            // Navigate to classifier release
        WAITING_OPEN_SESAME,    // Wait for release to complete
        POST_RELEASE_COLLECT,   // Collect balls from release cluster
        WAITING_POST_COLLECT,   // Wait for collection
        END                     // Autonomous complete
    }

    private AutonPhase phase = AutonPhase.INIT;
    private AutonPhase previousPhase = AutonPhase.INIT;

    // Progress tracking
    private int ballGroupsCompleted = 0;
    private int launchCycles = 0;
    private boolean targetingStarted = false;  // For TARGET_AND_LAUNCH phase

    public Autonomous(Robot robot) {
        this.robot = robot;
    }

    /**
     * Initialize autonomous with current configuration.
     * Call this during init phase before starting.
     */
    public void init() {
        // Configure missions based on starting position
        if (START_AT_AUDIENCE_WALL) {
            robot.missions.setBallGroupOrder(BALL_GROUP_ORDER);
        } else {
            // Reverse order for goal wall start
            int[] reversed = {BALL_GROUP_ORDER[2], BALL_GROUP_ORDER[1], BALL_GROUP_ORDER[0]};
            robot.missions.setBallGroupOrder(reversed);
        }
        robot.missions.resetGroupProgress();

        ballGroupsCompleted = 0;
        launchCycles = 0;
        targetingStarted = false;
        phase = AutonPhase.INIT;
    }

    /**
     * Execute autonomous - call this every loop cycle.
     */
    public void execute() {
        // Clear any completed mission state before checking
        robot.missions.clearState();

        switch (phase) {
            case INIT:
                // Start by launching any preloaded balls
                setPhase(AutonPhase.LAUNCH_PRELOADS);
                break;

            case LAUNCH_PRELOADS:
                // Start the launch preloads mission
                if (!robot.missions.isActive()) {
                    if (!robot.loader.isEmpty()) {
                        robot.missions.startLaunchPreloads();
                        setPhase(AutonPhase.WAITING_PRELOADS);
                    } else {
                        // No preloads, skip to ball group collection
                        setPhase(AutonPhase.BALL_GROUP);
                    }
                }
                break;

            case WAITING_PRELOADS:
                if (robot.missions.isComplete()) {
                    launchCycles++;
                    // Good time for vision correction - robot is stationary, facing goal
                    robot.applyVisionPoseCorrection();
                    setPhase(AutonPhase.BALL_GROUP);
                } else if (robot.missions.isFailed()) {
                    // Mission failed, try to continue
                    setPhase(AutonPhase.BALL_GROUP);
                }
                break;

            case BALL_GROUP:
                // Get next ball group to collect
                int nextGroup = robot.missions.getNextBallGroup();
                if (nextGroup >= 0) {
                    robot.missions.startBallGroup(nextGroup);
                    setPhase(AutonPhase.WAITING_BALL_GROUP);
                } else {
                    // All groups collected, check if we should release classifier
                    if (DO_OPEN_SESAME && ballGroupsCompleted >= 3) {
                        setPhase(AutonPhase.OPEN_SESAME);
                    } else {
                        setPhase(AutonPhase.END);
                    }
                }
                break;

            case WAITING_BALL_GROUP:
                if (robot.missions.isComplete()) {
                    robot.missions.advanceToNextGroup();
                    ballGroupsCompleted++;
                    // After collecting, launch the balls
                    setPhase(AutonPhase.TARGET_AND_LAUNCH);
                } else if (robot.missions.isFailed()) {
                    // Skip this group, try next
                    robot.missions.advanceToNextGroup();
                    setPhase(AutonPhase.BALL_GROUP);
                }
                break;

            case TARGET_AND_LAUNCH:
                // Target and launch collected balls
                if (!robot.loader.isEmpty()) {
                    if (!targetingStarted) {
                        // Start targeting
                        robot.setBehavior(Robot.Behavior.TARGETING);
                        targetingStarted = true;
                    } else if (robot.getBehavior() == Robot.Behavior.MANUAL) {
                        // Targeting complete (returned to MANUAL), now launch
                        targetingStarted = false;
                        robot.missions.startLaunchPreloads();
                        setPhase(AutonPhase.WAITING_LAUNCH);
                    }
                    // else: still targeting, wait
                } else {
                    // Nothing to launch, continue to next group
                    targetingStarted = false;
                    setPhase(AutonPhase.BALL_GROUP);
                }
                break;

            case WAITING_LAUNCH:
                if (robot.missions.isComplete()) {
                    launchCycles++;
                    // Good time for vision correction - robot is stationary, facing goal
                    robot.applyVisionPoseCorrection();
                    // Continue to next ball group
                    setPhase(AutonPhase.BALL_GROUP);
                } else if (robot.missions.isFailed()) {
                    setPhase(AutonPhase.BALL_GROUP);
                }
                break;

            case OPEN_SESAME:
                // Navigate to classifier and release balls
                robot.missions.startOpenSesame();
                setPhase(AutonPhase.WAITING_OPEN_SESAME);
                break;

            case WAITING_OPEN_SESAME:
                if (robot.missions.isComplete()) {
                    // After release, collect from cluster
                    setPhase(AutonPhase.POST_RELEASE_COLLECT);
                } else if (robot.missions.isFailed()) {
                    setPhase(AutonPhase.END);
                }
                break;

            case POST_RELEASE_COLLECT:
                // TODO: Implement ball cluster collection (vision-based or statistical)
                // For now, just end
                setPhase(AutonPhase.END);
                break;

            case WAITING_POST_COLLECT:
                if (robot.missions.isComplete() || robot.missions.isFailed()) {
                    // Final launch if we collected anything
                    if (!robot.loader.isEmpty()) {
                        robot.missions.startLaunchPreloads();
                        // Then end
                    }
                    setPhase(AutonPhase.END);
                }
                break;

            case END:
                // Autonomous complete - wait for teleop
                break;
        }
    }

    private void setPhase(AutonPhase newPhase) {
        if (phase != newPhase) {
            previousPhase = phase;
            phase = newPhase;
        }
    }

    /**
     * Get current autonomous phase.
     */
    public AutonPhase getPhase() {
        return phase;
    }

    @Override
    public String getTelemetryName() {
        return "Autonomous";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Phase", phase);
        telemetry.put("Ball Groups Done", ballGroupsCompleted);
        telemetry.put("Launch Cycles", launchCycles);

        if (debug) {
            telemetry.put("Previous Phase", previousPhase);
            telemetry.put("Mission Active", robot.missions.isActive());
            telemetry.put("Mission State", robot.missions.getMissionState());
            telemetry.put("Start Position", START_AT_AUDIENCE_WALL ? "Audience" : "Goal");
        }

        return telemetry;
    }
}
