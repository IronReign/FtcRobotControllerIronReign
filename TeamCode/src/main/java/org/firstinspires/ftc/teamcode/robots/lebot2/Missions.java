package org.firstinspires.ftc.teamcode.robots.lebot2;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.TankDrivePinpoint;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.TelemetryProvider;

import java.util.LinkedHashMap;
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

    // ==================== MISSION ENUMS ====================

    public enum Mission {
        NONE,               // No mission active
        LAUNCH_PRELOADS,    // Fire preloaded balls
        BALL_GROUP,         // Navigate to ball group, intake n balls
        OPEN_SESAME,        // Navigate to classifier release pose
        GO_BALL_CLUSTER     // (future) Vision/statistical ball pickup
    }

    public enum MissionState {
        IDLE,       // No mission or mission not started
        RUNNING,    // Mission in progress
        COMPLETE,   // Mission finished successfully
        FAILED      // Mission encountered an error
    }

    // Current mission tracking
    private Mission currentMission = Mission.NONE;
    private MissionState missionState = MissionState.IDLE;

    // ==================== FIELD POSITIONS (placeholder coordinates) ====================

    // Ball group poses - 3 groups of 3 balls each, aligned on short straight lines
    // These need real field measurements
    public static Pose2d BALL_GROUP_0_POSE = new Pose2d(24, 24, Math.toRadians(0));
    public static Pose2d BALL_GROUP_1_POSE = new Pose2d(48, 24, Math.toRadians(0));
    public static Pose2d BALL_GROUP_2_POSE = new Pose2d(72, 24, Math.toRadians(0));

    // Classifier release pose - where to back into the release lever
    public static Pose2d CLASSIFIER_RELEASE_POSE = new Pose2d(0, 48, Math.toRadians(180));

    // Post-release cluster pose - where balls tend to accumulate after release
    public static Pose2d POST_RELEASE_CLUSTER_POSE = new Pose2d(12, 60, Math.toRadians(45));

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
        WAITING_FOR_LAUNCH
    }
    private LaunchPreloadsState launchPreloadsState = LaunchPreloadsState.IDLE;

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
        NAVIGATING_TO_GROUP,
        INTAKING,
        COMPLETE
    }
    private BallGroupState ballGroupState = BallGroupState.IDLE;
    private int ballCountAtStart = 0;

    // RoadRunner action tracking
    private Action currentAction = null;
    private TelemetryPacket actionPacket = new TelemetryPacket();

    // ==================== CONSTRUCTOR ====================

    public Missions(Robot robot) {
        this.robot = robot;
    }

    // ==================== MISSION CONTROL ====================

    /**
     * Start the LaunchPreloads mission.
     * Fires any balls that were preloaded at match start.
     */
    public void startLaunchPreloads() {
        if (missionState == MissionState.RUNNING) {
            return;  // Already running a mission
        }
        currentMission = Mission.LAUNCH_PRELOADS;
        missionState = MissionState.RUNNING;
        launchPreloadsState = LaunchPreloadsState.IDLE;
    }

    /**
     * Start the BallGroup mission.
     * Navigates to specified ball group and intakes balls.
     *
     * @param groupIndex Which group to collect (0, 1, or 2)
     */
    public void startBallGroup(int groupIndex) {
        if (missionState == MissionState.RUNNING) {
            return;
        }
        targetGroupIndex = Math.max(0, Math.min(2, groupIndex));
        currentMission = Mission.BALL_GROUP;
        missionState = MissionState.RUNNING;
        ballGroupState = BallGroupState.IDLE;
        ballCountAtStart = robot.loader.getBallCount();
    }

    /**
     * Start the OpenSesame mission.
     * Navigates to classifier release pose and backs into the lever.
     */
    public void startOpenSesame() {
        if (missionState == MissionState.RUNNING) {
            return;
        }
        currentMission = Mission.OPEN_SESAME;
        missionState = MissionState.RUNNING;
        openSesameState = OpenSesameState.IDLE;
    }

    /**
     * Abort the current mission immediately.
     */
    public void abort() {
        // Cancel any running trajectory
        if (robot.driveTrain instanceof TankDrivePinpoint) {
            ((TankDrivePinpoint) robot.driveTrain).cancelAction();
        }
        robot.driveTrain.drive(0, 0, 0);

        // Reset to manual control
        robot.setBehavior(Robot.Behavior.MANUAL);
        robot.intake.off();

        // Clear mission state
        currentMission = Mission.NONE;
        missionState = MissionState.IDLE;
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
     * Check if the current mission failed.
     */
    public boolean isFailed() {
        return missionState == MissionState.FAILED;
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
     * Clear completion/failure state (call before starting next mission).
     */
    public void clearState() {
        if (missionState == MissionState.COMPLETE || missionState == MissionState.FAILED) {
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

            case NONE:
            default:
                missionState = MissionState.IDLE;
                break;
        }
    }

    // ==================== MISSION IMPLEMENTATIONS ====================

    private void updateLaunchPreloadsMission() {
        switch (launchPreloadsState) {
            case IDLE:
                // Check if we have balls to launch
                if (robot.loader.isEmpty()) {
                    missionState = MissionState.COMPLETE;
                    return;
                }
                // Start the LAUNCH_ALL robot behavior
                robot.setBehavior(Robot.Behavior.LAUNCH_ALL);
                launchPreloadsState = LaunchPreloadsState.WAITING_FOR_LAUNCH;
                break;

            case WAITING_FOR_LAUNCH:
                // Wait for Robot.Behavior.LAUNCH_ALL to complete
                // Robot returns to MANUAL when launch sequence is done
                if (robot.getBehavior() == Robot.Behavior.MANUAL) {
                    missionState = MissionState.COMPLETE;
                    launchPreloadsState = LaunchPreloadsState.IDLE;
                }
                break;
        }
    }

    private void updateOpenSesameMission() {
        TankDrivePinpoint driveTrain = (TankDrivePinpoint) robot.driveTrain;

        switch (openSesameState) {
            case IDLE:
                // Build trajectory to classifier release pose
                Pose2d currentPose = robot.driveTrain.getPose();
                Action trajectory = driveTrain.actionBuilder(currentPose)
                        .splineToLinearHeading(CLASSIFIER_RELEASE_POSE, CLASSIFIER_RELEASE_POSE.heading.toDouble())
                        .build();
                driveTrain.runAction(trajectory);
                openSesameState = OpenSesameState.NAVIGATING;
                break;

            case NAVIGATING:
                // Update trajectory action
                actionPacket = new TelemetryPacket();
                if (!driveTrain.updateAction(actionPacket)) {
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

        switch (ballGroupState) {
            case IDLE:
                // Get target pose for the specified group
                Pose2d targetPose = getBallGroupPose(targetGroupIndex);

                // Build trajectory to ball group
                Pose2d currentPose = robot.driveTrain.getPose();
                Action trajectory = driveTrain.actionBuilder(currentPose)
                        .splineToLinearHeading(targetPose, targetPose.heading.toDouble())
                        .build();
                driveTrain.runAction(trajectory);
                ballGroupState = BallGroupState.NAVIGATING_TO_GROUP;
                break;

            case NAVIGATING_TO_GROUP:
                // Update trajectory action
                actionPacket = new TelemetryPacket();
                if (!driveTrain.updateAction(actionPacket)) {
                    // Arrived at ball group, start intake
                    robot.intake.loadAll();
                    robot.loader.requestBeltForIntake();
                    ballGroupState = BallGroupState.INTAKING;
                }
                break;

            case INTAKING:
                // Drive slowly forward while intaking
                robot.driveTrain.drive(0.15, 0, 0);

                // Check if we've collected enough balls
                int ballsCollected = robot.loader.getBallCount() - ballCountAtStart;
                if (ballsCollected >= BALLS_PER_GROUP || robot.loader.isFull()) {
                    robot.driveTrain.drive(0, 0, 0);
                    robot.intake.off();
                    robot.loader.releaseBeltFromIntake();
                    ballGroupState = BallGroupState.COMPLETE;
                }
                break;

            case COMPLETE:
                missionState = MissionState.COMPLETE;
                ballGroupState = BallGroupState.IDLE;
                break;
        }
    }

    private Pose2d getBallGroupPose(int groupIndex) {
        switch (groupIndex) {
            case 0: return BALL_GROUP_0_POSE;
            case 1: return BALL_GROUP_1_POSE;
            case 2: return BALL_GROUP_2_POSE;
            default: return BALL_GROUP_0_POSE;
        }
    }

    private void resetMissionStates() {
        launchPreloadsState = LaunchPreloadsState.IDLE;
        openSesameState = OpenSesameState.IDLE;
        ballGroupState = BallGroupState.IDLE;
        currentAction = null;
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

        if (debug) {
            telemetry.put("Group Order", java.util.Arrays.toString(ballGroupOrder));
            telemetry.put("Current Group Index", currentGroupIndex);

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
        }

        return telemetry;
    }
}
