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
        NAVIGATE_TO_FIRE,   // Navigate to fire position (can be reversed)
        LAUNCH_PRELOADS,    // Fire preloaded balls
        BALL_GROUP,         // Navigate to ball row, intake through row
        OPEN_SESAME,        // Navigate to gate, back into release lever
        GO_BALL_CLUSTER     // (future) Vision/statistical ball pickup
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
    private TelemetryPacket actionPacket = new TelemetryPacket();

    // ==================== CONSTRUCTOR ====================

    public Missions(Robot robot) {
        this.robot = robot;
    }

    // ==================== MISSION CONTROL ====================

    /**
     * Start the NavigateToFire mission with auto-direction.
     * Calculates whether forward or reverse requires less turning and uses that.
     * Also spins up the launcher during navigation.
     *
     * @param firePosition Which fire position (1-4)
     */
    public void startNavigateToFire(int firePosition) {
        if (missionState == MissionState.RUNNING) {
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
        if (missionState == MissionState.RUNNING) {
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
        if (missionState == MissionState.RUNNING) {
            return;  // Already running a mission
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
        if (missionState == MissionState.RUNNING) {
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
        if (missionState == MissionState.RUNNING) {
            return;
        }
        currentMission = Mission.OPEN_SESAME;
        missionState = MissionState.RUNNING;
        openSesameState = OpenSesameState.IDLE;
        missionTimer.reset();
    }

    /**
     * Abort the current mission immediately.
     * After aborting, the same mission can be restarted from the current position.
     *
     * Usage pattern for collision avoidance:
     *   robot.missions.abort();           // Stop immediately
     *   // ... handle obstacle ...
     *   robot.missions.clearState();      // Clear ABORTED state
     *   robot.missions.startNavigateToFire(1, false);  // Restart from current position
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
     * Clear terminal state (complete/failed/aborted) so a new mission can start.
     * Call before checking status each loop, or before restarting after abort.
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
            robot.driveTrain.drive(0, 0, 0);
            robot.launcher.stop();
            missionState = MissionState.FAILED;
            navToFireState = NavigateToFireState.IDLE;
            return;
        }

        switch (navToFireState) {
            case IDLE:
                // Start spinning up launcher during navigation
                robot.launcher.spinUp();

                // Get target fire position from FieldMap
                Pose2d firePose = getFirePose(targetFirePosition);
                Pose2d currentPose = robot.driveTrain.getPose();

                // Build trajectory - use setReversed for backing up
                Action trajectory;
                if (navReversed) {
                    trajectory = driveTrain.actionBuilder(currentPose)
                            .setReversed(true)
                            .splineTo(firePose.position, firePose.heading)
                            .build();
                } else {
                    trajectory = driveTrain.actionBuilder(currentPose)
                            .turnTo(FieldMap.bearingTo(currentPose, firePose))
                            .splineTo(firePose.position, firePose.heading)
                            .build();
                }
                driveTrain.runAction(trajectory);
                navToFireState = NavigateToFireState.NAVIGATING;
                break;

            case NAVIGATING:
                // Update trajectory action
                actionPacket = new TelemetryPacket();
                if (!driveTrain.updateAction(actionPacket)) {
                    // Arrived at fire position
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
            robot.setBehavior(Robot.Behavior.MANUAL);
            missionState = MissionState.FAILED;
            launchPreloadsState = LaunchPreloadsState.IDLE;
            return;
        }

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
                Pose2d currentPose = robot.driveTrain.getPose();

                // Build tank-compatible trajectory: turn toward target, drive, turn to final heading
                Action trajectory = driveTrain.actionBuilder(currentPose)
                        .turnTo(FieldMap.bearingTo(currentPose, gatePose))
                        .splineTo(gatePose.position, gatePose.heading)
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

        // Check for timeout
        if (missionTimer.seconds() > NAVIGATION_TIMEOUT_SECONDS + INTAKE_TIMEOUT_SECONDS) {
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

                // Build trajectory to row start
                Pose2d currentPose = robot.driveTrain.getPose();
                Action trajectory = driveTrain.actionBuilder(currentPose)
                        .turnTo(FieldMap.bearingTo(currentPose, rowStart))
                        .splineTo(rowStart.position, rowStart.heading)
                        .build();
                driveTrain.runAction(trajectory);
                ballGroupState = BallGroupState.NAVIGATING_TO_ROW_START;
                break;

            case NAVIGATING_TO_ROW_START:
                // Update trajectory action
                actionPacket = new TelemetryPacket();
                if (!driveTrain.updateAction(actionPacket)) {
                    // Arrived at row start, begin intake run through row
                    robot.intake.loadAll();
                    robot.loader.requestBeltForIntake();

                    // Build trajectory to drive through row
                    Pose2d nowPose = robot.driveTrain.getPose();
                    Action intakeTrajectory = driveTrain.actionBuilder(nowPose)
                            .lineToY(targetRowEnd.position.y)
                            .build();
                    driveTrain.runAction(intakeTrajectory);
                    ballGroupState = BallGroupState.INTAKING_THROUGH_ROW;
                }
                break;

            case INTAKING_THROUGH_ROW:
                // Update trajectory while intaking
                actionPacket = new TelemetryPacket();
                boolean trajectoryComplete = !driveTrain.updateAction(actionPacket);

                // Check if we've collected enough balls or reached end of row
                int ballsCollected = robot.loader.getBallCount() - ballCountAtStart;
                if (trajectoryComplete || ballsCollected >= BALLS_PER_GROUP || robot.loader.isFull()) {
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
        }

        return telemetry;
    }
}
