package org.firstinspires.ftc.teamcode.robots.lebot2;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Launcher;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.LEDStatus;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Loader;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Vision;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.drivetrain.DriveTrainBase;
import org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.TankDrivePinpoint;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.TelemetryProvider;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

/**
 * Robot coordinator class for Lebot2.
 *
 * ARCHITECTURE:
 * - Each subsystem manages its own behaviors (state machines)
 * - Robot only coordinates multi-subsystem sequences
 * - Drive is ALWAYS available (never blocked by intake/launcher)
 *
 * SUBSYSTEM BEHAVIORS:
 * - Intake: OFF, INTAKE, LOAD_ALL, EJECT (auto-completes when full)
 * - Loader: Manages belt ownership (INTAKE vs LAUNCHER priority)
 * - Launcher: IDLE, SPIN_UP, READY, FIRING, COOLDOWN (claims belt when firing)
 * - DriveTrain: MANUAL, RR_ACTION, PID_TURN (interrupted by joystick)
 *
 * ROBOT ARTICULATIONS (multi-subsystem only):
 * - MANUAL: All subsystems controlled directly by DriverControls
 * - TARGETING: Turn to target using IMU then vision
 * - LAUNCH_ALL: Fire all balls in sequence (coordinates launcher + loader)
 *
 * THREE-PHASE UPDATE CYCLE:
 * Every loop, Robot.update() executes:
 *   Phase 1: readSensors() - All I2C reads, bulk cache clear
 *   Phase 2: calc()        - Subsystem state machines
 *   Phase 3: act()         - Flush motor/servo commands
 */

@Config(value = "Lebot2_Robot")
public class Robot implements TelemetryProvider {

    // Subsystems
    public final DriveTrainBase driveTrain;
    public final Intake intake;
    public final Loader loader;
    public final Launcher launcher;
    public final Vision vision;
    public final LEDStatus ledStatus;

    // Missions coordinator (Layer 4 - above Robot behaviors)
    public final Missions missions;

    // Subsystem list for bulk operations
    public final List<Subsystem> subsystems = new ArrayList<>();

    // LynxModule hubs for bulk caching
    private final List<LynxModule> hubs;

    // Voltage monitoring
    private final VoltageSensor voltageSensor;
    private double voltage = 12.0;

    // Configuration
    public static boolean isRedAlliance = true;

    // Starting position configuration
    public enum StartingPosition {
        UNKNOWN,      // Position unknown (teleop testing) - use MT1 for initial fix
        AUDIENCE,     // Touching audience wall - can see goal AprilTag (long range)
        GOAL,         // Touching goal wall - too close for vision, must back up first
        CALIBRATION   // Field center, facing red goal - for MT1/MT2 comparison testing
    }
    private StartingPosition startingPosition = StartingPosition.UNKNOWN;
    private boolean initialPositionSet = false;  // True once we've set or corrected initial pose

    // Starting poses are now defined in FieldMap (START_AUDIENCE, START_GOAL)
    // Use FieldMap.getPose(name, isRedAlliance) to get alliance-aware poses

    // Calibration pose - field center, facing red goal
    // In DECODE coordinates: X+ toward audience, so facing red goal ≈ 135°
    // (Red goal is in the back-left quadrant from audience perspective)
    // Used for MT1/MT2 comparison testing - provides known heading to seed MT2
    public static final Pose2d CALIBRATION_POSE = new Pose2d(0, 0, 0);

    // Robot Dimensions (inches, relative to center of rotation)
    // Center of rotation = midpoint between drive wheels (on the axle line)
    // Since wheels are mounted near the back, FRONT_EXTENT > BACK_EXTENT
    public static double FRONT_EXTENT = 9.0;   // Center of rotation to front bumper
    public static double BACK_EXTENT = 5.0;    // Center of rotation to back bumper
    public static double SIDE_EXTENT = 7.0;    // Center to side (half robot width)
    public static double TRACK_WIDTH = 14.0;   // NOT USED - actual track width is hardcoded in TankDrivePinpoint.java:187

    // Camera offset from center of rotation (used to transform Limelight pose to robot center)
    // Limelight reports camera position - we need to offset to get robot center of rotation
    public static double CAMERA_FORWARD_OFFSET_M = 0.33;  // 33cm forward of center of rotation
    public static double CAMERA_SIDE_OFFSET_M = 0.0;      // Lateral offset (positive = right)
    public static double CAMERA_HEIGHT_M = 0.30;          // Height above ground (~12 inches)

    // Robot-level behaviors (multi-subsystem coordination only)
    // Note: Most behaviors are now handled by individual subsystems
    public enum Behavior {
        MANUAL,         // Subsystems controlled directly by DriverControls
        TARGETING,      // Two-phase targeting: IMU then vision
        LAUNCH_ALL      // Fire all balls in sequence
    }
    private Behavior behavior = Behavior.MANUAL;

    // Targeting sequence state
    public enum TargetingState {
        IDLE,
        TURNING_IMU,
        TURNING_VISION
    }
    private TargetingState targetingState = TargetingState.IDLE;

    // Launch-all sequence state for paddle design
//    public enum LaunchAllState {
//        IDLE,
//        SPINNING_UP,
//        FIRING,
//        WAITING_SHOT_COMPLETE,
//        INTER_SHOT_DELAY,
//        ADVANCE_BELT,
//        COMPLETE
//    }
    // Launch-all sequence state for TPU ramp design (simplified)
    public enum LaunchAllState {
        IDLE,
        SPINNING_UP,
        WAITING_READY,
        FIRING,
        COMPLETE
    }



    private LaunchAllState launchAllState = LaunchAllState.IDLE;
    private long launchTimer = 0;
    public static double INTER_SHOT_DELAY_SECONDS = 1.5;  // Delay for flywheel recovery between shots
    public static double BELT_ADVANCE_TIMEOUT_SECONDS = 0.5; // Max time to wait for next ball

    public Robot(HardwareMap hardwareMap, boolean simulated) {
        // Setup MANUAL bulk caching for best performance
        // This requires us to call clearBulkCache() at the start of each loop
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Initialize subsystems
        driveTrain = new TankDrivePinpoint(hardwareMap, new Pose2d(0, 0, 0));
        intake = new Intake(hardwareMap);
        loader = new Loader(hardwareMap);
        launcher = new Launcher(hardwareMap);
        vision = new Vision(hardwareMap);
        ledStatus = new LEDStatus(hardwareMap);

        // Connect subsystems that need references to each other
        intake.setLoader(loader);       // For LOAD_ALL completion check
        launcher.setLoader(loader);     // For belt claiming and ball counting
        launcher.setIntake(intake);     // For intake suppression during firing
        launcher.setVision(vision);     // For distance-based speed calculation
        driveTrain.setVision(vision);   // For continuous vision-based centering
        ledStatus.setLoader(loader);    // For ball detection
        ledStatus.setLauncher(launcher); // For firing detection

        // Add to subsystems list for bulk operations
        subsystems.add((Subsystem) driveTrain);
        subsystems.add(intake);
        subsystems.add(loader);
        subsystems.add(launcher);
        subsystems.add(vision);
        subsystems.add(ledStatus);

        // Voltage sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Set alliance
        vision.setAlliance(isRedAlliance);

        // Initialize missions coordinator (needs reference to this robot)
        missions = new Missions(this);
    }

    /**
     * Main update method - called every loop cycle.
     *
     * Implements the THREE-PHASE UPDATE PATTERN:
     *   Phase 1: Read all sensors (I2C, bulk cache refresh)
     *   Phase 2: Run calculations (state machines, PID, articulations)
     *   Phase 3: Write all outputs (flush lazy motors/servos)
     *
     * @param fieldOverlay Canvas for drawing on FTC Dashboard
     */
    public void update(Canvas fieldOverlay) {

        // ===== PHASE 1: READ ALL SENSORS =====
        // Clear bulk cache first - this refreshes all motor encoder/velocity data
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        // Configure Vision MT1/MT2 mode based on whether initial position is known
        // When position is unknown, force MT1 since Pinpoint heading may be wrong
        vision.setForceMT1(!initialPositionSet);

        // Feed current heading to Vision for MegaTag2 localization
        // Only useful when initialPositionSet is true (otherwise MT1 is forced)
        if (initialPositionSet) {
            vision.updateRobotOrientation(driveTrain.getHeadingDegrees());
        }

        // Read I2C sensors for all subsystems
        for (Subsystem subsystem : subsystems) {
            subsystem.readSensors();
        }

        // Update voltage (analog, not I2C, but read early)
        voltage = voltageSensor.getVoltage();

        // ===== PHASE 2: CALCULATIONS =====
        // Handle Robot-level behaviors (multi-subsystem coordination)
        switch (behavior) {
            case MANUAL:
                //handleManualBehavior();
                // Subsystems controlled directly by DriverControls
                // Nothing to coordinate at Robot level
                break;

            case TARGETING:
                handleTargetingBehavior();
                break;

            case LAUNCH_ALL:
                handleLaunchAllBehavior2();
                //handleLaunchAllBehavior();
                break;
        }

        // Update missions if active (conserves compute when idle)
        if (missions.isActive()) {
            missions.calc(fieldOverlay);
        }

        // Run calculations for all subsystems
        for (Subsystem subsystem : subsystems) {
            subsystem.calc(fieldOverlay);
        }

        // ===== PHASE 3: WRITE ALL OUTPUTS =====
        // Flush pending motor/servo commands
        for (Subsystem subsystem : subsystems) {
            subsystem.act();
        }
    }

    // ==================== BEHAVIOR HANDLERS ====================

    private void handleTargetingBehavior() {
        // Two-phase targeting: IMU rough turn, then vision fine tune
        switch (targetingState) {
            case IDLE:
                // Calculate bearing from current position to goal (alliance-aware)
                Pose2d currentPose = driveTrain.getPose();
                Pose2d goalPose = FieldMap.getPose(FieldMap.GOAL, isRedAlliance);
                double targetHeadingRadians = FieldMap.bearingTo(currentPose, goalPose);
                int targetHeadingDegrees = (int) Math.toDegrees(targetHeadingRadians);
                driveTrain.turnToHeading(targetHeadingDegrees, 0.7);
                targetingState = TargetingState.TURNING_IMU;
                break;

            case TURNING_IMU:
                if (driveTrain.isTurnComplete()) {
                    // IMU turn complete, start vision centering
                    driveTrain.centerOnTarget();
                    targetingState = TargetingState.TURNING_VISION;
                }
                break;

            case TURNING_VISION:
                if (driveTrain.isTurnComplete()) {
                    targetingState = TargetingState.IDLE;
                    behavior = Behavior.MANUAL;
                }
                break;
        }
    }

    /**
     * LAUNCH_ALL for TPU ramp design.
     * Simplified: spin up flywheel, fire once, conveyor pushes all balls through.
     * Launcher handles FIRING → LIFTING → COMPLETE sequence internally.
     */
    private void handleLaunchAllBehavior2() {
        switch (launchAllState) {
            case IDLE:
//                if (loader.isEmpty()) {
//                    behavior = Behavior.MANUAL;
//                    return;
//                }
                // Start flywheel spinning
                launcher.setBehavior(Launcher.Behavior.SPINNING);
                launchAllState = LaunchAllState.SPINNING_UP;
                break;

            case SPINNING_UP:
                // Wait for flywheel to reach speed
                if (launcher.isReady()) {
                    launchAllState = LaunchAllState.WAITING_READY;
                }
                break;

            case WAITING_READY:
                // Fire! Launcher handles the full sequence internally
                launcher.fire();
                launchAllState = LaunchAllState.FIRING;
                break;

            case FIRING:
                // Wait for launcher to complete the sequence
                Launcher.LaunchState launcherState = launcher.getState();
                if (launcherState == Launcher.LaunchState.IDLE ||
                    launcherState == Launcher.LaunchState.READY) {
                    launchAllState = LaunchAllState.COMPLETE;
                }
                break;

            case COMPLETE:
                // All done, return to manual
                launchAllState = LaunchAllState.IDLE;
                behavior = Behavior.MANUAL;
                break;
        }
    }
    // Old paddle design method - commented out for reference
//    private void handleLaunchAllBehavior() {
//        // Launch all balls in sequence until empty
//        // Simplified: no ball counting, just fire until sensors show empty
//        // Uses inter-shot delay for flywheel recovery
//        switch (launchAllState) {
//            case IDLE:
//                if (loader.isEmpty()) {
//                    behavior = Behavior.MANUAL;
//                    return;
//                }
//                // Force-reset launcher to clean state before starting
//                launcher.setBehavior(Launcher.Behavior.IDLE);
//                launcher.setBehavior(Launcher.Behavior.SPINNING);
//                launchAllState = LaunchAllState.SPINNING_UP;
//                break;
//
//            case SPINNING_UP:
//                // Wait for flywheel to reach speed (this check works reliably)
//                if (launcher.isReady()) {
//                    launchAllState = LaunchAllState.FIRING;
//                }
//                break;
//
//            case FIRING:
//                launcher.fire();
//                if (launcher.getState() == Launcher.LaunchState.COOLDOWN) {
//                    // Shot initiated, wait for it to complete
//                    launchAllState = LaunchAllState.WAITING_SHOT_COMPLETE;
//                }
//                break;
//
//            case WAITING_SHOT_COMPLETE:
//                // Wait for launcher to return to READY (shot finished)
//                if (launcher.getState() == Launcher.LaunchState.READY) {
//                    // Start inter-shot delay for flywheel recovery
//                    launchTimer = futureTime(INTER_SHOT_DELAY_SECONDS);
//                    launchAllState = LaunchAllState.INTER_SHOT_DELAY;
//                }
//                break;
//
//            case INTER_SHOT_DELAY:
//                // Wait for flywheel to recover
//                if (isPast(launchTimer)) {
//                    // Check if there might be more balls
//                    if (loader.isEmpty()) {
//                        launchAllState = LaunchAllState.COMPLETE;
//                    } else {
//                        // Advance belt to bring next ball to back
//                        launchTimer = futureTime(BELT_ADVANCE_TIMEOUT_SECONDS);
//                        launchAllState = LaunchAllState.ADVANCE_BELT;
//                    }
//                }
//                break;
//
//            case ADVANCE_BELT:
//                // Belt should be running (launcher claims belt during SPINNING)
//                // Wait for ball at back sensor or timeout
//                if (loader.isBallAtBack()) {
//                    launchAllState = LaunchAllState.FIRING;
//                } else if (isPast(launchTimer)) {
//                    // Timeout - check if truly empty
//                    if (loader.isEmpty()) {
//                        launchAllState = LaunchAllState.COMPLETE;
//                    } else {
//                        // Still has balls somewhere, try firing anyway
//                        launchAllState = LaunchAllState.FIRING;
//                    }
//                }
//                break;
//
//            case COMPLETE:
//                // Wait for launcher to finish any ongoing COOLDOWN
//                if (launcher.getState() == Launcher.LaunchState.READY ||
//                    launcher.getState() == Launcher.LaunchState.IDLE) {
//                    launcher.setBehavior(Launcher.Behavior.IDLE);
//                    launchAllState = LaunchAllState.IDLE;
//                    behavior = Behavior.MANUAL;
//                }
//                break;
//        }
//    }

//    public void handleManualBehavior(){
//        launcher.setBehavior(Launcher.Behavior.SPINNING);
//    }

    // ==================== PUBLIC METHODS ====================

    /**
     * Set the robot-level behavior (multi-subsystem coordination).
     *
     * @param newBehavior The desired behavior
     */
    public void setBehavior(Behavior newBehavior) {
        if (behavior != newBehavior) {
            // Reset state machines when switching
            targetingState = TargetingState.IDLE;
            launchAllState = LaunchAllState.IDLE;
            behavior = newBehavior;
        }
    }

    /**
     * Get current robot-level behavior.
     */
    public Behavior getBehavior() {
        return behavior;
    }

    /**
     * Check if robot is in manual mode (subsystems controlled directly).
     */
    public boolean isManualMode() {
        return behavior == Behavior.MANUAL;
    }

    /**
     * Set alliance color.
     */
    public void setAlliance(boolean isRed) {
        isRedAlliance = isRed;
        vision.setAlliance(isRed);
    }

    // ==================== STARTING POSITION ====================

    /**
     * Set starting position and initialize drivetrain pose.
     * Call during init_loop based on driver selection.
     *
     * @param position The starting position (AUDIENCE, GOAL, or UNKNOWN)
     */
    public void setStartingPosition(StartingPosition position) {
        this.startingPosition = position;

        // Update FieldMap offset selection so Dashboard shows correct waypoints
        FieldMap.IS_AUDIENCE_START = (position == StartingPosition.AUDIENCE);

        // Set appropriate fallback launch speed for the start position
        if (position == StartingPosition.AUDIENCE) {
            Launcher.MIN_LAUNCH_SPEED = FieldMap.FIRE_4_DEFAULT_DPS;
        } else {
            Launcher.MIN_LAUNCH_SPEED = FieldMap.FIRE_1_DEFAULT_DPS;
        }

        if (position == StartingPosition.UNKNOWN) {
            // Unknown position - don't set pose, wait for vision fix
            initialPositionSet = false;
        } else {
            // Get the appropriate pose based on alliance and position
            Pose2d pose = getStartingPose(position);
            driveTrain.setPose(pose);
            initialPositionSet = true;
        }
    }

    /**
     * Get the starting pose for a given position based on current alliance.
     * Uses FieldMap waypoints which handle alliance reflection automatically.
     */
    public Pose2d getStartingPose(StartingPosition position) {
        // Calibration pose is alliance-independent (field center)
        if (position == StartingPosition.CALIBRATION) {
            return CALIBRATION_POSE;
        }

        // Get pose from FieldMap - handles alliance reflection automatically
        String waypointName = (position == StartingPosition.AUDIENCE)
                ? FieldMap.START_AUDIENCE
                : FieldMap.START_GOAL;
        return FieldMap.getPose(waypointName, isRedAlliance);
    }

    /**
     * Get current starting position selection.
     */
    public StartingPosition getStartingPosition() {
        return startingPosition;
    }

    /**
     * Check if initial position has been set (either via selection or vision fix).
     * When false, Vision should prefer MT1 to get full pose including heading.
     */
    public boolean isInitialPositionSet() {
        return initialPositionSet;
    }

    /**
     * Mark initial position as set (called after successful vision correction when UNKNOWN).
     */
    public void markInitialPositionSet() {
        initialPositionSet = true;
    }

    // ==================== VISION POSE CORRECTION ====================

    /**
     * Apply vision-based pose correction to Pinpoint localizer.
     * Uses MegaTag2 when available (better single-tag accuracy with IMU fusion),
     * falls back to MegaTag1 when heading is unknown (robot restart scenario).
     *
     * The Limelight reports camera position, so we transform to robot center
     * using the camera offset constants.
     *
     * Call this when:
     * - Driver triggers correction (Back button during teleop)
     * - At autonomous checkpoints (after reaching launch position)
     *
     * @return true if correction was applied, false if no valid vision pose
     */
    public boolean applyVisionPoseCorrection() {
        if (!vision.hasBotPose()) {
            return false;
        }

        // Get vision pose - this is the CAMERA position (Limelight uses meters)
        double camX = vision.getRobotX();
        double camY = vision.getRobotY();
        double headingRad = vision.getRobotHeading();

        // Transform from camera position to robot center of rotation
        // Camera is forward of center, so robot center is BEHIND camera
        // robot_center = camera_pos - offset_in_robot_frame (rotated to field frame)
        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);

        // Forward offset: subtract because robot center is behind camera
        // Side offset: positive = camera right of center, so subtract for robot center
        double robotX = camX - CAMERA_FORWARD_OFFSET_M * cosH + CAMERA_SIDE_OFFSET_M * sinH;
        double robotY = camY - CAMERA_FORWARD_OFFSET_M * sinH - CAMERA_SIDE_OFFSET_M * cosH;

        // Convert to inches for RoadRunner
        double xInches = robotX * 39.3701;
        double yInches = robotY * 39.3701;

        Pose2d visionPose = new Pose2d(xInches, yInches, headingRad);

        driveTrain.setPose(visionPose);

        // Mark initial position as set (important for UNKNOWN starting position)
        if (!initialPositionSet) {
            initialPositionSet = true;
        }

        return true;
    }

    /**
     * Check if vision pose correction is available.
     * Use this to enable/disable the correction button or provide feedback.
     *
     * @return true if vision has a valid botpose that could be applied
     */
    public boolean canApplyVisionCorrection() {
        return vision.hasBotPose();
    }

    /**
     * Get current battery voltage.
     */
    public double getVoltage() {
        return voltage;
    }


    /**
     * Stop all subsystems and abort any running mission.
     */
    public void stop() {
        // Abort any running mission first
        if (missions.isActive()) {
            missions.abort();
        }

        for (Subsystem subsystem : subsystems) {
            subsystem.stop();
        }
    }

    /**
     * Reset all subsystem states.
     */
    public void resetStates() {
        behavior = Behavior.MANUAL;
        targetingState = TargetingState.IDLE;
        launchAllState = LaunchAllState.IDLE;
        for (Subsystem subsystem : subsystems) {
            subsystem.resetStates();
        }
    }

    // ==================== ROBOT GEOMETRY HELPERS ====================

    /**
     * Get total robot length (front to back).
     * @return Length in inches
     */
    public static double getRobotLength() {
        return FRONT_EXTENT + BACK_EXTENT;
    }

    /**
     * Get total robot width (side to side).
     * @return Width in inches
     */
    public static double getRobotWidth() {
        return SIDE_EXTENT * 2;
    }

    /**
     * Check if a field point is within the robot's footprint given current pose.
     * Useful for collision detection.
     *
     * @param pose Robot's current pose (center of rotation)
     * @param pointX Field X coordinate to check
     * @param pointY Field Y coordinate to check
     * @return true if point is inside robot bounds
     */
    public static boolean isPointInsideRobot(Pose2d pose, double pointX, double pointY) {
        // Transform point to robot-local coordinates
        double dx = pointX - pose.position.x;
        double dy = pointY - pose.position.y;
        double heading = pose.heading.toDouble();

        // Rotate to robot frame
        double localX = dx * Math.cos(-heading) - dy * Math.sin(-heading);
        double localY = dx * Math.sin(-heading) + dy * Math.cos(-heading);

        // Check bounds (localX positive = forward, localY positive = left)
        return localX >= -BACK_EXTENT && localX <= FRONT_EXTENT
                && Math.abs(localY) <= SIDE_EXTENT;
    }

    @Override
    public String getTelemetryName() {
        return "Robot";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Behavior", behavior);
        telemetry.put("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.put("Start Pos", startingPosition + (initialPositionSet ? "" : " (unset)"));
        telemetry.put("Balls", loader.getBallCount() + "/" + Loader.MAX_BALLS);

        if (debug) {
            if (behavior == Behavior.TARGETING) {
                telemetry.put("Targeting State", targetingState);
            }
            if (behavior == Behavior.LAUNCH_ALL) {
                telemetry.put("Launch State", launchAllState);
            }
            telemetry.put("Voltage", String.format("%.1f V", voltage));
            telemetry.put("Robot Size", String.format("%.1f x %.1f in", getRobotLength(), getRobotWidth()));
            telemetry.put("Extents (F/B/S)", String.format("%.1f / %.1f / %.1f in", FRONT_EXTENT, BACK_EXTENT, SIDE_EXTENT));
        }

        return telemetry;
    }
}
