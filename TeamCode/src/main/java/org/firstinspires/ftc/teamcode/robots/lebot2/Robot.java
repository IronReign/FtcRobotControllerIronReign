package org.firstinspires.ftc.teamcode.robots.lebot2;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Launcher;
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

    // Launch-all sequence state
    public enum LaunchAllState {
        IDLE,
        SPINNING_UP,
        FIRING,
        FEEDING_NEXT,
        COMPLETE
    }
    private LaunchAllState launchAllState = LaunchAllState.IDLE;
    private long launchTimer = 0;
    public static double FEED_DELAY_SECONDS = 4;
    private int shotsRemaining = 3;

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

        // Connect subsystems that need references to each other
        intake.setLoader(loader);       // For LOAD_ALL completion check
        launcher.setLoader(loader);     // For belt claiming and ball counting
        launcher.setIntake(intake);     // For intake suppression during firing
        launcher.setVision(vision);     // For distance-based speed calculation

        // Add to subsystems list for bulk operations
        subsystems.add((Subsystem) driveTrain);
        subsystems.add(intake);
        subsystems.add(loader);
        subsystems.add(launcher);
        subsystems.add(vision);

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
                // Subsystems controlled directly by DriverControls
                // Nothing to coordinate at Robot level
                break;

            case TARGETING:
                handleTargetingBehavior();
                break;

            case LAUNCH_ALL:
                handleLaunchAllBehavior();
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

    public static int targetHeadingDegrees = 45;

    private void handleTargetingBehavior() {
        // Two-phase targeting: IMU rough turn, then vision fine tune
        switch (targetingState) {
            case IDLE:
                // Start with IMU turn toward target area
                driveTrain.turnToHeading(targetHeadingDegrees, 0.7);
                targetingState = TargetingState.TURNING_IMU;
                break;

            case TURNING_IMU:
                if (driveTrain.isTurnComplete()) {
                    // IMU turn complete, start vision targeting
                    if (vision.hasTarget()) {
                        driveTrain.turnToTarget(vision.getTx(), 0.5);
                        targetingState = TargetingState.TURNING_VISION;
                    } else {
                        // No target visible, complete
                        targetingState = TargetingState.IDLE;
                        behavior = Behavior.MANUAL;
                    }
                }
                break;

            case TURNING_VISION:
                // Keep updating target while turning
                if (vision.hasTarget()) {
                    driveTrain.turnToTarget(vision.getTx(), 0.5);
                }
                if (driveTrain.isTurnComplete()) {
                    targetingState = TargetingState.IDLE;
                    behavior = Behavior.MANUAL;
                }
                break;
        }
    }

    private void handleLaunchAllBehavior() {
        // Launch all balls in sequence
        // Note: Launcher claims belt and suppresses intake automatically
        switch (launchAllState) {
            case IDLE:
                if (loader.isEmpty()) {
                    behavior = Behavior.MANUAL;
                    return;
                }
                // Start spinning up - launcher pulls distance from Vision
                launcher.setBehavior(Launcher.Behavior.SPINNING);
                shotsRemaining = loader.getBallCount();
                launchAllState = LaunchAllState.SPINNING_UP;
                break;

            case SPINNING_UP:
                if (launcher.isReady()) {
                    launchAllState = LaunchAllState.FIRING;
                }
                break;

            case FIRING:
                launcher.fire();
                if (launcher.getState() == Launcher.LaunchState.COOLDOWN) {
                    shotsRemaining--;
                    if (shotsRemaining <= 0 || loader.isEmpty()) {
                        launchAllState = LaunchAllState.COMPLETE;
                    } else {
                        // Belt is already claimed by launcher, just wait for next ball
                        launchTimer = futureTime(FEED_DELAY_SECONDS);
                        launchAllState = LaunchAllState.FEEDING_NEXT;
                    }
                }
                break;

            case FEEDING_NEXT:
                if (isPast(launchTimer) && loader.isBallAtBack()) {
                    launchAllState = LaunchAllState.FIRING;
                }
                break;

            case COMPLETE:
                launcher.setBehavior(Launcher.Behavior.IDLE);
                launchAllState = LaunchAllState.IDLE;
                behavior = Behavior.MANUAL;
                break;
        }
    }

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

    @Override
    public String getTelemetryName() {
        return "Robot";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Behavior", behavior);
        telemetry.put("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.put("Balls", loader.getBallCount() + "/" + Loader.MAX_BALLS);

        if (debug) {
            if (behavior == Behavior.TARGETING) {
                telemetry.put("Targeting State", targetingState);
            }
            if (behavior == Behavior.LAUNCH_ALL) {
                telemetry.put("Launch State", launchAllState);
                telemetry.put("Shots Remaining", shotsRemaining);
            }
            telemetry.put("Voltage", String.format("%.1f V", voltage));
        }

        return telemetry;
    }
}
