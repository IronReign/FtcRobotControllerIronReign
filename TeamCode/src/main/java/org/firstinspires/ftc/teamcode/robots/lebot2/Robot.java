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
 * This class:
 * - Owns all subsystems
 * - Coordinates multi-subsystem behaviors (Articulations)
 * - Handles the main update loop with THREE-PHASE PATTERN
 * - Provides consolidated telemetry
 *
 * THREE-PHASE UPDATE CYCLE:
 * Every loop, Robot.update() executes:
 *   Phase 1: readSensors() - All I2C reads, bulk cache clear
 *   Phase 2: calc()        - State machines, articulations
 *   Phase 3: act()         - Flush motor/servo commands
 *
 * This pattern ensures all sensor reads happen before any calculations,
 * preventing stale data bugs and optimizing I2C bandwidth.
 *
 * The Robot does NOT handle gamepad input directly - that's DriverControls' job.
 * The Robot provides articulations that DriverControls (or Autonomous) can trigger.
 */

@Config(value = "Lebot2_Robot")
public class Robot implements TelemetryProvider {

    // Subsystems
    public final DriveTrainBase driveTrain;
    public final Intake intake;
    public final Loader loader;
    public final Launcher launcher;
    public final Vision vision;

    // Subsystem list for bulk operations
    public final List<Subsystem> subsystems = new ArrayList<>();

    // LynxModule hubs for bulk caching
    private final List<LynxModule> hubs;

    // Voltage monitoring
    private final VoltageSensor voltageSensor;
    private double voltage = 12.0;

    // Configuration
    public static boolean isRedAlliance = true;

    // High-level robot states (Articulations)
    public enum Articulation {
        MANUAL,         // Driver has full control
        INTAKE,         // Actively gathering balls
        TRANSIT,        // Driving to launch position
        TARGETING,      // Aligning with target
        LAUNCHING,      // Firing balls
        LAUNCH_ALL,      // Fire all balls in sequence
        RELEASE          // Release balls from classifier - navigation TBD
    }
    private Articulation articulation = Articulation.MANUAL;
    private Articulation previousArticulation = Articulation.MANUAL;

    // Launch sequence state (for LAUNCHING and LAUNCH_ALL)
    public enum LaunchSequenceState {
        IDLE,
        SPINNING_UP,
        TARGETING_IMU,
        TARGETING_VISION,
        FIRING,
        FEEDING_NEXT,
        COMPLETE
    }
    public LaunchSequenceState launchSequenceState = LaunchSequenceState.IDLE;
    private long launchTimer = 0;
    public static double FEED_DELAY_SECONDS = 4;

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
        launcher.setLoader(loader);

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
        // Handle articulation state machine (Robot-level behavior)
        switch (articulation) {
            case MANUAL:
                // Subsystems controlled directly by DriverControls
                break;

            case INTAKE:
                handleIntakeArticulation();
                break;

            case TRANSIT:
                handleTransitArticulation();
                break;

            case TARGETING:
                handleTargetingArticulation();
                break;

            case LAUNCHING:
                handleLaunchingArticulation();
                break;

            case LAUNCH_ALL:
                handleLaunchAllArticulation();
                break;
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

    // ==================== ARTICULATION HANDLERS ====================

    public void handleIntakeArticulation() {
        // Run intake and loader belt together
        intake.on();
        loader.runBelt();

        // Auto-stop when full
        if (loader.isFull()) {
            intake.off();
            loader.stopBelt();
            articulate(Articulation.MANUAL);
            shotsRemaining = 3;
        }
    }

    private void handleTransitArticulation() {
        // Everything off, just driving
        intake.off();
        loader.stopBelt();
    }

    public int headingDegrees = 45;

    public void handleTargetingArticulation() {
        // Two-phase targeting: IMU rough turn, then vision fine tune
        switch (launchSequenceState) {
            case IDLE:
                // Start with IMU turn toward target area
                // TODO: Get target heading from field position
                driveTrain.turnToHeading(headingDegrees, 0.7);
                launchSequenceState = LaunchSequenceState.TARGETING_IMU;
                break;

            case TARGETING_IMU:
                if (driveTrain.isTurnComplete()) {
                    launchSequenceState = LaunchSequenceState.TARGETING_VISION;
                }
                break;

            case TARGETING_VISION:
                if (vision.hasTarget()) {
                    driveTrain.turnToTarget(vision.getTx(), 0.5);
                    if (driveTrain.isTurnComplete()) {
                        launchSequenceState = LaunchSequenceState.IDLE;
                        articulate(Articulation.MANUAL);
                    }
                } else {
                    // No target visible, give up
                    launchSequenceState = LaunchSequenceState.IDLE;
                    articulate(Articulation.MANUAL);
                }
                break;

            default:
                launchSequenceState = LaunchSequenceState.IDLE;
        }
    }

    private void handleLaunchingArticulation() {
        // Single ball launch sequence
        switch (launchSequenceState) {
            case IDLE:
                if (vision.hasTarget()) {
                    launcher.prepareToLaunch(vision.getDistanceToTarget());
                    launchSequenceState = LaunchSequenceState.SPINNING_UP;
                } else {
                    // No target, use default speed
                    launcher.prepareToLaunchAtSpeed(Launcher.MIN_LAUNCH_SPEED);
                    launchSequenceState = LaunchSequenceState.SPINNING_UP;
                }
                break;

            case SPINNING_UP:
                if (true) {
                    launchSequenceState = LaunchSequenceState.FIRING;
                }
                break;

            case FIRING:
                launcher.fire();
                if (launcher.getState() == Launcher.LaunchState.COOLDOWN) {
                    launchSequenceState = LaunchSequenceState.COMPLETE;
                }
                break;

            case COMPLETE:
                //launcher.abort();
                launchSequenceState = LaunchSequenceState.IDLE;
                articulate(Articulation.MANUAL);
                break;

            default:
                launchSequenceState = LaunchSequenceState.IDLE;
        }
    }

    private int shotsRemaining = 3;

    public void handleLaunchAllArticulation() {
        // Launch all balls in sequence
        switch (launchSequenceState) {
            case IDLE:
                if (loader.isEmpty()) {
                    // No balls to launch
                    articulate(Articulation.MANUAL);
                    return;
                }
                if (vision.hasTarget()) {
                    launcher.prepareToLaunch(vision.getDistanceToTarget());
                } else {
                    launcher.prepareToLaunchAtSpeed(Launcher.MIN_LAUNCH_SPEED);
                }
                launchSequenceState = LaunchSequenceState.SPINNING_UP;
                break;

            case SPINNING_UP:
                if (launcher.isReady()) { //&& loader.isBallAtBack()
                    launchSequenceState = LaunchSequenceState.FIRING;
                }/* else if (launcher.isReady()) {
                    // Need to feed a ball first
                    loader.feedBall();
                    launchSequenceState = LaunchSequenceState.FEEDING_NEXT;
                    launchTimer = futureTime(1.0);
                }*/
                break;

            case FIRING:
                launcher.fire();
                if (launcher.getState() == Launcher.LaunchState.COOLDOWN) {
                    shotsRemaining--;
                    if (shotsRemaining <= 0 || loader.isEmpty()) {
                        launchSequenceState = LaunchSequenceState.COMPLETE;
                    } else {
                        loader.feedBall();
                        launchTimer = futureTime(FEED_DELAY_SECONDS);
                        launchSequenceState = LaunchSequenceState.FEEDING_NEXT;
                    }
                }
                break;

            case FEEDING_NEXT:
                if (isPast(launchTimer) && loader.isBallAtBack()) {
                    loader.stopBelt();
                    launchSequenceState = LaunchSequenceState.FIRING;
                }/* else if (isPast(launchTimer) && loader.isEmpty()) {
                    // No more balls
                    loader.stopBelt();
                    launchSequenceState = LaunchSequenceState.COMPLETE;
                }*/
                break;

            case COMPLETE:
                launcher.abort();
                loader.stopBelt();
                launchSequenceState = LaunchSequenceState.IDLE;
                articulate(Articulation.MANUAL);
                break;

            /*default:
                launchSequenceState = LaunchSequenceState.IDLE;*/
        }
    }

    // ==================== PUBLIC METHODS ====================

    /**
     * Request a new articulation (high-level robot behavior).
     *
     * @param newArticulation The desired articulation
     */
    public void articulate(Articulation newArticulation) {
        if (articulation != newArticulation) {
            previousArticulation = articulation;
            articulation = newArticulation;
            launchSequenceState = LaunchSequenceState.IDLE;
        }
    }

    /**
     * Get current articulation.
     */
    public Articulation getArticulation() {
        return articulation;
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
     * Stop all subsystems.
     */
    public void stop() {
        for (Subsystem subsystem : subsystems) {
            subsystem.stop();
        }
    }

    /**
     * Reset all subsystem states.
     */
    public void resetStates() {
        articulation = Articulation.MANUAL;
        launchSequenceState = LaunchSequenceState.IDLE;
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

        telemetry.put("Articulation", articulation);
        telemetry.put("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.put("Balls", loader.getBallCount() + "/" + Loader.MAX_BALLS);

        if (debug) {
            telemetry.put("Launch Seq State", launchSequenceState);
            telemetry.put("Voltage", String.format("%.1f V", voltage));
        }

        return telemetry;
    }
}
