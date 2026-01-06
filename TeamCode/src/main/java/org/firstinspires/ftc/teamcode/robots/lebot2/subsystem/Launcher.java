package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.LazyServo;

import java.util.LinkedHashMap;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

/**
 * Launcher subsystem - controls the flywheel and paddle for launching balls.
 *
 * The launcher consists of:
 * - Flywheel motor: spins up to calculated speed based on target distance
 * - Paddle servo: lifts balls into contact with the flywheel
 *
 * BEHAVIOR PATTERN:
 * User sets behavior via setBehavior():
 * - IDLE: Flywheel off, paddle down (default)
 * - SPINNING: Flywheel at speed, ready to fire. Speed auto-calculated from Vision.
 *
 * Call fire() when behavior is SPINNING and isReady() returns true.
 *
 * INTERNAL STATE MACHINE (LaunchState):
 * IDLE → SPINNING_UP → READY → FIRING → COOLDOWN → (back to READY or IDLE)
 *
 * The internal state handles the complexity of spin-up timing, firing sequence,
 * and cooldown. Users only interact with Behavior and fire().
 *
 * THREE-PHASE UPDATE:
 * - readSensors(): Nothing needed (motor velocity is SDK bulk-cached)
 * - calc(): Update target speed from Vision, run state machine
 * - act(): Flush servo command
 */
@Config(value = "Lebot2_Launcher")
public class Launcher implements Subsystem {

    // Hardware
    private final DcMotorEx flywheel;  // Not wrapped - SDK handles velocity control
    private final LazyServo paddle;    // Wrapped for three-phase pattern

    // Paddle positions (servo units 0-1, converted from pulse widths)
    public static double PADDLE_DOWN = 0.167;  // ~1000µs - gate closed
    public static double PADDLE_UP = 0.447;    // ~1420µs - launch position
    public static double PADDLE_PASS = 0.447;  // ~1420µs - pass-through mode

    // Flywheel configuration
    public static double MIN_LAUNCH_SPEED = 935;   // degrees/sec
    public static double SPEED_TOLERANCE = 15;      // degrees/sec margin
    public static double FLYWHEEL_SPINDOWN_TIME = 0.5; // seconds

    // Launch timing
    public static double PADDLE_LIFT_TIME = 0.35;   // seconds to hold paddle up
    public static double COOLDOWN_TIME = 0.25;      // seconds after paddle down

    // Speed calculation constants (from physics model)
    public static double LIMELIGHT_OFFSET = 0.23;   // meters from launch point
    public static double LAUNCH_ANGLE = 60;         // degrees
    public static double TARGET_HEIGHT = 0.711;     // meters

    // ==================== BEHAVIOR (user-facing) ====================

    /**
     * User-selectable launcher behaviors.
     * Set via setBehavior(), query via getBehavior().
     */
    public enum Behavior {
        IDLE,       // Flywheel off, paddle down (default)
        SPINNING    // Flywheel at speed, ready to fire
    }
    private Behavior behavior = Behavior.IDLE;

    // ==================== INTERNAL STATE MACHINE ====================

    /**
     * Internal state for launch sequence execution.
     * Users don't set this directly - it's managed by the behavior.
     */
    public enum LaunchState {
        IDLE,           // Flywheel off, paddle down
        SPINNING_UP,    // Flywheel ramping to speed
        READY,          // Flywheel at speed, waiting for fire command
        FIRING,         // Paddle lifting ball into flywheel
        COOLDOWN        // Paddle returning, preparing for next
    }
    private LaunchState state = LaunchState.IDLE;

    // State data
    private double targetSpeed = MIN_LAUNCH_SPEED;
    private double currentSpeed = 0;
    private long stateTimer = 0;
    private boolean fireRequested = false;
    private boolean passThroughMode = false;

    // References to other subsystems for coordination
    private Loader loader = null;
    private Intake intake = null;
    private Vision vision = null;

    // Track if we've claimed resources this firing cycle
    private boolean resourcesClaimed = false;

    public Launcher(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        paddle = new LazyServo(hardwareMap, "paddle");
        paddle.setPosition(PADDLE_DOWN);
        paddle.flush();  // Apply initial position immediately
    }

    /**
     * Set reference to Loader for coordinated operations.
     * Call this after all subsystems are created.
     */
    public void setLoader(Loader loader) {
        this.loader = loader;
    }

    /**
     * Set reference to Intake for suppression during firing.
     * Call this after all subsystems are created.
     */
    public void setIntake(Intake intake) {
        this.intake = intake;
    }

    /**
     * Set reference to Vision for distance-based speed calculation.
     * Call this after all subsystems are created.
     */
    public void setVision(Vision vision) {
        this.vision = vision;
    }

    // ==================== BEHAVIOR CONTROL ====================

    /**
     * Set the launcher behavior.
     *
     * @param newBehavior IDLE or SPINNING
     */
    public void setBehavior(Behavior newBehavior) {
        if (behavior != newBehavior) {
            behavior = newBehavior;

            // Handle behavior transitions
            if (newBehavior == Behavior.IDLE) {
                // Abort any ongoing launch sequence
                releaseResources();
                state = LaunchState.IDLE;
                fireRequested = false;
            } else if (newBehavior == Behavior.SPINNING) {
                // Start spinning up
                if (state == LaunchState.IDLE) {
                    state = LaunchState.SPINNING_UP;
                }
            }
        }
    }

    /**
     * Get the current behavior.
     */
    public Behavior getBehavior() {
        return behavior;
    }

    // ==================== RESOURCE MANAGEMENT ====================

    /**
     * Claim shared resources (belt and intake suppression).
     * Called when entering a firing state.
     */
    private void claimResources() {
        if (!resourcesClaimed) {
            if (loader != null) {
                loader.claimBeltForLauncher();
            }
            if (intake != null) {
                intake.suppress();
            }
            resourcesClaimed = true;
        }
    }

    /**
     * Release shared resources (belt and intake suppression).
     * Called when exiting firing states.
     */
    private void releaseResources() {
        if (resourcesClaimed) {
            if (loader != null) {
                loader.releaseBeltFromLauncher();
            }
            if (intake != null) {
                intake.unsuppress();
            }
            resourcesClaimed = false;
        }
    }

    // ==================== THREE-PHASE METHODS ====================

    @Override
    public void readSensors() {
        // PHASE 1: Motor velocity is SDK bulk-cached, no I2C read needed
        // We read currentSpeed in calc() from the bulk cache
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        // PHASE 2: Read cached velocity and run state machine

        // Get flywheel speed (bulk-cached by SDK)
        currentSpeed = flywheel.getVelocity(AngleUnit.DEGREES);

        // Update target speed from Vision when spinning
        // This allows continuous speed adjustment as robot moves
        if (behavior == Behavior.SPINNING) {
            if (vision != null && vision.hasBotPose()) {
                targetSpeed = calculateLaunchSpeed(vision.getDistanceToGoal());
            } else {
                targetSpeed = MIN_LAUNCH_SPEED;
            }
        }

        // Sync behavior to state (if behavior changed externally)
        if (behavior == Behavior.IDLE && state != LaunchState.IDLE) {
            // Force back to idle
            state = LaunchState.IDLE;
        } else if (behavior == Behavior.SPINNING && state == LaunchState.IDLE) {
            // Start spinning
            state = LaunchState.SPINNING_UP;
        }

        // Run state machine
        switch (state) {
            case IDLE:
                handleIdleState();
                break;

            case SPINNING_UP:
                handleSpinningUpState();
                break;

            case READY:
                handleReadyState();
                break;

            case FIRING:
                handleFiringState();
                break;

            case COOLDOWN:
                handleCooldownState();
                break;
        }
    }

    @Override
    public void act() {
        // PHASE 3: Flush paddle servo command
        paddle.flush();
    }

    private void handleIdleState() {
        // Ensure resources are released when idle
        releaseResources();
        flywheel.setVelocity(0);
        setPaddlePosition(passThroughMode ? PADDLE_PASS : PADDLE_DOWN);
    }

    private void handleSpinningUpState() {
        flywheel.setVelocity(targetSpeed, AngleUnit.DEGREES);

        if (isFlywheelAtSpeed()) {
            state = LaunchState.READY;
        }
    }

    private void handleReadyState() {
        // Maintain flywheel speed
        flywheel.setVelocity(targetSpeed, AngleUnit.DEGREES);
        setPaddlePosition(PADDLE_DOWN);

        if (fireRequested) {
            fireRequested = false;

            // Claim belt and suppress intake before firing
            claimResources();

            state = LaunchState.FIRING;
            stateTimer = futureTime(PADDLE_LIFT_TIME);
            setPaddlePosition(PADDLE_UP);

            // Tell loader a ball was fired
            if (loader != null) {
                loader.ballFired();
            }
        }
    }
    private void handleFiringState() {
        // Keep flywheel spinning and paddle up
        flywheel.setVelocity(targetSpeed, AngleUnit.DEGREES);
        setPaddlePosition(PADDLE_UP);

        if (isPast(stateTimer)) {
            state = LaunchState.COOLDOWN;
            stateTimer = futureTime(COOLDOWN_TIME);
            setPaddlePosition(PADDLE_DOWN);
        }
    }

    private void handleCooldownState() {
        // Keep flywheel spinning for next shot
        flywheel.setVelocity(targetSpeed, AngleUnit.DEGREES);
        setPaddlePosition(PADDLE_DOWN);

        if (isPast(stateTimer)) {
            // Stay in READY if we might fire again, else go to IDLE
            state = LaunchState.READY;
        }
    }

    private void setPaddlePosition(double position) {
        paddle.setPosition(position);  // Queues position, flushed in act()
    }

    /**
     * Check if flywheel is at target speed.
     */
    public boolean isFlywheelAtSpeed() {
        return currentSpeed >= (targetSpeed - SPEED_TOLERANCE);
    }

    /**
     * Prepare launcher for firing at a given distance.
     * @deprecated Use setBehavior(Behavior.SPINNING) instead. Distance is auto-pulled from Vision.
     */
    @Deprecated
    public void prepareToLaunch(double distanceMeters) {
        targetSpeed = calculateLaunchSpeed(distanceMeters);
        setBehavior(Behavior.SPINNING);
    }

    /**
     * Prepare launcher with a specific speed (for manual testing).
     * @deprecated Use setBehavior(Behavior.SPINNING) instead. Set MIN_LAUNCH_SPEED if needed.
     */
    @Deprecated
    public void prepareToLaunchAtSpeed(double speed) {
        targetSpeed = speed;
        setBehavior(Behavior.SPINNING);
    }

    /**
     * Fire the ball (only works when in READY state).
     * Behavior must be SPINNING and isReady() must return true.
     */
    public void fire() {
        if (state == LaunchState.READY) {
            fireRequested = true;
        }
    }

    /**
     * Abort launch and return to idle.
     * Equivalent to setBehavior(Behavior.IDLE).
     */
    public void abort() {
        setBehavior(Behavior.IDLE);
    }

    /**
     * Check if launcher is ready to fire.
     */
    public boolean isReady() {
        return state == LaunchState.READY;
    }

    /**
     * Check if launcher is currently in a launch sequence.
     */
    public boolean isBusy() {
        return state != LaunchState.IDLE && state != LaunchState.READY;
    }

    /**
     * Enable pass-through mode (paddle stays up).
     * Used for passing balls to partner robot.
     */
    public void setPassThroughMode(boolean enabled) {
        passThroughMode = enabled;
        if (enabled && state == LaunchState.IDLE) {
            setPaddlePosition(PADDLE_PASS);
        }
    }

    /**
     * Calculate launch speed based on distance using physics model.
     * Based on projectile motion equations.
     *
     * @param distanceMeters Horizontal distance to target
     * @return Required flywheel speed in degrees/sec
     */
    public double calculateLaunchSpeed(double distanceMeters) {
        double d = distanceMeters + LIMELIGHT_OFFSET;
        double theta = Math.toRadians(LAUNCH_ANGLE);
        double h = TARGET_HEIGHT;

        // Projectile motion: d = v*cos(θ)*t, h = v*sin(θ)*t - 0.5*g*t²
        // Solve for v given d, h, θ
        double tanTheta = Math.tan(theta);
        double cosTheta = Math.cos(theta);

        // Time of flight from vertical motion
        double discriminant = (tanTheta * d - h) / 4.905;
        if (discriminant < 0) discriminant = 0.1; // Fallback

        double time = Math.sqrt(discriminant);

        // Required velocity
        double velocity = d / (cosTheta * time);

        // Convert to flywheel speed (degrees/sec)
        // This factor depends on flywheel diameter and motor characteristics
        double flywheelSpeed = velocity * 1090 / 6.67;

        // Add safety margin
        return flywheelSpeed + 50;
    }

    /**
     * Get current launch state.
     */
    public LaunchState getState() {
        return state;
    }

    /**
     * Get current flywheel speed.
     */
    public double getCurrentSpeed() {
        return currentSpeed;
    }

    /**
     * Get target flywheel speed.
     */
    public double getTargetSpeed() {
        return targetSpeed;
    }

    @Override
    public void stop() {
        behavior = Behavior.IDLE;
        releaseResources();
        state = LaunchState.IDLE;
        fireRequested = false;
        flywheel.setVelocity(0);
        paddle.setPosition(PADDLE_DOWN);
        paddle.flush();  // Immediate write for emergency stop
    }

    @Override
    public void resetStates() {
        behavior = Behavior.IDLE;
        releaseResources();
        state = LaunchState.IDLE;
        fireRequested = false;
        passThroughMode = false;
    }

    @Override
    public String getTelemetryName() {
        return "Launcher";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Behavior", behavior);
        telemetry.put("Flywheel Speed", String.format("%.0f / %.0f", currentSpeed, targetSpeed));
        telemetry.put("Ready", isReady() ? "YES" : "no");

        if (debug) {
            telemetry.put("Internal State", state);
            telemetry.put("At Speed", isFlywheelAtSpeed() ? "YES" : "no");
            telemetry.put("Paddle Pos", paddle.getPendingPosition());
            telemetry.put("Pass-Through", passThroughMode ? "ON" : "off");
            telemetry.put("Fire Requested", fireRequested);
        }

        return telemetry;
    }
}
