package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
 * The paddle has dual roles:
 * - DOWN position: acts as a gate/stop for balls in the loader
 * - UP position: lifts ball into flywheel for launch
 *
 * CRITICAL: The flywheel MUST be at target speed before the paddle lifts,
 * otherwise the ball will jam the flywheel.
 *
 * Launch Sequence State Machine:
 * IDLE → SPINNING_UP → READY → FIRING → COOLDOWN → IDLE
 */
@Config(value = "Lebot2_Launcher")
public class Launcher implements Subsystem {

    // Hardware
    private final DcMotorEx flywheel;
    private final Servo paddle;

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

    // State machine
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

    // Reference to Loader for coordinated feeding
    private Loader loader = null;

    public Launcher(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        paddle = hardwareMap.get(Servo.class, "paddle");
        paddle.setPosition(PADDLE_DOWN);
    }

    /**
     * Set reference to Loader for coordinated operations.
     * Call this after both subsystems are created.
     */
    public void setLoader(Loader loader) {
        this.loader = loader;
    }

    @Override
    public void update(Canvas fieldOverlay) {
        // READ: Get flywheel speed
        currentSpeed = flywheel.getVelocity(AngleUnit.DEGREES);

        // PROCESS: State machine
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

        // WRITE: Outputs are set in state handlers
    }

    private void handleIdleState() {
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
        paddle.setPosition(position);
    }

    /**
     * Check if flywheel is at target speed.
     */
    public boolean isFlywheelAtSpeed() {
        return currentSpeed >= (targetSpeed - SPEED_TOLERANCE);
    }

    /**
     * Prepare launcher for firing at a given distance.
     * Starts the flywheel spinning up to calculated speed.
     *
     * @param distanceMeters Distance to target in meters
     */
    public void prepareToLaunch(double distanceMeters) {
        targetSpeed = calculateLaunchSpeed(distanceMeters);
        if (state == LaunchState.IDLE) {
            state = LaunchState.SPINNING_UP;
        }
    }

    /**
     * Prepare launcher with a specific speed (for manual testing).
     *
     * @param speed Flywheel speed in degrees/sec
     */
    public void prepareToLaunchAtSpeed(double speed) {
        targetSpeed = speed;
        if (state == LaunchState.IDLE) {
            state = LaunchState.SPINNING_UP;
        }
    }

    /**
     * Fire the ball (only works when in READY state).
     */
    public void fire() {
        if (state == LaunchState.READY) {
            fireRequested = true;
        }
    }

    /**
     * Abort launch and return to idle.
     */
    public void abort() {
        state = LaunchState.IDLE;
        fireRequested = false;
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
        state = LaunchState.IDLE;
        fireRequested = false;
        flywheel.setVelocity(0);
        setPaddlePosition(PADDLE_DOWN);
    }

    @Override
    public void resetStates() {
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

        telemetry.put("State", state);
        telemetry.put("Flywheel Speed", String.format("%.0f / %.0f", currentSpeed, targetSpeed));
        telemetry.put("At Speed", isFlywheelAtSpeed() ? "YES" : "no");

        if (debug) {
            telemetry.put("Paddle Pos", paddle.getPosition());
            telemetry.put("Pass-Through", passThroughMode ? "ON" : "off");
            telemetry.put("Fire Requested", fireRequested);
        }

        return telemetry;
    }
}
