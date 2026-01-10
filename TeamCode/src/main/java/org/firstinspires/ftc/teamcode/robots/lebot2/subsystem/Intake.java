package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.lebot2.util.LazyMotor;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Intake subsystem - controls the overhead belts that pull balls into the robot.
 *
 * The intake is the front-facing mechanism with overhead belts and flaps
 * that grab balls from the ground and pull them into the loader chamber.
 *
 * BEHAVIOR STATE MACHINE:
 * - OFF: Motor stopped (default)
 * - INTAKE: Motor running until manually stopped
 * - LOAD_ALL: Run until loader reports full, then auto-stop
 * - EJECT: Reverse motor to push balls out
 *
 * SUPPRESSION:
 * When Launcher claims the belt for firing, intake is suppressed (motor stops).
 * When Launcher releases the belt, intake resumes if behavior is still active.
 *
 * THREE-PHASE UPDATE:
 * - readSensors(): Nothing to read (motor has no I2C sensors we need)
 * - calc(): Run behavior state machine, set power via lazyMotor.setPower()
 * - act(): Flush motor command
 */
@Config(value = "Lebot2_Intake")
public class Intake implements Subsystem {

    // Hardware (wrapped for three-phase pattern)
    private final LazyMotor intakeMotor;

    // Configuration
    public static double INTAKE_POWER = 1.0;
    public static double EJECT_POWER = -0.5; // Reverse to push balls out

    // Behavior state machine
    public enum Behavior {
        OFF,        // Motor stopped (default)
        INTAKE,     // Motor running until manually stopped
        LOAD_ALL,   // Run until loader full, then auto-stop
        EJECT       // Reverse motor
    }
    private Behavior behavior = Behavior.OFF;

    // Suppression state (set by Launcher when it claims belt)
    private boolean suppressed = false;

    // Reference to Loader for LOAD_ALL completion check
    private Loader loader = null;

    // Internal state
    private double currentPower = 0;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = new LazyMotor(hardwareMap, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Set reference to Loader for LOAD_ALL completion check.
     * Call this after both subsystems are created.
     */
    public void setLoader(Loader loader) {
        this.loader = loader;
    }

    // ==================== THREE-PHASE METHODS ====================

    @Override
    public void readSensors() {
        // Intake has no I2C sensors to read
        // (Motor current monitoring not needed for intake)
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        // Run behavior state machine
        switch (behavior) {
            case OFF:
                currentPower = 0;
                break;

            case INTAKE:
                currentPower = suppressed ? 0 : INTAKE_POWER;
                break;

            case LOAD_ALL:
                if (suppressed) {
                    currentPower = 0;
                } else if (loader != null && loader.isFull()) {
                    // Auto-complete: loader is full
                    behavior = Behavior.OFF;
                    currentPower = 0;
                } else {
                    currentPower = INTAKE_POWER;
                }
                break;

            case EJECT:
                currentPower = suppressed ? 0 : EJECT_POWER;
                break;
        }

        // Queue power for Phase 3
        intakeMotor.setPower(currentPower);
    }

    @Override
    public void act() {
        // Flush motor command to hardware
        intakeMotor.flush();
    }

    // ==================== BEHAVIOR CONTROL ====================

    /**
     * Set the intake behavior.
     *
     * @param newBehavior The desired behavior
     */
    public void setBehavior(Behavior newBehavior) {
        this.behavior = newBehavior;
    }

    /**
     * Get the current behavior.
     */
    public Behavior getBehavior() {
        return behavior;
    }

    /**
     * Turn intake on (simple INTAKE behavior).
     */
    public void on() {
        behavior = Behavior.INTAKE;
    }

    /**
     * Turn intake off.
     */
    public void off() {
        behavior = Behavior.OFF;
    }

    /**
     * Start LOAD_ALL behavior - runs until loader is full.
     */
    public void loadAll() {
        behavior = Behavior.LOAD_ALL;
    }

    /**
     * Run intake in reverse to eject balls.
     */
    public void eject() {
        behavior = Behavior.EJECT;
    }

    /**
     * Check if the intake is currently active (not OFF and not suppressed).
     *
     * @return true if intake is running or would be running if not suppressed
     */
    public boolean isActive() {
        return behavior != Behavior.OFF;
    }

    /**
     * Check if the intake motor is actually running.
     *
     * @return true if motor power is non-zero
     */
    public boolean isRunning() {
        return currentPower != 0;
    }

    // ==================== SUPPRESSION (called by Launcher) ====================

    /**
     * Suppress the intake (called when Launcher claims belt).
     * Motor stops but behavior is preserved for resume.
     */
    public void suppress() {
        suppressed = true;
    }

    /**
     * Release suppression (called when Launcher releases belt).
     * Motor resumes if behavior is still active.
     */
    public void unsuppress() {
        suppressed = false;
    }

    /**
     * Check if intake is currently suppressed.
     */
    public boolean isSuppressed() {
        return suppressed;
    }

    // ==================== LIFECYCLE ====================

    @Override
    public void stop() {
        behavior = Behavior.OFF;
        suppressed = false;
        currentPower = 0;
        intakeMotor.stop();  // Immediate stop, bypasses lazy pattern
    }

    @Override
    public void resetStates() {
        behavior = Behavior.OFF;
        suppressed = false;
    }

    @Override
    public String getTelemetryName() {
        return "Intake";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Behavior", behavior);
        telemetry.put("Running", isRunning() ? "YES" : (suppressed ? "SUPPRESSED" : "no"));

        if (debug) {
            telemetry.put("Power", String.format("%.2f", currentPower));
            telemetry.put("Suppressed", suppressed);
        }

        return telemetry;
    }
}
