package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Intake subsystem - controls the overhead belts that pull balls into the robot.
 *
 * The intake is the front-facing mechanism with overhead belts and flaps
 * that grab balls from the ground and pull them into the loader chamber.
 *
 * This is a simple subsystem with no state machine - it's just on or off.
 * The Robot coordinator decides when to activate intake based on the
 * current operating mode.
 */
@Config(value = "Lebot2_Intake")
public class Intake implements Subsystem {

    // Hardware
    private final DcMotorEx intakeMotor;

    // Configuration
    public static double INTAKE_POWER = 1.0;
    public static double EJECT_POWER = -0.5; // Reverse to push balls out

    // State
    private double currentPower = 0;
    private boolean isRunning = false;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void update(Canvas fieldOverlay) {
        // Intake is simple - just apply the current power setting
        intakeMotor.setPower(currentPower);
    }

    /**
     * Turn intake on at full power.
     */
    public void on() {
        currentPower = INTAKE_POWER;
        isRunning = true;
    }

    /**
     * Turn intake off.
     */
    public void off() {
        currentPower = 0;
        isRunning = false;
    }

    /**
     * Run intake in reverse to eject balls.
     */
    public void eject() {
        currentPower = EJECT_POWER;
        isRunning = true;
    }

    /**
     * Set intake to a specific power level.
     *
     * @param power Power level (-1 to 1). Positive = intake, negative = eject.
     */
    public void setSpeed(double power) {
        currentPower = power;
        isRunning = power != 0;
    }

    /**
     * Check if the intake is currently running.
     *
     * @return true if intake power is non-zero
     */
    public boolean isRunning() {
        return isRunning;
    }

    @Override
    public void stop() {
        currentPower = 0;
        isRunning = false;
        intakeMotor.setPower(0);
    }

    @Override
    public void resetStates() {
        // Intake has no state machine, just stop it
        off();
    }

    @Override
    public String getTelemetryName() {
        return "Intake";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Running", isRunning ? "YES" : "no");
        telemetry.put("Power", String.format("%.2f", currentPower));

        return telemetry;
    }
}
