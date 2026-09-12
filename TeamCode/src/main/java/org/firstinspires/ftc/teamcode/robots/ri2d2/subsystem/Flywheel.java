package org.firstinspires.ftc.teamcode.robots.newBot.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "newBot_Flywheel")
public class Flywheel implements Subsystem {

    public static double SPEED_DEGREES_PER_SECOND = 1350;

    private final DcMotorEx flywheelMotor;
    private final DcMotorEx helperMotor;
    private boolean spinning = false;
    private double targetSpeed = 0;
    private double currentSpeed = 0;
    private double helperSpeed = 0;

    public Flywheel(HardwareMap hardwareMap, boolean useTwoMotors) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, useTwoMotors ? "shooter" : "flywheel");
        helperMotor = useTwoMotors ? hardwareMap.get(DcMotorEx.class, "shooter helper") : null;

        flywheelMotor.setDirection(DcMotor.Direction.FORWARD);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        if (helperMotor != null) {
            helperMotor.setDirection(DcMotor.Direction.REVERSE);
            helperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            helperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // Reuse lebot2's velocity PIDF
            flywheelMotor.setVelocityPIDFCoefficients(400, 0, 0, 20);
            helperMotor.setVelocityPIDFCoefficients(400, 0, 0, 20);
        }
        stop();
    }

    public void toggle() {
        spinning = !spinning;
    }

    @Override
    public void readSensors() {
        currentSpeed = flywheelMotor.getVelocity(AngleUnit.DEGREES);
        if (helperMotor != null) {
            helperSpeed = helperMotor.getVelocity(AngleUnit.DEGREES);
        }
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        targetSpeed = spinning ? Math.max(0, SPEED_DEGREES_PER_SECOND) : 0;
    }

    @Override
    public void act() {
        flywheelMotor.setVelocity(targetSpeed, AngleUnit.DEGREES);
        if (helperMotor != null) {
            helperMotor.setVelocity(targetSpeed, AngleUnit.DEGREES);
        }
    }

    @Override
    public void stop() {
        resetStates();
        flywheelMotor.setPower(0);
        if (helperMotor != null) {
            helperMotor.setPower(0);
        }
    }

    @Override
    public void resetStates() {
        spinning = false;
        targetSpeed = 0;
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();
        telemetry.put("Mode", helperMotor == null ? "One motor" : "Two motors");
        telemetry.put("Spinning", spinning);
        telemetry.put("Target Speed (deg/s)", targetSpeed);
        telemetry.put("Speed (deg/s)", currentSpeed);
        if (helperMotor != null) {
            telemetry.put("Helper Speed (deg/s)", helperSpeed);
        }
        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "Flywheel";
    }
}
