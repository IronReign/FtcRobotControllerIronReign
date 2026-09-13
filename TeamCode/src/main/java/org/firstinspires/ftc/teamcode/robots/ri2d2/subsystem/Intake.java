package org.firstinspires.ftc.teamcode.robots.ri2d2.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.LinkedHashMap;
import java.util.Map;

public class Intake implements Subsystem {

    private final DcMotorEx intakeMotor;
    private double targetPower = 0;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
    }

    //Queue power
    public void setPower(double power) {
        targetPower = Range.clip(power, -1.0, 1.0);
    }

    @Override
    public void readSensors() {
        // No sensors yet
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        // Future intake
    }

    @Override
    public void act() {
        intakeMotor.setPower(targetPower);
    }

    @Override
    public void stop() {
        targetPower = 0;
        intakeMotor.setPower(0);
    }

    @Override
    public void resetStates() {
        targetPower = 0;
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();
        telemetry.put("Target Power", targetPower);
        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "Intake";
    }
}
