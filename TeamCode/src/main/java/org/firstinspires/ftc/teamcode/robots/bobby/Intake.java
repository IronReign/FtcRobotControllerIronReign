package org.firstinspires.ftc.teamcode.robots.bobby;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;

import java.util.LinkedHashMap;
import java.util.Map;

public class Intake implements Subsystem {
    private final DcMotorEx intake;
    private double intakePower;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setIntakePower(double power) {
        intakePower = power;
        intake.setPower(power);
    }

    @Override
    public void update(Canvas fieldOverlay) {
    }

    @Override
    public void stop() {
        setIntakePower(0.0);
    }

    @Override
    public void resetStates() {
        setIntakePower(0.0);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();
        telemetry.put("Intake power", intakePower);
        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "Intake";
    }
}