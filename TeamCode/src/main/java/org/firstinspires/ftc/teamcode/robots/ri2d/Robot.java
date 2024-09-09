package org.firstinspires.ftc.teamcode.robots.ri2d;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Subsystem;

import java.util.LinkedHashMap;
import java.util.Map;

public class Robot implements Subsystem {

    public Subsystem[] subsystems;
    public Swerve swerve;
    public DcMotorEx rightHook, leftHook;
    public HardwareMap hardwareMap;
    public long[] subsystemUpdateTimes;

    public Robot(HardwareMap hardwareMap){
        swerve = new Swerve(hardwareMap,this);
        subsystems = new Subsystem[]{swerve};
        rightHook = hardwareMap.get(DcMotorEx.class, "hook1");
        leftHook = hardwareMap.get(DcMotorEx.class, "hook2");
        rightHook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftHook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightHook.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void update(Canvas fieldOverlay) {
        for (Subsystem subsystem : subsystems) {
            subsystem.update(fieldOverlay);
        }
        handleTelemetry();
    }

    public void handleTelemetry() {

    }

    @Override
    public void stop() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "Robot";
    }
}
