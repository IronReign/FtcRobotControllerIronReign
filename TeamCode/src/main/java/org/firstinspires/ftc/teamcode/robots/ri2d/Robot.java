package org.firstinspires.ftc.teamcode.robots.ri2d;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Subsystem;

import java.util.LinkedHashMap;
import java.util.Map;

public class Robot implements Subsystem {

    public Subsystem[] subsystems;
    public Swerve swerve;
    public HardwareMap hardwareMap;
    public long[] subsystemUpdateTimes;

    public Robot(HardwareMap hardwareMap){
        swerve = new Swerve(hardwareMap,this);
        subsystems = new Subsystem[]{swerve};
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
