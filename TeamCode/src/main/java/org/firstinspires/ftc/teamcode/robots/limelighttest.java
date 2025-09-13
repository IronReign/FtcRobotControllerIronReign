package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.deepthought.SensorLimelight3A;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;

import java.util.Collections;
import java.util.Map;

@Config(value = "LimeLightTest")
public class limelighttest implements Subsystem{
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    private SensorLimelight3A limelight3A;


    public limelighttest(HardwareMap hardwareMap, Gamepad gamepad){
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad;
        StickyGamepad spad1;
        spad1 = new StickyGamepad(gamepad1);

        limelight3A = hardwareMap.get(SensorLimelight3A.class, "limelight");

    }
    @Override
    public void update(Canvas fieldOverlay) {

    }

    @Override
    public void stop() {

    }

    @Override
    public void resetStates() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        return Collections.emptyMap();
    }

    @Override
    public String getTelemetryName() {
        return "";
    }
}


