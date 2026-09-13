package org.firstinspires.ftc.teamcode.robots.ri2d2.subsystem;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.ri2d2.subsystem.Subsystem;

import java.util.Collections;
import java.util.Map;

public class Catapult implements Subsystem {

    DcMotorEx catapultMotor;

    public Catapult(HardwareMap hardwareMap){
        catapultMotor = hardwareMap.get(DcMotorEx.class, "catapult");
    }

    @Override
    public void readSensors() {

    }

    @Override
    public void calc(Canvas fieldOverlay) {

    }

    @Override
    public void act() {

    }

    @Override
    public void resetStates() {

    }

    @Override
    public void stop() {

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
