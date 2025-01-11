package org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.samplers;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.CRServo;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

public class RollerClawImpl extends Arm{
    public CRServo CRSOne;
    public CRServo CRSTwo;

    @Override
    boolean intake() {
        return false;
    }

    @Override
    boolean outtake() {
        return false;
    }

    @Override
    boolean tuck() {
        return false;
    }

    @Override
    String updateColorSensor() {
        return "";
    }

    @Override
    boolean stopOnSample() {
        return false;
    }

    @Override
    public void update(Canvas fieldOverlay) {

    }

    @Override
    public void stop() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();

        telemetryMap.put("slide target : real", slideTargetPosition + " : " + slide.getCurrentPosition());
        telemetryMap.put("current sample", currentSample.name());
        telemetryMap.put("colorsensor", colorSensorEnabled);
        if (colorSensorEnabled) {
            telemetryMap.put("colorsensor hsv", "" + HSVasString());
            telemetryMap.put("colorsensor rgb", colorSensor.getNormalizedColors().red + " " + colorSensor.getNormalizedColors().green + " " + colorSensor.getNormalizedColors().blue);
        }
        telemetryMap.put("beater speed", servoPower);
        telemetryMap.put("elbow angle target : real", elbow.getTargetAngle() + " : " + elbow.getCurrentAngle());

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "RollerClaw";
    }
}
