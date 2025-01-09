package org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.samplers;

import android.graphics.Color;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Trident;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class Beater extends Arm {

    @Override
    public void stopOnSample() {

    }

    @Override
    public void update(Canvas fieldOverlay) {

    }

    @Override
    public void stop() {
        servoPower = 0;
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
        return telemetryMap;
    }


    @Override
    public boolean intake() {
        return false;
    }

    @Override
    public boolean outtake() {
        return false;
    }

    @Override
    public String updateColorSensor() {
        double hue = getHSV()[0];
        if (hue < 45 && hue > 35) {
            currentSample = Sample.NEUTRAL;
            return "NEUTRAL";
        } else if (hue < 15 || hue > 350) {
            currentSample = Sample.RED;
            return "RED";
        } else if (hue < 225 && hue > 200) {
            currentSample = Sample.BLUE;
            return "BLUE";
        } else {
            currentSample = Sample.NO_SAMPLE;
            return "NO SAMPLE";
        }
    }

    public String HSVasString() {
        float[] hsv = getHSV();
        return hsv[0] + " " + hsv[1] + " " + hsv[2];
    }

    public float[] getHSV() {
        float[] hsv = new float[3];
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);
        return hsv;
    }

    @Override
    public String getTelemetryName() {
        return "Beater";
    }
}
