package org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.samplers;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public interface Arm extends Subsystem {
    DcMotorEx slide = null;
    int slideTargetPosition = 0;
    int slideMinPosition = 0;
    //todo - determine this
    int slideMaxPosition = 2450;
    double servoPower = 0;
    boolean colorSensorEnabled = false;

    enum Sample {
        RED, BLUE, NEUTRAL, NO_SAMPLE
    }

    Sample currentSample = Sample.NO_SAMPLE;
    List<Sample> targetSamples = new ArrayList<>();

    String updateColorSensor();
    void stopOnSample();
}
