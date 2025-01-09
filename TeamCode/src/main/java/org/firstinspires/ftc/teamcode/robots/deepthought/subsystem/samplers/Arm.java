package org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.samplers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public abstract class Arm implements Subsystem {
    DcMotorEx slide = null;
    NormalizedColorSensor colorSensor = null;

    int slideTargetPosition = 0;
    int slideMinPosition = 0;
    //todo - determine this
    int slideMaxPosition = 2450;

    double servoPower = 0;

    boolean colorSensorEnabled = false;
    boolean inControl = false;

    enum Sample {
        RED, BLUE, NEUTRAL, NO_SAMPLE
    }
    Sample currentSample = null;
    List<Sample> targetSamples = new ArrayList<>();

    abstract boolean intake();
    abstract boolean outtake();

    abstract String updateColorSensor();
    abstract void stopOnSample();
}
