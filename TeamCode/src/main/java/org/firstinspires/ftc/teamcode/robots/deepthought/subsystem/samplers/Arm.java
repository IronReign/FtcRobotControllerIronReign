package org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.samplers;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Joint;

import java.util.ArrayList;
import java.util.List;

public abstract class Arm implements Subsystem {
    HardwareMap hardwareMap;
    Robot robot;

    DcMotorEx slide = null;
    NormalizedColorSensor colorSensor = null;
    Joint elbow;
    static boolean specimenTargeted = false;

    int slideTargetPosition = 0;
    int slideMinPosition = 0;
    //todo - determine this
    int slideMaxPosition = 2450;

    //SLIDE VARIABLES
    public static int slidePositionMax = 3800;
    public static int slidePositionMin = 0;
    public int SLIDE_INTAKE_MIN_POSITION = 200;
    public int SLIDE_PREINTAKE_POSITION = 2200;
    public int SLIDE_LOWOUTTAKE_POSITION = 320;
    public int SLIDE_HIGHOUTTAKE_POSITION = 2880;
    public static int slideSpeed = 80;
    public static double SLIDE_SPEED = 2000;


    //ELBOW JOINT VARIABLES
    public static double ELBOW_START_ANGLE = 145;
    public static int ELBOW_HOME_POSITION = 2050;
    public static double ELBOW_PWM_PER_DEGREE = -5.672222222222222;
    public static double ELBOW_JOINT_SPEED = 120;
    public static double ELBOW_MIN_ANGLE = -15;
    public static double ELBOW_MAX_ANGLE = 220;
    public static int ELBOW_ADJUST_ANGLE = 5;
    public static double ELBOW_PREINTAKE_ANGLE = 20;
    public static double ELBOW_LOWOUTTAKE_ANGLE = 102;
    public static double ELBOW_HIGHOUTTAKE_ANGLE = 70;

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
    abstract boolean tuck();

    abstract String updateColorSensor();

    public String HSVasString() {
        float[] hsv = getHSV();
        return hsv[0] + " " + hsv[1] + " " + hsv[2];
    }
    public float[] getHSV() {
        float[] hsv = new float[3];
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);
        return hsv;
    }

    abstract boolean stopOnSample();
}
