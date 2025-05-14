package org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.samplers;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;
import static org.firstinspires.ftc.teamcode.util.utilMethods.withinError;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Trident;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Joint;

import java.util.ArrayList;
import java.util.List;

public abstract class Arm implements Subsystem {
    HardwareMap hardwareMap;
    Robot robot;
    Trident trident;

    public DcMotorEx slide;
    public Joint elbow;

    NormalizedColorSensor colorSensor = null;
    float[] colorLastHSV = {0.0f, 0.0f, 0.0f};
    NormalizedRGBA colorLastRGBA = new NormalizedRGBA();

    int slidePos = 0;
    int slidePosPrev = 10;

    static boolean specimenTargeted = false;

    int slideMinPosition = 0;
    //todo - determine this
    int slideMaxPosition = Integer.MAX_VALUE;

    //SLIDE VARIABLES - todo, values should be moved into implementations
    public static int slidePositionMax = 3300;
    public static int slidePositionMin = 0;
    public int SLIDE_INTAKE_MIN_POSITION = 200;
    public int SLIDE_PREINTAKE_POSITION = 1200;
    public int SLIDE_LOWOUTTAKE_POSITION = 320;

    public int SLIDE_HIGHOUTTAKE_POSITION = 2320;
    public static int slideSpeed = 80;
    public static double SLIDE_SPEED = 8000;


    //ELBOW JOINT VARIABLES - expect these to be set in implementation
    public double ELBOW_START_ANGLE;
    public static int ELBOW_HOME_POSITION;
    public static double ELBOW_PWM_PER_DEGREE;
    public static double ELBOW_JOINT_SPEED;
    public double ELBOW_MIN_ANGLE;
    public double ELBOW_MAX_ANGLE;
    public static int ELBOW_ADJUST_ANGLE;
    public double ELBOW_PREINTAKE_ANGLE;  // used to set the intake at a height to clear the sub perimeter
    public double ELBOW_LOWOUTTAKE_ANGLE;
    public double ELBOW_HIGHOUTTAKE_ANGLE;

    public double servoPower = 0;

    public static boolean colorSensorEnabled = false;

    boolean inControl = false;

    abstract public void adjustSlide(int adjustTicks);


    public enum Sample {
        RED, BLUE, NEUTRAL, NO_SAMPLE
    }

    public Sample currentSample = Sample.NO_SAMPLE;
    List<Sample> targetSamples;

    abstract boolean finalizeTargets();

    abstract boolean intake();

    abstract boolean outtake();

    abstract boolean tuck();

    abstract String updateColorSensor();

    public String HSVasString() {
        float[] hsv = colorLastHSV;
        return hsv[0] + " " + hsv[1] + " " + hsv[2];
    }

    public float[] getHSV() {
        float[] hsv = new float[3];
        colorLastRGBA = colorSensor.getNormalizedColors();
        Color.colorToHSV(colorLastRGBA.toColor(), hsv);
        colorLastHSV = hsv;
        return hsv;
    }

    public abstract boolean stopOnSample();

    public abstract void adjustElbow(double adjustAngle);

    boolean calibrated = true;
    int calibrateIndex = 0;
    public long calibrateTimer = 0;

    public boolean calibrate(Arm subs, double elbowStartAngle) {
        switch (calibrateIndex) {
            case 0: //set elbow position and retract slide until it stalls
                calibrated = false;
                elbow = subs.elbow;
                slide = subs.slide;
                subs.servoPower = 0; //stop the beater(s)

                elbow.setTargetAngle(elbowStartAngle);

                slidePos = subs.slide.getCurrentPosition();
                slidePosPrev = slidePos + 10;
                calibrateTimer = futureTime(1); //enough time to assure it has begun moving
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slide.setPower(-.5);
                calibrateIndex++;
                break;
            case 1: // has the slide stopped moving?
                slidePos = slide.getCurrentPosition();
                //withinError to allow stall detection even if vibrating slightly
                if (withinError(slidePos, slidePosPrev, 2) && isPast(calibrateTimer)) {
                    calibrateIndex++;
                }
                slidePosPrev = slidePos;
                break;
            case 2: // reset the encoders to zero

                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide.setPower(1);
                slide.setTargetPosition(10);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                calibrateIndex = 0;
                calibrated = true;
                return true;
        }
        return false;
    }
}
