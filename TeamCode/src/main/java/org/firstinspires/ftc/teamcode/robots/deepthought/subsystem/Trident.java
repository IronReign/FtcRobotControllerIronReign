package org.firstinspires.ftc.teamcode.robots.deepthought.subsystem;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;
import static org.firstinspires.ftc.teamcode.util.utilMethods.withinError;

import android.graphics.Color;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Constants;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.DcMotorExResetable;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Joint;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils;

import java.util.*;


@Config(value = "00_ITD_TRIDENT")
public class Trident implements Subsystem {
    HardwareMap hardwareMap;
    Robot robot;

    public static boolean enforceSlideLimits;
    public DcMotorExResetable slide = null;
    public DcMotorExResetable shoulder = null;
    public Joint elbow;

    public CRServo beater = null;
    public Servo pincer;
    public static boolean tuckFromHighOuttake = false;

    public NormalizedColorSensor colorSensor = null;
    public static boolean colorSensorEnabled = false;

    public static int colorSensorGain = 12;

    public boolean calibrated = false;

    public void intakeSlideLimits() {
        if (slideTargetPosition > slidePositionMax) slideTargetPosition = slidePositionMax;
        if (slideTargetPosition < slidePositionMin) slideTargetPosition = slidePositionMin;
    }


    public enum Sample {
        RED, BLUE, NEUTRAL, NO_SAMPLE

    }

    public Sample currentSample = Sample.NO_SAMPLE;
    public List<Sample> targetSamples = new ArrayList<>();

    public static boolean preferHighOuttake = true;
    //SLIDE
    public static int slideTargetPosition = 0;
    public static int slidePositionMax = 3800;
    public static int slidePositionMin = 0;
    public static int SLIDE_INTAKE_MIN_POSITION = 200;
    public static int SLIDE_PREINTAKE_POSITION = 2200;
    public static int SLIDE_LOWOUTTAKE_POSITION = 320;
    public static int SLIDE_HIGHOUTTAKE_POSITION = 2880;
    public static int SLIDE_ADJUST_SPEED = 80;
    public static double SLIDE_SPEED = 2000;


    //shoulder
    public static int SHOULDER_CALIBRATE_ENCODER = Integer.MIN_VALUE;
    public int shoulderTargetPosition = 0;
    public static int shoulderSpeed = 45;
    public static int SHOULDER_HOME_POSITION = 250;
    public static int SHOULDER_INTAKE_POSITION = 250;
    public static int SHOULDER_LOWOUTTAKE_POSITION = 2105;
    public static int SHOULDER_HIGHOUTTAKE_POSITION = 1925;
    public int shoulderPositionMax = 850;

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

    //BEATER
    public static double beaterPower;

    //PINCER
    public static double PINCER_CLOSE = 1750;
    public static double PINCER_OPEN = 2150;
    public boolean pincerEnabled = true;


    public enum Articulation {
        MANUAL, //does nothing - used for transition tracking
        TUCK, //does nothing - used for transition tracking
        INTAKE, OUTTAKE
    }

    //LIVE STATES
    public Articulation articulation;
    private Articulation prevArticulation;

    public Articulation articulate(Articulation articulation) {
        this.articulation = articulation;
        return articulation;
    }

    public Articulation articulate() {
        switch (articulation) {
            case MANUAL:
                break;
            case TUCK:
                if (tuck()) {
                    articulation = Articulation.MANUAL;
                }
                break;
            //SHOULD ONLY BE ACCESSED BY SAMPLE()
            case INTAKE:
                if (intake()) {
                    articulation = Articulation.MANUAL;
                }
                break;

            case OUTTAKE:
                if (outtake()) articulation = Articulation.MANUAL;
                break;
            default:
                throw new RuntimeException("how the ^%*( did you get here?");
        }
        prevArticulation = articulation;
        return articulation;
    }

    public long tuckTimer = 0;
    public static int tuckIndex = 0;

    public boolean tuck() {
        switch (tuckIndex) {
            case 0:
                tuckTimer = futureTime(.7);
                elbow.setTargetAngle(ELBOW_START_ANGLE);
                beaterPower = 0;
                tuckIndex++;
                break;
            case 1:
                if (isPast(tuckTimer)) {
                    slideTargetPosition = 0;
                    tuckIndex++;
                }
                break;
            case 2:
                if (slide.getCurrentPosition() < 150) {
                    shoulderTargetPosition = SHOULDER_HOME_POSITION;
                    return true;
                }
        }
        return false;
    }

    public static int intakeIndex;
    public long intakeTimer;

    public boolean intake() {
        switch (intakeIndex) {
            case 0:
                elbow.setTargetAngle(ELBOW_PREINTAKE_ANGLE);
                intakeTimer = futureTime(.5);
                intakeIndex++;
                break;
            case 1:
                if (isPast(intakeTimer)) {
                    shoulderTargetPosition = SHOULDER_INTAKE_POSITION;
                    slideTargetPosition = SLIDE_PREINTAKE_POSITION;
                    intakeIndex++;
                }
                break;
            case 2:
                if (withinError(shoulder.getCurrentPosition(), SHOULDER_INTAKE_POSITION, 10) && withinError(slide.getCurrentPosition(), SLIDE_PREINTAKE_POSITION, 10)) {
                    beaterPower = .8;
                    intakeTimer = futureTime(8);
                    intakeIndex++;
                    colorSensorEnabled = true;
                }
                break;
            case 3:
                if (slideTargetPosition > SLIDE_INTAKE_MIN_POSITION) {
                    slideTargetPosition -= 120;
                    shoulderTargetPosition -= 5;
                }
                if (stopOnSample() || isPast(intakeTimer)) {
                    intakeIndex = 0;
                    robot.articulation = Robot.Articulation.MANUAL;
                    return true;
                }
                break;
        }
        return false;
    }

    public static int outtakeIndex;
    public long outtakeTimer;

    public boolean outtake() {
        switch (outtakeIndex) {
            case 0:
                outtakeTimer = futureTime(0);
//                elbow.setTargetAngle(preferHighOuttake ? ELBOW_HIGHOUTTAKE_ANGLE : ELBOW_LOWOUTTAKE_ANGLE);
                outtakeIndex++;
                break;
            case 1:
                if (isPast(outtakeTimer)) {
                    slideTargetPosition = preferHighOuttake ? SLIDE_HIGHOUTTAKE_POSITION : SLIDE_LOWOUTTAKE_POSITION;
                    shoulderTargetPosition = preferHighOuttake ? (int)(SHOULDER_HIGHOUTTAKE_POSITION * .75) : SHOULDER_LOWOUTTAKE_POSITION;
                    outtakeIndex++;
                }
                break;
            case 2:
                if (withinError(slide.getCurrentPosition(), preferHighOuttake ? SLIDE_HIGHOUTTAKE_POSITION : SLIDE_LOWOUTTAKE_POSITION, 10)) {
                    elbow.setTargetAngle(preferHighOuttake ? ELBOW_HIGHOUTTAKE_ANGLE : ELBOW_LOWOUTTAKE_ANGLE);
                    outtakeTimer = futureTime(.3);
                    outtakeIndex++;
                }
                break;
            case 3:
                if(isPast(outtakeTimer)) {
                    shoulderTargetPosition = preferHighOuttake ? SHOULDER_HIGHOUTTAKE_POSITION : SHOULDER_LOWOUTTAKE_POSITION;
                    outtakeIndex ++;
                }
                break;
            case 4:
                if (!sampleDetected()) {
                    return true;
                }
                break;
        }
        return false;
    }

    public void sample(List<Sample> samples) {
        articulation = Articulation.INTAKE;
        this.targetSamples = samples;
    }

    public void sample(Constants.Alliance alliance) {
        List<Sample> samples = new ArrayList<>();
        if (alliance.isRed()) samples.add(Sample.RED);
        else samples.add(Sample.BLUE);
        samples.add(Sample.NEUTRAL);
        articulation = Articulation.INTAKE;
        this.targetSamples = samples;
    }

    public boolean sampleDetected() {
        return targetSamples.contains(currentSample);
    }

    public boolean stopOnSample() {
        colorSensorEnabled = true;
        beaterPower = .8;
        if (targetSamples.contains(currentSample)) {
            beaterPower = 0;
            colorSensorEnabled = false;
            return true;
        }
        return false;
    }

    public Trident(HardwareMap hardwareMap, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        elbow = new Joint(hardwareMap, "elbow", false, ELBOW_HOME_POSITION, ELBOW_PWM_PER_DEGREE, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE, ELBOW_START_ANGLE, ELBOW_JOINT_SPEED);
        DcMotorEx bruh = this.hardwareMap.get(DcMotorEx.class, "slide");
        DcMotorEx bruhx2 = this.hardwareMap.get(DcMotorEx.class, "shoulder");
        slide = new DcMotorExResetable(bruh);
        shoulder = new DcMotorExResetable(bruhx2);
        colorSensor = this.hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");
        beater = this.hardwareMap.get(CRServo.class, "beater");
        pincer = this.hardwareMap.get(Servo.class, "pincer");
        slide.setMotorEnable();
        slide.setPower(1);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setVelocity(SLIDE_SPEED);

        shoulder.setMotorEnable();
        shoulder.setPower(1);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setTargetPosition(0);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setVelocity(1500);
        articulation = Articulation.MANUAL;
        SHOULDER_CALIBRATE_ENCODER = Integer.MIN_VALUE;
    }

    public void adjustElbow(double angle) {
        elbow.setTargetAngle(elbow.getCurrentAngle() + angle);
    }


    @Override
    public void update(Canvas fieldOverlay) {
        //compute the current articulation/behavior
        articulate();
        //allow real-time flipper speed changes
        elbow.setSpeed(ELBOW_JOINT_SPEED);

        if (colorSensorEnabled) {
            updateColorSensor();
            colorSensor.setGain(colorSensorGain);
        }


        //actually instruct actuators to go to desired targets
        elbow.update();
        if (enforceSlideLimits) intakeSlideLimits();
        slide.setTargetPosition(slideTargetPosition);
        if (calibrated) {
            shoulder.setTargetPosition(shoulderTargetPosition);
        }
        beater.setPower(-beaterPower);

//        if(pincerEnabled) {
//            pincer.setPosition(Utils.servoNormalize(PINCER_OPEN));
//        }
//        else {
//            pincer.setPosition(Utils.servoNormalize(PINCER_CLOSE));
//        }

    }

    public String updateColorSensor() {
        double hue = getHSV()[0];
        if (hue < 35 && hue > 30) {
            currentSample = Sample.NEUTRAL;
            return "NEUTRAL";
        } else if (hue < 5 || hue > 350) {
            currentSample = Sample.RED;
            return "RED";
        } else if (hue < 220 && hue > 210) {
            currentSample = Sample.BLUE;
            return "BLUE";
        } else {
            currentSample = Sample.NO_SAMPLE;
            return "NO SAMPLE";
        }
    }

    @Override
    public void stop() {
        slide.setMotorDisable();
        beater.setPower(Utils.servoNormalize(1500));
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
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("articulation", articulation.name());
        telemetryMap.put("preferHighOuttake", preferHighOuttake);
        telemetryMap.put("intake index", intakeIndex);
        telemetryMap.put("outtake index", outtakeIndex);
        telemetryMap.put("shoulder current", shoulder.getCurrent(CurrentUnit.AMPS));
        telemetryMap.put("shoulder target : real", "" + shoulderTargetPosition + " : " + shoulder.getCurrentPosition());
        if(robot.fetchedPosition != null)
            telemetryMap.put("shoulder fetched pos", robot.fetchedPosition.getShoulderPosition());
        telemetryMap.put("shoulder offset", shoulder.offset);
        telemetryMap.put("slide target : real", slideTargetPosition + " : " + slide.getCurrentPosition());
        telemetryMap.put("current sample", currentSample.name());
        telemetryMap.put("colorsensor", colorSensorEnabled);
        if (colorSensorEnabled) {
            telemetryMap.put("colorsensor hsv", "" + HSVasString());
            telemetryMap.put("colorsensor rgb", colorSensor.getNormalizedColors().red + " " + colorSensor.getNormalizedColors().green + " " + colorSensor.getNormalizedColors().blue);
        }
//        0 40 214
        telemetryMap.put("beater speed", beater.getPower());
        telemetryMap.put("elbow angle target : real", elbow.getTargetAngle() + " : " + elbow.getCurrentAngle());
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "TRIDENT";
    }
}
