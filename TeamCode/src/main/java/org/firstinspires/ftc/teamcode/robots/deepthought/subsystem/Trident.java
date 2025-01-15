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
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.samplers.Sampler;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.samplers.SpeciMiner;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.DcMotorExResetable;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Joint;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils;

import java.util.*;


@Config(value = "00_ITD_TRIDENT")
public class Trident implements Subsystem {
    HardwareMap hardwareMap;
    Robot robot;

    public Sampler sampler;
    public SpeciMiner speciMiner;
    public static boolean enforceSlideLimits;
    public DcMotorExResetable slide = null;
    public DcMotorExResetable shoulder = null;
    public Joint elbow;

    public NormalizedColorSensor colorSensor = null;
    public static boolean colorSensorEnabled = false;

    public static int colorSensorGain = 12;

    public boolean calibrated = false;

    public final boolean useSampler = true;
    public final boolean useSpeciminer = false;


    public enum Sample {
        RED, BLUE, NEUTRAL, NO_SAMPLE

    }

    public Sample currentSample = Sample.NO_SAMPLE;
    public List<Sample> targetSamples = new ArrayList<>();

    public static boolean preferHighOuttake = true;


    //shoulder - these are defaults - but the Sampler and Speciminer classes define their own local versions
    public static int SHOULDER_CALIBRATE_ENCODER = Integer.MIN_VALUE;
    public int shoulderTargetPosition = 0;
    public static int shoulderSpeed = 45;
    public static int SHOULDER_HOME_POSITION = 250;
    public static int SHOULDER_INTAKE_POSITION = 250;
    public static int SHOULDER_LOWOUTTAKE_POSITION = 2105;
    public static int SHOULDER_HIGHOUTTAKE_POSITION = 1925;
    public int shoulderPositionMax = 850;

    //ELBOW JOINT VARIABLES - todo these are unused and need to be removed - they now exist locally in Sampler and Specimer
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
        INTAKE_SAMPLER, OUTTAKE_SAMPLER,
        SAMPLER_TUCK, FLOOR_PREP, SAMPLER_FLOOR, SAMPLER_BASKET, SPECIMINER_TUCK, SPECIMINER_FLOOR, SPECIMINER_WALL, SPECIMINER_BAR
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
            case TUCK: // todo purge - tuck should be called directly on the Sampler and/or Speciminer - with variations that don't affect the shoulder
                if (tuck()) {
                    articulation = Articulation.MANUAL;
                }
                break;
            //SHOULD ONLY BE ACCESSED BY SAMPLE()
            case INTAKE_SAMPLER:
                if (intakeSampler()) {
                    articulation = Articulation.MANUAL;
                }
                break;

            case OUTTAKE_SAMPLER:
                if (outtakeSampler()) articulation = Articulation.MANUAL;
                break;

            default:
                throw new RuntimeException("how the ^%*( did you get here?");
        }
        prevArticulation = articulation;
        return articulation;
    }

    public long tuckTimer = 0;
    public static int tuckIndex = 0;

    public boolean tuck() { //todo tuck needs to be tested after refactoring - this only does sampler.tuck()
        //todo dangerous way of calling tuck in sampler - sampler articulations don't know it's happening
        return sampler.tuck();
        // this won't work either as first line must only be called once
        // sampler.articulate(Sampler.Articulation.TUCK);
        //return (sampler.articulation==Sampler.Articulation.MANUAL)? true : false;
        // kinda moot, shouldn't really be a tuck operation at this level
        // this is a relic of short term migration needs
    }

    public boolean intakeSampler() {
        return sampler.intake(); // todo cheapo delegation - fixup later
    }

    public boolean outtakeSampler() { //todo - trident shouldn't be in the business of passing sampler and speciminer articulations through
        return sampler.outtake();
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
        //moar subsystems
        sampler = new Sampler(hardwareMap, robot, this);
        //speciMiner = new SpeciMiner(hardwareMap, robot, this);

        //elbow = new Joint(hardwareMap, "elbow", false, ELBOW_HOME_POSITION, ELBOW_PWM_PER_DEGREE, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE, ELBOW_START_ANGLE, ELBOW_JOINT_SPEED);
        //DcMotorEx bruh = this.hardwareMap.get(DcMotorEx.class, "slide");
        DcMotorEx bruhx2 = this.hardwareMap.get(DcMotorEx.class, "shoulder");
        //slide = new DcMotorExResetable(bruh);
        shoulder = new DcMotorExResetable(bruhx2);
        //colorSensor = this.hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");
        //beater = this.hardwareMap.get(CRServo.class, "beater");
//        pincer = this.hardwareMap.get(Servo.class, "pincer");
//        slide.setMotorEnable();
//        slide.setPower(1);
//        slide.setDirection(DcMotorSimple.Direction.REVERSE);
//        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slide.setTargetPosition(0);
//        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slide.setVelocity(SLIDE_SPEED);

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
////        elbow.setSpeed(ELBOW_JOINT_SPEED);
//
//        if (colorSensorEnabled) {
//            updateColorSensor();
//            colorSensor.setGain(colorSensorGain);
//        }


        //actually instruct actuators to go to desired targets
//        elbow.update();
//        if (enforceSlideLimits) intakeSlideLimits();
//        slide.setTargetPosition(slideTargetPosition);
        if (calibrated) {
            shoulder.setTargetPosition(shoulderTargetPosition);
        }
//        beater.setPower(-beaterPower);

//        if(pincerEnabled) {
//            pincer.setPosition(Utils.servoNormalize(PINCER_OPEN));
//        }
//        else {
//            pincer.setPosition(Utils.servoNormalize(PINCER_CLOSE));
//        }

    }

    @Override
    public void stop() {
    shoulderTargetPosition=shoulder.getCurrentPosition();
    }


    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("articulation", articulation.name());
        telemetryMap.put("preferHighOuttake", preferHighOuttake);
        telemetryMap.put("shoulder current", shoulder.getCurrent(CurrentUnit.AMPS));
        telemetryMap.put("shoulder target : real", "" + shoulderTargetPosition + " : " + shoulder.getCurrentPosition());
        if(robot.fetchedPosition != null)
            telemetryMap.put("shoulder fetched pos", robot.fetchedPosition.getShoulderPosition());
        telemetryMap.put("shoulder offset", shoulder.offset);
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "TRIDENT";
    }
}
