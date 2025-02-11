package org.firstinspires.ftc.teamcode.robots.deepthought.subsystem;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;
import static org.firstinspires.ftc.teamcode.util.utilMethods.withinError;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.samplers.Arm;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.samplers.Sampler;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.samplers.SpeciMiner;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.DcMotorExResetable;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Joint;

import java.util.*;


@Config(value = "00_ITD_TRIDENT")
public class Trident implements Subsystem {
    HardwareMap hardwareMap;
    Robot robot;

    public Sampler sampler;
    public SpeciMiner speciMiner;

    Arm activeArm; //just to keep track of which arm is actively requesting shoulder services
    public static boolean enforceSlideLimits;
    DcMotorExResetable shoulder = null;
    public Joint elbow;

    public NormalizedColorSensor colorSensor = null;
    //public static boolean colorSensorEnabled = false;

    public boolean calibrated = true;

    public void finalizeTargets() {
        sampler.finalizeTargets();
        speciMiner.finalizeTargets();
    }

    public void setActiveArm(Arm arm) {
        this.activeArm = arm;
        if (arm instanceof Sampler) {
            speciMiner.articulate(SpeciMiner.Articulation.TUCK);
        } else {
            sampler.articulate(Sampler.Articulation.TUCK);
        }
    }

    public enum Sample {
        RED, BLUE, NEUTRAL, NO_SAMPLE
    }

    public Sample currentSample = Sample.NO_SAMPLE;
    public List<Sample> targetSamples = new ArrayList<>();

    public static boolean preferHighOuttake = true;


    //shoulder - these are defaults - but the Sampler and Speciminer classes define their own local versions
    public int SHOULDER_CALIBRATE_ENCODER = Integer.MIN_VALUE;
    public static int shoulderTargetPosition = 0;
    public static int shoulderSpeed = 45;
    public static int SHOULDER_CALIBRATE_HORIZONTAL = -2020; // offset to get to horizontal when shoulder is at max
    public static int SHOULDER_SIZING = 540;  //todo re-tune after horizontal tuning
    public int SHOULDER_HORIZONTAL = 0;
    public static int SHOULDER_MIN_POSITION = -625;


    //BEATER
    public static double beaterPower;

    //PINCER
    public static double PINCER_CLOSE = 1750;
    public static double PINCER_OPEN = 2150;
    public boolean pincerEnabled = true;


    public enum Articulation {
        MANUAL, //does nothing - used for transition tracking
        CALIBRATE,
        TUCK, //does nothing - used for transition tracking
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
            case CALIBRATE:
                if (calibrate()) {
                    articulation = Articulation.MANUAL;
                }
                break;
            //this is master tuck, calls both tucks + shoulder and should only rarely be used
            case TUCK:
                if (tuck()) {
                    articulation = Articulation.MANUAL;
                }
                break;
            default:
                throw new RuntimeException("how the ^%*( did you get here?");
        }
        prevArticulation = articulation;
        return articulation;
    }

    public int calibrateIndex = 0;
    public long calibrateTimer = 0;

    public boolean calibrate() {
        switch (calibrateIndex) {
            case 0: //tuck the arms and start moving the shoulder up until it stalls
                calibrated = false;
                sampler.articulate(Sampler.Articulation.CALIBRATE);
                speciMiner.articulate(SpeciMiner.Articulation.CALIBRATE);
                calibrateTimer = futureTime(1); //enough time to assure it has begun moving
                shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shoulder.setPower(.3);
                calibrateIndex++;
                break;
            case 1: // has the arm stopped moving?
                if (SHOULDER_CALIBRATE_ENCODER == shoulder.getCurrentPosition() && isPast(calibrateTimer)) {
                    calibrateIndex++;
                } else {
                    SHOULDER_CALIBRATE_ENCODER = shoulder.getCurrentPosition();
                }
                break;
            case 2: // lower to horizontal - experimentally determined offset
                shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                shoulder.setTargetPosition(SHOULDER_CALIBRATE_HORIZONTAL);
                shoulder.setPower(1);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                calibrateIndex++;
                break;

            case 3: // reset the encoders to make horizontal zero
                if (withinError(shoulder.getCurrentPosition(), SHOULDER_CALIBRATE_HORIZONTAL, 3)) {
                    shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //shoulder.setDirection(DcMotorSimple.Direction.REVERSE);
                    shoulder.setPower(1);
                    shoulder.setVelocity(400);
                    shoulder.setTargetPosition(0);
                    shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    calibrateIndex++;
                }
                break;
            case 4:
                shoulderTargetPosition = SHOULDER_SIZING;

                speciMiner.shoulderTargetPosition = shoulderTargetPosition;
                calibrateIndex = 0;
                calibrated = true;
                return true;
        }
        return false;
    }

    public long tuckTimer = 0;
    public static int tuckIndex = 0;

    public boolean tuck() { //todo tuck needs to be tested after refactoring - this only does sampler.tuck()
        switch (tuckIndex) {
            case 0:
                shoulderTargetPosition = SHOULDER_HORIZONTAL;

                speciMiner.shoulderTargetPosition = SHOULDER_HORIZONTAL;
                tuckIndex++;
                break;
            case 1:
                if (withinError(shoulder.getCurrentPosition(), SHOULDER_HORIZONTAL, 500)) {
                    Sampler.tuckIndex = 0;
                    sampler.articulate(Sampler.Articulation.TUCK);
//                SpeciMiner.tuckIndex = 0;
                    speciMiner.articulate(SpeciMiner.Articulation.TUCK);
                }
            case 2:
                if (sampler.slide.getCurrentPosition() < 150 && speciMiner.slide.getCurrentPosition() < 150) {
                    return true;
                }
                break;
        }
        return false;
        // this won't work either as first line must only be called once
        // sampler.articulate(Sampler.Articulation.TUCK);
        //return (sampler.articulation==Sampler.Articulation.MANUAL)? true : false;
        // kinda moot, shouldn't really be a tuck operation at this level
        // this is a relic of short term migration needs
    }

    public boolean sampleDetected() {
        return targetSamples.contains(currentSample);
    }

    public boolean stopOnSample() {
        return activeArm.stopOnSample();
    }

    public Trident(HardwareMap hardwareMap, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        //moar subsystems
        sampler = new Sampler(hardwareMap, robot, this);
        speciMiner = new SpeciMiner(hardwareMap, robot, this);
        activeArm = sampler; //just a default - can change any time

        DcMotorEx bruhx2 = this.hardwareMap.get(DcMotorEx.class, "shoulder");
        shoulder = new DcMotorExResetable(bruhx2);
        shoulder.setDirection(DcMotorSimple.Direction.REVERSE);
        shoulder.setMotorEnable();
        shoulder.setPower(1);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setTargetPosition(0);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setVelocity(1500);
        SHOULDER_CALIBRATE_ENCODER = Integer.MIN_VALUE;
        articulation = Articulation.MANUAL;
    }

    // shoulder services
    public int getShoulderTarget() {
        return shoulderTargetPosition;
    }

    public int getShoulderCurrentPosition() {
        return shoulder.getCurrentPosition();
    }

    public void setShoulderTarget(Arm subsystem, int targetTics) {
        setActiveArm(subsystem);
        setShoulderTarget(targetTics);
    }

    private void setShoulderTarget(int targetTics) {
        if (targetTics > SHOULDER_MIN_POSITION) {
            shoulderTargetPosition = targetTics;
        } else {
            shoulderTargetPosition = SHOULDER_MIN_POSITION;
        }
    }

    public Arm getActiveArm() {
        return activeArm;
    }

    public boolean isSamplerActive() {
        return activeArm instanceof Sampler;
    }

    public DcMotorExResetable getShoulder() {
        return shoulder;
    }

    @Override
    public void update(Canvas fieldOverlay) {
        //compute the current articulation/behavior
        articulate();
        if (calibrated) {

            shoulder.setTargetPosition(shoulderTargetPosition);
        }

        sampler.update(fieldOverlay);
        speciMiner.update(fieldOverlay);

    }

    @Override
    public void stop() {
//    shoulderTargetPosition=shoulder.getCurrentPosition();
    }

    @Override
    public void resetStates() {
        tuckIndex = 0;
        calibrateIndex = 0;
        sampler.resetStates();
        speciMiner.resetStates();
    }


    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("articulation", articulation.name());
        telemetryMap.put("preferHighOuttake", preferHighOuttake);
        telemetryMap.put("activeArm", activeArm.getTelemetryName());
        telemetryMap.put("shoulder current", shoulder.getCurrent(CurrentUnit.AMPS));
        telemetryMap.put("shoulder target : real", "" + shoulderTargetPosition + " : " + shoulder.getCurrentPosition());
        if (robot.fetchedPosition != null)
            telemetryMap.put("shoulder fetched pos", robot.fetchedPosition.getShoulderPosition());
        telemetryMap.put("shoulder offset", shoulder.offset);

        telemetryMap.put("calibrate stage", calibrateIndex);

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "TRIDENT";
    }
}
