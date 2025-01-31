package org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.samplers;

import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.alliance;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;
import static org.firstinspires.ftc.teamcode.util.utilMethods.withinError;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Trident;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.DcMotorExResetable;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Joint;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "00_ITD_SAMPLER")
public class Sampler extends Arm {
    CRServo beater = null;
    //public DcMotorEx slide = null;

    //ELBOW JOINT VARIABLES
    public static boolean preferHighOuttake = true;

    public int slideTargetPosition = 0;

    // Shoulder values to request from Trident
    public static int shoulderSpeed = 45;
    public static int SHOULDER_HOME_POSITION = 250;
    public static int SHOULDER_PREINTAKE_POSITION = -100;
    public static int SHOULDER_INTAKE_POSITION = -375;
    public static int SHOULDER_LOWOUTTAKE_POSITION = 2105;
    public static int SHOULDER_HIGHOUTTAKE_POSITION = 1385;
    public static int SLIDE_ADJUST_SPEED = 80;
    public int shoulderPositionMax = 850;

    public static int colorSensorGain = 12;

    public Sampler(HardwareMap hardwareMap, Robot robot, Trident trident) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        this.trident = trident; // to request services from Trident - mainly setting the shoulder angle

        articulation = Articulation.MANUAL;

        SLIDE_HIGHOUTTAKE_POSITION = 2320;


        //defaults specific to sampler
        ELBOW_START_ANGLE = 140;
        ELBOW_HOME_POSITION = 2050;
        ELBOW_PWM_PER_DEGREE = -5.672222222222222;
        ELBOW_JOINT_SPEED = 120;
        ELBOW_MIN_ANGLE = -15;
        ELBOW_MAX_ANGLE = 220;
        ELBOW_ADJUST_ANGLE = 5;
        ELBOW_PREINTAKE_ANGLE = 5;
        ELBOW_LOWOUTTAKE_ANGLE = 102;
        ELBOW_HIGHOUTTAKE_ANGLE = 60;

        elbow = new Joint(hardwareMap, "samplerElbow", false, ELBOW_HOME_POSITION, ELBOW_PWM_PER_DEGREE, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE, ELBOW_START_ANGLE, ELBOW_JOINT_SPEED);
        DcMotorEx bruh = this.hardwareMap.get(DcMotorEx.class, "samplerSlide");
        slide = new DcMotorExResetable(bruh);
        slide.setMotorEnable();
        slide.setPower(1);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setVelocity(SLIDE_SPEED);
        colorSensor = this.hardwareMap.get(NormalizedColorSensor.class, "samplerSensor");
        beater = this.hardwareMap.get(CRServo.class, "samplerBeater");
    }

    public boolean sampleDetected() {
        updateColorSensor();
        return targetSamples.contains(currentSample);
    }

    @Override
    public boolean stopOnSample() {
        servoPower = 0.8;
        if (sampleDetected()) {
            servoPower = 0;
            return true;
        }
        return false;
    }

    public void adjustElbow(double angle) {
        elbow.setTargetAngle(elbow.getCurrentAngle() + angle);
    }

    @Override
    public void update(Canvas fieldOverlay) {
        beater.setPower(-servoPower);
        elbow.update();
        if (trident.calibrated) {
            if(slideTargetPosition < 0)
                slideTargetPosition = 0;
            slide.setTargetPosition(slideTargetPosition);
            //allow real-time elbow speed changes
            elbow.setSpeed(ELBOW_JOINT_SPEED);
//            trident.setShoulderTarget(this, shoulderTargetPosition);
        }
        if (colorSensorEnabled) {
             updateColorSensor();
        }
        //compute the current articulation/behavior
        articulate();
    }

    @Override
    public void stop() {
        servoPower = 0;
        beater.setPower(0);
        slide.setTargetPosition(slide.getCurrentPosition());
    }

    @Override
    public void resetStates() {
        intakeIndex = 0;
        outtakeIndex = 0;
        tuckIndex = 0;
        calibrateIndex = 0;
    }

    public enum Articulation {
        MANUAL, //does nothing - used for transition tracking
        CALIBRATE,
        TUCK, // safely tuck the arm out of the way
        INTAKE_PREP, INTAKE, OUTTAKE
    }

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
                if (calibrate(this, ELBOW_START_ANGLE)) {
                    articulation = Articulation.MANUAL;
                }
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

            case INTAKE_PREP:
                samplerPrep();
                break;

            default:
                throw new RuntimeException("how the ^%*( did you get here?");
        }
        prevArticulation = articulation;
        return articulation;
    }

    @Override
    public void adjustSlide(int adjustTicks) {
        slideTargetPosition += adjustTicks;
    }

    public boolean finalizeTargets() {
        targetSamples = new ArrayList<Sample>();
        if (alliance.isRed()) {
            targetSamples.add(Sample.RED);
        } else {
            targetSamples.add(Sample.BLUE);
        }
        targetSamples.add(Sample.NEUTRAL);
        return true;
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
                    trident.setShoulderTarget(this, SHOULDER_INTAKE_POSITION);
                    slideTargetPosition = SLIDE_PREINTAKE_POSITION;
                    intakeIndex++;
                }
                break;
            case 2:
                if (withinError(trident.getShoulderCurrentPosition(), SHOULDER_INTAKE_POSITION, 10) && withinError(slide.getCurrentPosition(), SLIDE_PREINTAKE_POSITION, 10)) {
                    servoPower = .8;
                    intakeTimer = futureTime(5);
                    intakeIndex++;
                    colorSensorEnabled = true;
                }
                break;
            case 3:
                if (slideTargetPosition > SLIDE_INTAKE_MIN_POSITION) {
                    slideTargetPosition -= 20;
                    trident.setShoulderTarget(this, (int)(trident.getShoulderTarget()- 20 * 0.1534090909090909));
                }
                if (stopOnSample() || isPast(intakeTimer)) {
                    intakeIndex = 0;
                    trident.setShoulderTarget(this, SHOULDER_PREINTAKE_POSITION);
                    return true;
                }
                break;
        }
        return false;
    }

    public boolean samplerPrep() {
        // all we want to do here is set the sampler out horizontal to maximum horizontal extension
        // and just above sample or wall height
        // this should happen while the driver is approaching the target, usually in the sub
        // if there is a danger of contact with another robot, driver should trigger tuck

        elbow.setTargetAngle(ELBOW_PREINTAKE_ANGLE);
        trident.setShoulderTarget(this, SHOULDER_PREINTAKE_POSITION);
        slideTargetPosition = SLIDE_PREINTAKE_POSITION;
        return true;
    }


    public static int outtakeIndex;
    public long outtakeTimer;

    @Override
    public boolean outtake() {
        switch (outtakeIndex) {
            case 0:
//                elbow.setTargetAngle();
//                outtakeTimer = futureTime(0);
                elbow.setTargetAngle(preferHighOuttake ? ELBOW_HIGHOUTTAKE_ANGLE : ELBOW_LOWOUTTAKE_ANGLE);
                outtakeIndex++;
                break;
            case 1:
//                if (isPast(outtakeTimer)) {
                slideTargetPosition = preferHighOuttake ? SLIDE_HIGHOUTTAKE_POSITION : SLIDE_LOWOUTTAKE_POSITION;
                trident.setShoulderTarget(this, preferHighOuttake ? (int) (SHOULDER_HIGHOUTTAKE_POSITION * .75) : SHOULDER_LOWOUTTAKE_POSITION);
                outtakeIndex++;
//                }
                break;
            case 2:
                trident.setShoulderTarget(this, preferHighOuttake ? (int) (SHOULDER_HIGHOUTTAKE_POSITION) : SHOULDER_LOWOUTTAKE_POSITION);
//                if (withinError(slide.getCurrentPosition(), preferHighOuttake ? SLIDE_HIGHOUTTAKE_POSITION : SLIDE_LOWOUTTAKE_POSITION, 10)) {
//                    elbow.setTargetAngle(preferHighOuttake ? ELBOW_HIGHOUTTAKE_ANGLE : ELBOW_LOWOUTTAKE_ANGLE);
//                    outtakeTimer = futureTime(.3);
//                    outtakeIndex++;
//                }
                outtakeIndex ++;
                break;
            case 3:
//                if(isPast(outtakeTimer)) {
//                    shoulderTargetPosition = preferHighOuttake ? SHOULDER_HIGHOUTTAKE_POSITION : SHOULDER_LOWOUTTAKE_POSITION;
//                    outtakeIndex ++;
//                }
                outtakeIndex++;
                break;
            case 4:
                if (!sampleDetected()) {
                    outtakeIndex = 0;
                    return true;
                }
                break;
        }
        return false;
    }

    public long tuckTimer = 0;
    public static int tuckIndex = 0;

    @Override
    public boolean tuck() {
        switch (tuckIndex) {
            case 0:
                elbow.setTargetAngle(ELBOW_START_ANGLE);
                slideTargetPosition = 0;
                servoPower = 0;
                tuckIndex++;
               return true;

        }
        return false;
    }

    @Override
    public String updateColorSensor() {
        colorSensor.setGain(colorSensorGain);
        double hue = getHSV()[0];
        if (hue > 60 && hue < 80) {
            currentSample = Sample.NEUTRAL;
            return "NEUTRAL";
        } else if (hue < 20) {
            currentSample = Sample.RED;
            return "RED";
        } else if (hue < 23 5 && hue > 200) {
            currentSample = Sample.BLUE;
            return "BLUE";
        } else {
            currentSample = Sample.NO_SAMPLE;
            return "NO SAMPLE";
        }
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("Articulation", articulation);
        telemetryMap.put("preferHighOuttake", preferHighOuttake);
        telemetryMap.put("intake index", intakeIndex);
        telemetryMap.put("outtake index", outtakeIndex);
        telemetryMap.put("tuck index", tuckIndex);
        telemetryMap.put("tuck timer", isPast(tuckTimer));
        telemetryMap.put("slide target : real", slideTargetPosition + " : " + slide.getCurrentPosition());

        telemetryMap.put("current sample", currentSample.name());
        telemetryMap.put("colorsensor hsv", "" + HSVasString()); // cached - changes when HSV is read
        telemetryMap.put("colorsensor rgb", colorLastRGBA.red + " " + colorLastRGBA.green + " " + colorLastRGBA.blue);
//        0 40 214
        telemetryMap.put("beater speed", beater.getPower());
        telemetryMap.put("elbow angle target : real", elbow.getTargetAngle() + " : " + elbow.getCurrentAngle());

        telemetryMap.put("calibrate stage", calibrateIndex);

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "Sampler";
    }
}
