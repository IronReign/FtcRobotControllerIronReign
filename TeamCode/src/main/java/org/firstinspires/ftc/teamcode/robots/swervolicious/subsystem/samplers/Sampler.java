package org.firstinspires.ftc.teamcode.robots.swervolicious.subsystem.samplers;

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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.DcMotorExResetable;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Joint;
import org.firstinspires.ftc.teamcode.robots.swervolicious.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.swervolicious.subsystem.Trident;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;

//@Config(value = "00_ITD_SAMPLER")
public class Sampler extends Arm {
    public static double TUNABLE_COEFFICIENT = 100;
    CRServo beater = null;
    //public DcMotorEx slide = null;

    //ELBOW JOINT VARIABLES
    public static boolean preferHighOuttake = true;

    public int getSlideTargetPosition() {
        return slideTargetPosition;
    }

    public void setSlideTargetPosition(int slideTargetPosition) {
        this.slideTargetPosition = slideTargetPosition;
    }

    int slideTargetPosition = 0;

    // Shoulder values to request from Trident
    public static int shoulderSpeed = 45;
    public static int SHOULDER_HOME_POSITION = 250;
    public static int SHOULDER_PREINTAKE_POSITION = -150;
    public static int SHOULDER_INTAKE_POSITION = -305;
    public static int SHOULDER_LOWOUTTAKE_POSITION = 2105;
    public static int SHOULDER_HIGHOUTTAKE_POSITION = 1485;
    public static int SLIDE_ADJUST_SPEED = 80;

    // sweep config uses sampler to slide samples to ozone
    // this sweep config is right at 42"
    public static int SWEEP_SLIDE_POS = 2240;
    public static int SWEEP_SHOULDER_POS = -220;

    int SWEEP_OVER_SHOULDER_POS = -85;

    // note for Sweep returning to alliance samples, set shoulder to horizontal
    public static double SWEEP_ELBOW_ANGLE = 205; // was 0 for axon

    double ELBOW_TUCK_ANGLE = 30; //softly resting on CF tube - was 140 for axon

    double ElbowBowHigh = 90;  // vertical
    double ElbowBowLow = SWEEP_ELBOW_ANGLE; // out and down

    int SlideBowOut = 1500;

    DigitalChannel digiColorSensorP0, digiColorSensorP1;
    public int shoulderPositionMax = 850;

    public static int colorSensorGain = 12;
    public static int elbowRange = 10;

    public Sampler(HardwareMap hardwareMap, Robot robot, Trident trident) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        this.trident = trident; // to request services from Trident - mainly setting the shoulder angle

        articulation = Articulation.MANUAL;
        SLIDE_HIGHOUTTAKE_POSITION = 1900;



        //defaults specific to sampler
        ELBOW_START_ANGLE = 0; // was 140 for axon - 190 for blue dsservo45 is pressing firmly
        ELBOW_HOME_POSITION = 900; // 2050 for axon  - 900 for dsservo 45kg
        ELBOW_PWM_PER_DEGREE = 5.672222222222222; // -5.672222222222222 for axon - the dsservo 45kg goes the other way
        ELBOW_JOINT_SPEED = 120;
        ELBOW_MIN_ANGLE = 0; // -15 for Axon
        ELBOW_MAX_ANGLE = 220;  // 220 for Axon
        ELBOW_ADJUST_ANGLE = 5;
        ELBOW_PREINTAKE_ANGLE = 150;  // to clear over the sub wall // 5 for axon
        ELBOW_LOWOUTTAKE_ANGLE = 110; // 120 for axon
        ELBOW_HIGHOUTTAKE_ANGLE = 120; // 60 for axon

        TUNABLE_COEFFICIENT = ELBOW_PREINTAKE_ANGLE;

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
        //colorSensor = this.hardwareMap.get(NormalizedColorSensor.class, "samplerSensor");
        digiColorSensorP0 = this.hardwareMap.get(DigitalChannel.class, "samplerP0");
        digiColorSensorP1 = this.hardwareMap.get(DigitalChannel.class, "samplerP1");

        beater = this.hardwareMap.get(CRServo.class, "samplerBeater");
    }

    public boolean sampleDetected() {
        updateColorSensor();
        return targetSamples.contains(currentSample);
    }

    @Override
    public boolean stopOnSample() {
        servoPower = beaterIntakeSpeed;
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
//        ELBOW_PREINTAKE_ANGLE = TUNABLE_COEFFICIENT;
        updateColorSensor(); // can do this without penalty when calling digital brushlabs color sensor

        beater.setPower(-servoPower);
        elbow.update();
        if (trident.calibrated) {
            if (slideTargetPosition < 0) slideTargetPosition = 0;
            slide.setTargetPosition(slideTargetPosition);
            //allow real-time elbow speed changes
            elbow.setSpeed(ELBOW_JOINT_SPEED);
//            trident.setShoulderTarget(this, shoulderTargetPosition);
        }
        ELBOW_PREINTAKE_ANGLE = TUNABLE_COEFFICIENT;

        //compute the current articulation/behavior
        articulate();
    }

    @Override
    public void stop() {
        servoPower = 0;
        beater.setPower(0);
        setElbowAngle(ELBOW_START_ANGLE);
        try {
            wait(250);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        slide.setTargetPosition(slide.getCurrentPosition());
    }

    @Override
    public void resetStates() {
        intakeIndex = 0;
        outtakeIndex = 0;
        tuckIndex = 0;
        calibrateIndex = 0;
        samplerPrepIndex = 0;
    }

    @Override
    public boolean calibrate() {
        return true; // stub - implement if needed
    }

    public void setElbowAngle(double elbowAngle) {
        elbow.setTargetAngle(elbowAngle);
    }

    public enum Articulation {
        MANUAL, //does nothing - used for transition tracking
        CALIBRATE, TUCK, // safely tuck the arm out of the way
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
            case 2: // start the intake beater when shoulder and slide are in position
                if (withinError(trident.getShoulderCurrentPosition(), SHOULDER_INTAKE_POSITION, 10) && withinError(slide.getCurrentPosition(), SLIDE_PREINTAKE_POSITION, 10)) {
                    servoPower = beaterIntakeSpeed;
                    intakeTimer = futureTime(2);
                    intakeIndex++;
                    colorSensorEnabled = true;
                }
                break;
            case 3:
                if (slideTargetPosition > SLIDE_INTAKE_MIN_POSITION) {
                    slideTargetPosition = 0;
                    int slideRange = 1040;
                    int slideProgress = ((trident.getShoulderCurrentPosition()))/slideRange;
                    int shoulderRange = 90;
                    int interpShoulder= -420 + slideProgress*shoulderRange;
                    int interpElbow = 150 + slideProgress*elbowRange;
                    //trident.setShoulderTarget(this, (int) (trident.getShoulderTarget() - 200 * 0.44090909090909));
                    trident.setShoulderTarget(this, interpShoulder);
                    setElbowAngle(interpElbow);

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

    int samplerPrepIndex = 0;

    public boolean samplerPrep() {
        // all we want to do here is set the sampler out horizontal to maximum horizontal extension
        // and just above sample or wall height
        // this should happen while the driver is approaching the target, usually in the sub
        // if there is a danger of contact with another robot, driver should trigger tuck

        switch (samplerPrepIndex) {
            case 0:
                trident.setShoulderTarget(this, SHOULDER_PREINTAKE_POSITION);
                samplerPrepIndex++;
                break;
            case 1:
                if (trident.getShoulderCurrentPosition() < 1000)
                    elbow.setTargetAngle(ELBOW_PREINTAKE_ANGLE);
                slideTargetPosition = SLIDE_PREINTAKE_POSITION;
                samplerPrepIndex = 0;
                return true;


        }
        return false;
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
                slideTargetPosition = SLIDE_HIGHOUTTAKE_POSITION;
                outtakeIndex++;
                break;
            case 1:
                if (slide.getCurrentPosition() > slideTargetPosition * .2) {
                    trident.setShoulderTarget(this, preferHighOuttake ? (int) (SHOULDER_HIGHOUTTAKE_POSITION) : SHOULDER_LOWOUTTAKE_POSITION);
                    outtakeIndex++;
                }
                break;
            case 2:

                if (withinError(slide.getCurrentPosition(), preferHighOuttake ? SLIDE_HIGHOUTTAKE_POSITION : SLIDE_LOWOUTTAKE_POSITION, 10)) {
//                    outtakeTimer = futureTime(.3);
                    outtakeIndex++;
                }

                break;
            case 3:
                if (withinError(trident.getShoulderCurrentPosition(), preferHighOuttake ? SHOULDER_HIGHOUTTAKE_POSITION : SHOULDER_LOWOUTTAKE_POSITION, 10)) {
                    outtakeIndex++;
                }
                break;
            case 4:

                break;
            case 5:
                slideTargetPosition += 170;
                servoPower = beaterIntakeSpeed *2;
                outtakeTimer = futureTime(.3);
                outtakeIndex++;
                break;
            case 6:
                if (!sampleDetected() && isPast(outtakeTimer)) {
                    elbow.setTargetAngle(180);
                    outtakeIndex = 0;
                    return true;
                }
                break;
        }
        return false;
    }

    public void incrementOuttake() {
        outtakeIndex++;
    }

    public long tuckTimer = 0;
    public static int tuckIndex = 0;

    @Override
    boolean tuck() {
        switch (tuckIndex) {
            case 0:
                elbow.setTargetAngle(ELBOW_TUCK_ANGLE);
                slideTargetPosition = 20;
                servoPower = 0;
                tuckIndex++;
                break;
            case 1:
                tuckIndex = 0;
                return true;
        }
        return false;
    }

    long bowTimer = 0;
    public int bowIndex = 0;

    public boolean bow() {
        switch (bowIndex) {
            case 0:  //assume this is triggered from a tucked position
                // extend the slide and raise the end effector elbow
                bowTimer = futureTime(2);
                elbow.setTargetAngle(ElbowBowHigh);
                slideTargetPosition = SlideBowOut;
                servoPower = 0;
                bowIndex++;
                break;
            case 1: // bow the elbow low
                if (isPast(bowTimer)) {
                    servoPower = beaterIntakeSpeed;
                    elbow.setTargetAngle(ElbowBowLow);
                    slideTargetPosition = SlideBowOut;
                    bowTimer = futureTime(2);
                    bowIndex++;
                }
                break;
            case 2: // return
                if (isPast(bowTimer)) {
                    slideTargetPosition = 20;
                    elbow.setTargetAngle(ELBOW_START_ANGLE);
                    servoPower = 0;
                    if (slide.getCurrentPosition() < 25) {
                        bowIndex = 0;
                        return true;
                    }
                }
                break;
        }
        return false;

    }

    @Override
    public String updateColorSensor() {

        if (digiColorSensorP0.getState()) {
            if (digiColorSensorP1.getState()) {
                currentSample = Sample.NEUTRAL;
                return "NEUTRAL";
            } else {
                currentSample = Sample.BLUE;
                return "BLUE";
            }
        } else {
            if (digiColorSensorP1.getState()) {
                currentSample = Sample.RED;
                return "RED";
            }
            else{
                currentSample = Sample.NO_SAMPLE;
                return "NO_SAMPLE";
            }
        }


//        colorSensor.setGain(colorSensorGain);
//        double hue = getHSV()[0];
//        if (hue > 35 && hue < 45) {
//            currentSample = Sample.NEUTRAL;
//            return "NEUTRAL";
//        } else if (hue < 26) {
//            currentSample = Sample.RED;
//            return "RED";
//        } else if (hue < 235 && hue > 200) {
//            currentSample = Sample.BLUE;
//            return "BLUE";
//        } else {
//            currentSample = Sample.NO_SAMPLE;
//            return "NO SAMPLE";
//        }
    }

    public long sweepTimer = 0;
    public static int sweepIndex = 0;

    public boolean sweepConfig(boolean flyOver) {
        switch (sweepIndex) {
            case 0:
                elbow.setTargetAngle(SWEEP_ELBOW_ANGLE);
                slideTargetPosition = SWEEP_SLIDE_POS;
                if (flyOver)
                    trident.setShoulderTarget(this, SWEEP_OVER_SHOULDER_POS);
                else
                    trident.setShoulderTarget(this, SWEEP_SHOULDER_POS);
                servoPower = 0;
                sweepIndex++;
                break;
            case 1: // wait until shoulder and slide have reached position to return true
                if (withinError(slideTargetPosition, slide.getCurrentPosition(), 10)
                        && withinError(trident.getShoulderTarget(), trident.getShoulderCurrentPosition(), 10)) {
                    sweepIndex = 0;
                    return true;
                }
        }
        return false;
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
        telemetryMap.put("slide amps", slide.getCurrent(CurrentUnit.AMPS));
        telemetryMap.put("current sample", currentSample.name());
        //telemetryMap.put("colorsensor hsv", "" + HSVasString()); // cached - changes when HSV is read
        //telemetryMap.put("colorsensor rgb", colorLastRGBA.red + " " + colorLastRGBA.green + " " + colorLastRGBA.blue);
//        0 40 214
        telemetryMap.put("beater speed", beater.getPower());
        telemetryMap.put("elbow angle target : real", elbow.getTargetAngle() + " : " + elbow.getCurrentAngle());

        telemetryMap.put("calibrate stage", calibrateIndex);
        telemetryMap.put("sweep stage", sweepIndex);

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "Sampler";
    }
}
