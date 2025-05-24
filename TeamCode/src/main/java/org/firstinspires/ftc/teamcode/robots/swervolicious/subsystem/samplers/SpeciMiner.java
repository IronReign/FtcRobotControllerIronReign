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
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Constants;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.DcMotorExResetable;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Joint;
import org.firstinspires.ftc.teamcode.robots.swervolicious.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.swervolicious.subsystem.Trident;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Config(value = "00_ITD_SPECIMINER")
public class SpeciMiner extends Arm {
    public CRServo CRSOne;
    public CRServo CRSTwo;

    DigitalChannel digiColorSensorP0, digiColorSensorP1;
    public static boolean preferHighOuttake = true;


    // Shoulder values to request from Trident

    public static int shoulderSpeed = 45;
    public static int SHOULDER_HOME_POSITION = 250;
    public static int SHOULDER_PREINTAKE_POSITION = 250;
    public static int SHOULDER_PREOUTTAKE_POSITION = 810;
    public static int SHOULDER_WALLTAKE_POSITION = -400;

    public static int SHOULDER_WALLTAKE_OFFSET = 100; // how much to raise the shoulder to lift off the wall
    public static int SHOULDER_LOWOUTTAKE_POSITION = 0;
    public static int SHOULDER_HIGHBAR_POSITION = 1710; //vertical latch 1710
    public int shoulderPositionMax = 850;


    public static int ELBOW_HIGHOUTTAKE_PREP_ANGLE = 0;
    public static int colorSensorGain = 12;
    public int slideTargetPosition = 0;
    public static int SLIDE_WALLTAKE_POSITION = 1300;
    public int SLIDE_HIGHBAR_POSITION = 1440; // 1200 vertical
    public int SAMPLER_SLIDE_HIBAR_POSITION = 200; // 720 to bump, 880 to clear - sampler needs to be just above the lowbar so it doesn't get in the way

    public int SLIDE_LATCH_OFFSET = 680; //500 for vertical version,  how much further to extend slide to latch

    public double ELBOW_WALLTAKE_ANGLE = 68;
    public double ELBOW_HIGHBAR_ANGLE = 15;

    public double ELBOW_FLOORTAKE_ANGLE = 140; // todo tune

    double ElbowBowHigh = ELBOW_WALLTAKE_ANGLE + 25;  // vertical
    double ElbowBowLow = ELBOW_FLOORTAKE_ANGLE; // out and down

    int SlideBowOut = 1500;


    public SpeciMiner(HardwareMap hardwareMap, Robot robot, Trident trident) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        this.trident = trident; // to request services from Trident - mainly setting the shoulder angle

        SLIDE_HIGHOUTTAKE_POSITION = 500;

        //defaults specific to sampler
        ELBOW_START_ANGLE = 0;
        ELBOW_HOME_POSITION = 2050;
        ELBOW_PWM_PER_DEGREE = -5.672222222222222;
        ELBOW_JOINT_SPEED = 120;
        ELBOW_MIN_ANGLE = -15;
        ELBOW_MAX_ANGLE = 220;
        ELBOW_ADJUST_ANGLE = 5;
        ELBOW_PREINTAKE_ANGLE = 5;
        ELBOW_LOWOUTTAKE_ANGLE = 102;
        ELBOW_HIGHOUTTAKE_PREP_ANGLE = 155;
        ELBOW_HIGHOUTTAKE_ANGLE = ELBOW_WALLTAKE_ANGLE;
        elbow = new Joint(hardwareMap, "specElbow", false, ELBOW_HOME_POSITION, ELBOW_PWM_PER_DEGREE, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE, ELBOW_START_ANGLE, ELBOW_JOINT_SPEED);
        DcMotorEx bruh = this.hardwareMap.get(DcMotorEx.class, "specSlide");
        slide = new DcMotorExResetable(bruh);
        slide.setMotorEnable();
        slide.setPower(1);
        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setVelocity(SLIDE_SPEED);
        //colorSensor = this.hardwareMap.get(NormalizedColorSensor.class, "specSensor");
        CRSOne = this.hardwareMap.get(CRServo.class, "specBeater1");
        CRSTwo = this.hardwareMap.get(CRServo.class, "specBeater2");
        digiColorSensorP0 = this.hardwareMap.get(DigitalChannel.class, "specP0"); //brushlabs color sensor digital ports P0 and P1
        digiColorSensorP1 = this.hardwareMap.get(DigitalChannel.class, "specP1");


        articulation = Articulation.MANUAL;
    }

    public static int groundIntakeIndex;
    public long groundIntakeTimer;

    @Override
    public boolean intake() {
        switch (groundIntakeIndex) {
            case 0:
                elbow.setTargetAngle(ELBOW_PREINTAKE_ANGLE);
                trident.setShoulderTarget(this, SHOULDER_PREINTAKE_POSITION);
                slideTargetPosition = SLIDE_PREINTAKE_POSITION;
                groundIntakeIndex++;
                break;
            case 1:
                if (withinError(trident.getShoulderCurrentPosition(), SHOULDER_PREINTAKE_POSITION, 10) && withinError(slide.getCurrentPosition(), SLIDE_PREINTAKE_POSITION, 10)) {
                    colorSensorEnabled = true;
                    groundIntakeIndex++;
                }
                break;
            case 2:
                if (stopOnSample()) {
                    trident.setShoulderTarget(this, SHOULDER_PREINTAKE_POSITION);
                    groundIntakeIndex = 0;
                    return true;
                }
                break;
        }
        return false;
    }

    public static int outtakeIndex;
    public long outtakeTimer;

    @Override
    boolean outtake() {
        switch (outtakeIndex) {
            case 0:
                prelatchHigh();
                outtakeIndex++;
                break;
            case 1: //needs a manual advance

                break;
            case 2:
                return latch();
        }
        return false;
    }

    public boolean prelatchHigh() { // set angle and extension for high bar, exits immediately
        elbow.setTargetAngle(ELBOW_HIGHBAR_ANGLE);
        trident.setShoulderTarget(this, SHOULDER_HIGHBAR_POSITION, false);
        slideTargetPosition = SLIDE_HIGHBAR_POSITION;
        robot.trident.sampler.setElbowAngle(Sampler.SWEEP_ELBOW_ANGLE);
        return true;
    }

    public boolean prelatchHighSlide() { // set just the slide to pre-latch position
        slideTargetPosition = SLIDE_HIGHBAR_POSITION;
        return true;
    }

    public boolean latch() { // set speciminer to latching extent - assumes position should already achieved
        slideTargetPosition = SLIDE_HIGHBAR_POSITION + SLIDE_LATCH_OFFSET;
        return withinError(slide.getCurrentPosition(), SLIDE_HIGHBAR_POSITION + SLIDE_LATCH_OFFSET, 10);
    }

    long tuckTimer = 0;
    int tuckIndex = 0;

    @Override
    boolean tuck() {
        switch (tuckIndex) {
            case 0:
                tuckTimer = futureTime(.7);
                elbow.setTargetAngle(ELBOW_START_ANGLE);
                servoPower = 0;
                tuckIndex++;
                break;
            case 1:
                if (isPast(tuckTimer)) {
                    slideTargetPosition = 20;
                    tuckIndex++;
                }
                break;
            case 2:
                if (slide.getCurrentPosition() < 150) {
                    return true;
                }
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
                servoPower = 0; //stop beaters
                bowIndex++;
                break;
            case 1: // bow the elbow
                if (isPast(bowTimer)) {
                    servoPower = .8;
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

    public void wallTakePresets() {
        trident.sampler.tuck();
        trident.setShoulderTarget(this, SHOULDER_WALLTAKE_POSITION);
        slideTargetPosition = SLIDE_WALLTAKE_POSITION;
        elbow.setTargetAngle(ELBOW_WALLTAKE_ANGLE);
        servoPower = 1;
    }

    int wallTakeIndex = 0;
    long wallTakeTimer = 0;

    // prepare the arm and speciminer for grabbing from wall
    // no driving
    // returns true if it grabs an alliance specimen and lifts it off the wall
    public boolean wallTake(boolean auto) {
        switch (wallTakeIndex) {
            case 0:
                wallTakePresets();
                if (auto)
                    robot.driveTrain.drive(.15, 0, 0); //drive forward
                wallTakeIndex++;

            case 1: // assume chassis is driving forward under driver or auton control
                if (grab()) {
                    robot.driveTrain.drive(0, 0, 0);
                    wallTakeIndex++;
                }
                break;
            case 2: //raise the captured specimen off the wall
                trident.setShoulderTarget(this, SHOULDER_WALLTAKE_POSITION + SHOULDER_WALLTAKE_OFFSET);
                if (withinError(trident.getShoulderCurrentPosition(), SHOULDER_WALLTAKE_POSITION + SHOULDER_WALLTAKE_OFFSET, 10)) {
                    wallTakeIndex = 0;
                    return true;
                }
                break;
        }
        return false;
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
        return true;
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
//        if (hue < 45 && hue > 35) {
//            currentSample = Sample.NEUTRAL;
//            return "NEUTRAL";
//        } else if (hue < 15 || hue > 350) {
//            currentSample = Sample.RED;
//            return "RED";
//        } else if (hue < 225 && hue > 200) {
//            currentSample = Sample.BLUE;
//            return "BLUE";
//        } else {
//            currentSample = Sample.NO_SAMPLE;
//            return "NO SAMPLE";
//        }
    }

    public boolean sampleDetected() {
        updateColorSensor();
        return targetSamples.contains(currentSample);
    }

    public void setIntaking() {
        servoPower = 1;
    }

    public void setEjecting() {
        servoPower = -1;
    }

    public void setResting() {
        servoPower = 0;
    }


    public boolean grab() {  //grab target samples, reject opposing alliance samples
        setIntaking();
        updateColorSensor();
        if (currentSample == Sample.NO_SAMPLE) return false;
        else if (targetSamples.contains(currentSample)) {
            setResting();
            return true;
        } else eject();
        return false; //todo this really needs to return an enum, because false just means
        // we don't have the sample we want, but we could have just ejected an opposing sample
        // which means we are likely to pick it up again - probably need to signal the elbow
        // to raise or slide to adjust temporarily to a different position
    }

    public boolean eject() {
        setEjecting();
        updateColorSensor();
        if (currentSample == Sample.NO_SAMPLE) {
            setResting();
            return true;
        }
        return false;
    }


    @Override
    public boolean stopOnSample() {
        servoPower = 1.0;
        if (sampleDetected()) {
            servoPower = 0;
            return true;
        }
        return false;
    }

    @Override
    public void adjustElbow(double adjustAngle) {
        elbow.setTargetAngle(elbow.getCurrentAngle() + adjustAngle);
    }

    @Override
    public void update(Canvas fieldOverlay) {
        updateColorSensor(); // can do this without penalty when calling digital brushlabs color sensor
        CRSOne.setPower(-servoPower);
        CRSTwo.setPower(servoPower);
        if (slideTargetPosition > slideMaxPosition)
            slideTargetPosition = slideMaxPosition;
        slide.setTargetPosition(slideTargetPosition);

        //compute the current articulation/behavior
        articulate();
        //allow real-time flipper speed changes
        elbow.setSpeed(ELBOW_JOINT_SPEED);
        elbow.update();
//        trident.setShoulderTarget(this, shoulderTargetPosition);
    }

    @Override
    public void stop() {
        servoPower = 0;
        CRSOne.setPower(-servoPower);
        CRSTwo.setPower(servoPower);
        slide.setTargetPosition(slide.getCurrentPosition());
    }

    @Override
    public void resetStates() {
        tuckIndex = 0;
        groundIntakeIndex = 0;
        wallTakeIndex = 0;
        outtakeIndex = 0;
        calibrateIndex = 0;
    }

    @Override
    public boolean calibrate() {
        return true; // stub - implement if needed
    }

    public void incrementOuttake() {
        outtakeIndex++;
    }

    public enum Articulation {
        MANUAL, //does nothing - used for transition tracking
        CALIBRATE,
        TUCK, // safely tuck the arm out of the way
        FLOOR_PREP, GROUNDTAKE, WALLTAKE, OUTTAKE

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
            case GROUNDTAKE: //get samples from the floor
                if (intake()) {
                    articulation = Articulation.MANUAL;
                }
                break;

            case OUTTAKE:
                if (outtake()) articulation = Articulation.MANUAL;
                break;

            case WALLTAKE: // get specimens from the wall under driver control
                if (wallTake(false)) articulation = Articulation.MANUAL;
                break;

            default:
                throw new RuntimeException("how the ^%*( did you get here?");
        }
        prevArticulation = articulation;
        return articulation;
    }

    public void sample(List<Sample> samples) {
        articulation = Articulation.GROUNDTAKE;
        targetSamples = samples;
    }

    public void sample(Constants.Alliance alliance) {
        List<Sample> samples = new ArrayList<>();
        if (alliance.isRed()) samples.add(Sample.RED);
        else samples.add(Sample.BLUE);
        samples.add(Sample.NEUTRAL);
        articulation = Articulation.GROUNDTAKE;
        targetSamples = samples;
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("articulation", articulation.name());
        telemetryMap.put("preferHighOuttake", preferHighOuttake);
        telemetryMap.put("intake index", groundIntakeIndex);
        telemetryMap.put("outtake index", outtakeIndex);
        telemetryMap.put("slide target : real", slideTargetPosition + " : " + slide.getCurrentPosition());
        telemetryMap.put("slide amps", slide.getCurrent(CurrentUnit.AMPS));
        telemetryMap.put("current sample", currentSample.name());
        //telemetryMap.put("colorsensor hsv", "" + HSVasString()); // cached - changes when HSV is read
        //telemetryMap.put("colorsensor rgb", colorLastRGBA.red + " " + colorLastRGBA.green + " " + colorLastRGBA.blue);

        telemetryMap.put("beater speed", servoPower);
        telemetryMap.put("elbow angle target : real", elbow.getTargetAngle() + " : " + elbow.getCurrentAngle() + " " + elbow.getPosition());

        telemetryMap.put("calibrate stage", calibrateIndex);

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "Speciminer";
    }
}
