package org.firstinspires.ftc.teamcode.robots.deepthought.subsystem;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.deepthought.util.Joint;

import java.util.*;


@Config(value = "00_ITD_TRIDENT")
public class Trident implements Subsystem {
    HardwareMap hardwareMap;
    Robot robot;

    public DcMotorEx slide = null;
    public DcMotorEx crane = null;
    public Joint elbow;
    public Joint wrist;
    public CRServoImplEx beater = null;
    public ColorSensor colorSensor = null;
    public boolean colorSensorEnabled = false;

    public enum CurrentSample {
        RED, BLUE, NEUTRAL, NO_SAMPLE
    }
    public CurrentSample currentSample = CurrentSample.NO_SAMPLE;
    List<CurrentSample> targetSamples = new ArrayList<>();

    public static double slideTicksPerInch = 130.593132; // determined experimentally using a average distance and ticks
    public static double elbowTargetAngle, wristTargetAngle;
    public static int flipperPosition = 1888;

    public void setSlideTargetPosition(int slideTargetPosition) {
        this.slideTargetPosition = slideTargetPosition;
    }

    int slideTargetPosition = 0;
    public static int slidePositionMax = 1600;
    public static int slidePositionMin = 0;


    public static int slideSpeed = 15;
    public static double SLIDE_SPEED = 1500;

    int craneTargetPosition = 0;
    public static int craneSpeed = 30;
    public void setCraneTargetPosition(int craneTargetPosition) {
        this.craneTargetPosition = craneTargetPosition;
    }


    //ELBOW JOINT VARIABLES
    public static int ELBOW_HOME_POSITION = 2050;
    public static double ELBOW_PWM_PER_DEGREE = -5.672222222222222;
    //IN DEGREES PER SECOND
    public static double ELBOW_START_ANGLE = 0;

    public static double ELBOW_JOINT_SPEED = 60;

    public static double ELBOW_MIN_ANGLE = -15;
    public static double ELBOW_MAX_ANGLE = 220;
    public static double ELBOW_PRE_SCORE_ANGLE = 120;
    public static double ELBOW_TRAVEL_ANGLE = 0;
    public static int ELBOW_ADJUST_ANGLE = 5;
    public static int WRIST_ADJUST_ANGLE = 5;
    public static int ELEVATOR_ADJUST_ANGLE = 5;
    public static double ELBOW_DOCK_ANGLE = -15;

    //WRIST JOINT VARIABLES TODO tune these value
    public static int WRIST_HOME_POSITION = 950;
    public static double WRIST_PWM_PER_DEGREE = 7.22222222222;
    //IN DEGREES PER SECOND
    public static double WRIST_START_ANGLE = 0;
    public static double WRIST_TRAVEL_ANGLE = WRIST_START_ANGLE;

    public static double WRIST_INIT_ANGLE = 60;
    public static double WRIST_SCORE_ANGLE = 180;

    public static double WRIST_JOINT_SPEED = 50;

    public static double WRIST_MIN_ANGLE = 0;
    public static double WRIST_MAX_ANGLE = 180;

    private boolean flipped = false;

    private Articulation prevArticulation;

    public Articulation articulate(Articulation articulation) {
        this.articulation = articulation;
        return articulation;
    }

    public Articulation articulate() {
        switch (articulation) {
            case MANUAL:
                break;
            case TRAVEL:
                break;
            //SHOULD ONLY BE ACCESSED BY SAMPLE()
            case SAMPLE:
                if(intake()) {
                    articulation = Articulation.MANUAL;
                }
                break;
            default:
                throw new RuntimeException("how the ^%*( did you get here?");
        }
        prevArticulation = articulation;
        return articulation;
    }

    public static int intakeIndex;
    public long intakeTimer;
    public boolean intake() {
        switch (intakeIndex) {
            case 0:
                //get to position
                intakeIndex++;
                break;
            case 1:
                beater.setPower(1);
                colorSensorEnabled = true;
                //open the "gate"
                if(targetSamples.contains(currentSample)) {
                    intakeTimer = futureTime(3);
                    intakeIndex++;
                }
                break;
            case 2:
                //close the "gate"
                beater.setPower(-1);
                if(isPast(intakeTimer)) {
                    beater.setPower(0);
                    intakeIndex++;
                }
                break;
            case 3:
                return true;
            case 4:
                break;
            case 5:
                return true;
        }
        return false;
    }

    public void sample(List<CurrentSample> samples) {
        articulation = Articulation.SAMPLE;
        this.targetSamples = samples;
    }


    public enum Articulation {
        MANUAL, //does nothing - used for transition tracking
        TRAVEL, //does nothing - used for transition tracking
        SAMPLE
    }

    //LIVE STATES
    public Articulation articulation;

    public void setTargetAngle(double elbowAngle, double wristAngle) {
        elbow.setTargetAngle(elbowAngle);
        wrist.setTargetAngle(wristAngle);
    }

    public Trident(HardwareMap hardwareMap, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        elbow = new Joint(hardwareMap, "elbow", false, ELBOW_HOME_POSITION, ELBOW_PWM_PER_DEGREE, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE, ELBOW_START_ANGLE, ELBOW_JOINT_SPEED);
        wrist = new Joint(hardwareMap, "wrist", false, WRIST_HOME_POSITION, WRIST_PWM_PER_DEGREE, WRIST_MIN_ANGLE, WRIST_MAX_ANGLE, WRIST_START_ANGLE, WRIST_JOINT_SPEED);
        slide = this.hardwareMap.get(DcMotorEx.class, "slide");
        crane = this.hardwareMap.get(DcMotorEx.class, "crane");
        colorSensor = this.hardwareMap.get(ColorSensor.class, "intakeSensor");
        beater = this.hardwareMap.get(CRServoImplEx.class, "beater");
        wrist.setTargetAngle(WRIST_INIT_ANGLE);
        slide.setMotorEnable();
        slide.setPower(1);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setVelocity(SLIDE_SPEED);

        crane.setMotorEnable();
        crane.setPower(1);
        crane.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        crane.setTargetPosition(0);
        crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        crane.setVelocity(1500);
        articulation = Articulation.MANUAL;
    }

    public int getCraneTargetPosition() {
        return craneTargetPosition;
    }

    public void moveSlide(int distance) {
        slideTargetPosition += distance * slideSpeed;
        if (slideTargetPosition < slidePositionMin) {
            slideTargetPosition = slidePositionMin;
        }
        if (slideTargetPosition > slidePositionMax) {
            slideTargetPosition = slidePositionMax;
        }
        slide.setTargetPosition(slideTargetPosition);
    }

    public void adjustElbow(double angle) {
        elbow.setTargetAngle(elbow.getCurrentAngle() + angle);
    }

    public void adjustWrist(double angle) {
        wrist.setTargetAngle(wrist.getCurrentAngle() + angle);
    }

    public int getSlideTargetPosition() {
        return slideTargetPosition;
    }

    @Override
    public void update(Canvas fieldOverlay) {
        //compute the current articulation/behavior
        articulate();
        //allow real-time flipper speed changes
        elbow.setSpeed(ELBOW_JOINT_SPEED);
        wrist.setSpeed(WRIST_JOINT_SPEED);

        if(colorSensorEnabled) {
            updateColorSensor();
        }

        //actually instruct actuators to go to desired targets
        elbow.update();
        wrist.update();
        slide.setTargetPosition(slideTargetPosition);
        crane.setTargetPosition(craneTargetPosition);
    }

    public String updateColorSensor() {
        if (colorSensor.blue() + colorSensor.red() + colorSensor.green() > 500) {
            if(colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()) {
                currentSample = CurrentSample.RED;
                return "Red";
            } else if(colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()) {
                currentSample = CurrentSample.BLUE;
                return "Blue";
            } else {
                currentSample = CurrentSample.NEUTRAL;
                return "Neutral";
            }
        } else {
            currentSample = CurrentSample.NO_SAMPLE;
            return "No Sample";
        }
    }

    @Override
    public void stop() {
        slide.setMotorDisable();
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("crane target position", craneTargetPosition);
        telemetryMap.put("crane position", crane.getCurrentPosition());
        telemetryMap.put("articulation", articulation.name());
        telemetryMap.put("slide target position", slideTargetPosition);
        telemetryMap.put("beater speed", beater.getPower());
        telemetryMap.put("slide actual position", Robot.sensors.outtakeSlideTicks);
        telemetryMap.put("elbow ticks", flipperPosition);
        telemetryMap.put("elbow angle", elbow.getCurrentAngle());
        telemetryMap.put("elbow joint speed", ELBOW_JOINT_SPEED);
        telemetryMap.put("wrist angle", wrist.getCurrentAngle());
        telemetryMap.put("wrist joint speed", WRIST_JOINT_SPEED);
        telemetryMap.put("wrist target angle", wristTargetAngle);
        telemetryMap.put("elbow target angle", elbowTargetAngle);
        telemetryMap.put("current sample", currentSample.name());
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "OUTTAKE";
    }
}
