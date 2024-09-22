package org.firstinspires.ftc.teamcode.robots.deepthought.subsystem;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;
import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.gameState;
import static org.firstinspires.ftc.teamcode.util.utilMethods.withinError;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.deepthought.util.Joint;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "AA_CS_OUTTAKE")
public class Outtake implements Subsystem {
    HardwareMap hardwareMap;
    Robot robot;

    public DcMotorEx slide = null;

    public Joint elbow;
    public Joint wrist;

    public static double slideTicksPerInch = 130.593132; // determined experimentally using a average distance and ticks

    //Kinematics values in inches
//    public static double armBase = 15.75;
//    public static double armHeight = 10.625;
//    public static double armTheta;
//    public static double elevatorBottomBone = 4.505;
//    public static double elevatorTopBone = 4.505;
//    public static double armLengthToElevator = 13;
//    public static double elbowLength = 6;
//    public static double wristLength = 9.5;
//    public static double armLength = elbowLength+ wristLength;

    public static double elbowTargetAngle, wristTargetAngle;

    public static int flipperPosition = 1888;

    public void setSlideTargetPosition(int slideTargetPosition) {
        this.slideTargetPosition = slideTargetPosition;
    }

    int slideTargetPosition = 0;
    public static int slidePositionMax = 1600;
    public static int slidePositionMin = 0;
    public static int slidePositionPreDock = 500;
    public static int slidePositionDocked = 0;
    public static int slidePositionScore = 400;


    public static int slideSpeed = 15;
    public static int UNTUCK_SLIDE_POSITION = 500;
    public static double SLIDE_SPEED = 1500;
    public static boolean TEMP_FLIPPER_TUNE = false;
    public static double IK_ADJUST_INCHES = .2;

    //ELBOW JOINT VARIABLES
    public static int ELBOW_HOME_POSITION = 2050;
    public static double ELBOW_PWM_PER_DEGREE = -5.672222222222222;
    //IN DEGREES PER SECOND
    public static double ELBOW_START_ANGLE = 0;

    public static double ELBOW_JOINT_SPEED = 60;

    public static double ELBOW_MIN_ANGLE = -15;
    public static double ELBOW_SAFE_ANGLE = 15;
    public static double ELBOW_SCORE_ANGLE = 120;
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
            default:
                break;
        }
        prevArticulation = articulation;
        return articulation;
    }

    public void cleanArticulations() {
//        backdropPrepStage = 0;
//        travelStage = 0;
//        travelStageBack = 0;
//        ingestPositionIndex = 0;

    }


    public enum Articulation {
        MANUAL, //does nothing - used for transition tracking
        TRAVEL, //does nothing - used for transition tracking
//        START
    }

    //LIVE STATES
    public Articulation articulation;

    public void setTargetAngle(double elbowAngle, double wristAngle) {
        elbow.setTargetAngle(elbowAngle);
        wrist.setTargetAngle(wristAngle);
    }

    public Outtake(HardwareMap hardwareMap, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        elbow = new Joint(hardwareMap, "elbow", false, ELBOW_HOME_POSITION, ELBOW_PWM_PER_DEGREE, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE, ELBOW_START_ANGLE, ELBOW_JOINT_SPEED);
        wrist = new Joint(hardwareMap, "wrist", false, WRIST_HOME_POSITION, WRIST_PWM_PER_DEGREE, WRIST_MIN_ANGLE, WRIST_MAX_ANGLE, WRIST_START_ANGLE, WRIST_JOINT_SPEED);
        slide = this.hardwareMap.get(DcMotorEx.class, "slide");
        wrist.setTargetAngle(WRIST_INIT_ANGLE);
        slide.setMotorEnable();
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setVelocity(SLIDE_SPEED);
        articulation = Articulation.MANUAL;
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
        //actually instruct actuators to go to desired targets
        elbow.update();
        wrist.update();
        slide.setTargetPosition(slideTargetPosition);
    }

    @Override
    public void stop() {
        slide.setMotorDisable();
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("articulation", articulation.name());
        telemetryMap.put("slide target position", slideTargetPosition);
        telemetryMap.put("slide actual position", Robot.sensors.outtakeSlideTicks);
        telemetryMap.put("slide actual position (inches)", Robot.sensors.outtakeSlideTicks/ slideTicksPerInch);
        telemetryMap.put("elbow ticks", flipperPosition);
        telemetryMap.put("elbow angle", elbow.getCurrentAngle());
        telemetryMap.put("elbow target angle", elbow.getTargetAngle());
        telemetryMap.put("elbow joint speed", ELBOW_JOINT_SPEED);
        telemetryMap.put("wrist angle", wrist.getCurrentAngle());
        telemetryMap.put("wrist target angle", wrist.getTargetAngle());
        telemetryMap.put("wrist joint speed", WRIST_JOINT_SPEED);
        telemetryMap.put("wrist target angle", wristTargetAngle);
        telemetryMap.put("elbow target angle", elbowTargetAngle);
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "OUTTAKE";
    }
}
