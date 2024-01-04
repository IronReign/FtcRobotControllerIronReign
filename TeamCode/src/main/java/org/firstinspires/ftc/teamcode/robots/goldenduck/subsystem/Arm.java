package org.firstinspires.ftc.teamcode.robots.goldenduck.subsystem;

import static org.firstinspires.ftc.teamcode.util.utilMethods.between;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.csbot.util.Joint;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "GD_ARM")
public class Arm implements Subsystem {

    HardwareMap hardwareMap;
    Robot robot;

    private DcMotorEx arm = null;
    private Servo gripperInner, gripperOuter, triggerDrone;
    
    public Joint wristLeft, wristRight; //the wrist is a diffy mechanism - blend of pitch and roll

    public static int flipperPosition = 1888;


    public void setArmTargetPosition(int armTargetPosition) {
        this.armTargetPosition = armTargetPosition;
    }

    int armTargetPosition = 0;
    public static int armPositionMax = 100; //todo find real arm Max
    public static int armPositionMin = 0;
    public static int armPositionPreDock = 0;
    public static int slidePositionDocked = 0;


    int slideSpeed = 20;

    public static boolean TEMP_WRIST_TUNE = false;

    //WRIST JOINT VARIABLES
    public static int WRIST_HOME_POSITION = 1888;
    public static double WRIST_PWM_PER_DEGREE = -7.35;
    //IN DEGREES PER SECOND
    public static double WRIST_START_ANGLE = 45;

    public static double WRIST_JOINT_SPEED = 60;

    public static double WRIST_MIN_ANGLE = 0;
    public static double WRIST_MAX_ANGLE = 145;
    public static double WRIST_PRE_SCORE_ANGLE = 120;
    public static double WRIST_TRAVEL_ANGLE = 30;
    public static int WRIST_ADJUST_ANGLE = 10;
    public static double WRIST_DOCK_ANGLE = 0;

    private boolean flipped = false;

    private Behavior prevBehavior;

    public Behavior behave(Behavior articulation) {
        this.behavior = articulation;
        return articulation;
    }
    public Behavior behave() {
        switch (behavior) {
            case MANUAL:
                break;
            case BACKDROP:
                break;
            case TRAVEL:
                break;
            case TRAVEL_FROM_INGEST:
                if(travelFromIngest())
                    behavior = Behavior.TRAVEL;
                break;
            case INGEST_FROM_TRAVEL:
                if(ingestFromTravel()) {
                    behavior = Behavior.MANUAL;
                }
                break;
            case TRAVEL_FROM_BACKDROP:
                if(travelFromBackdrop()) {
                    behavior = Behavior.TRAVEL;
                }
                break;
            case BACKDROP_PREP:
                wristLeft.setTargetAngle(WRIST_PRE_SCORE_ANGLE);
                behavior = Behavior.BACKDROP;
                break;

            default:
                break;
        }
        prevBehavior = behavior;
        return behavior;
    }

    public enum Behavior {
        MANUAL, //does nothing - used for transition tracking
        TRAVEL, //does nothing - used for transition tracking
        TRAVEL_FROM_INGEST,
        INGEST_FROM_TRAVEL,
        TRAVEL_FROM_BACKDROP,
        BACKDROP_PREP,
        BACKDROP,
        FOLD
    }

    //LIVE STATES
    public Behavior behavior;

    public void setTargetAngle(double angle) {
        wristLeft.setTargetAngle(angle);
    }

    public Arm(HardwareMap hardwareMap, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        wristLeft = new Joint(hardwareMap, "wristLeft", false, WRIST_HOME_POSITION, WRIST_PWM_PER_DEGREE, WRIST_MIN_ANGLE, WRIST_MAX_ANGLE, WRIST_START_ANGLE, WRIST_JOINT_SPEED);
        wristLeft = new Joint(hardwareMap, "wristRight", false, WRIST_HOME_POSITION, WRIST_PWM_PER_DEGREE, WRIST_MIN_ANGLE, WRIST_MAX_ANGLE, WRIST_START_ANGLE, WRIST_JOINT_SPEED);
        triggerDrone = hardwareMap.get(Servo.class, "servoRailgun");

        gripperInner = hardwareMap.get(Servo.class, "gripperInner");
        gripperOuter = hardwareMap.get(Servo.class, "gripperOuter");

        arm = this.hardwareMap.get(DcMotorEx.class, "motorShoulder");
        arm.setMotorEnable();
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
//        pixelFlipper = this.hardwareMap.get(Servo.class, "pixelFlipper");

        behavior = Behavior.MANUAL;
    }

    public static int ingestPositionIndex = 0;
    public long ingestPositionTimer = 0;
    public boolean ingestFromTravel() { //should only call this if Travel was previously set
        switch (ingestPositionIndex) {
            case 0: //position outtake to position scoopagon just clear of the intake belt
                setArmTargetPosition(armPositionPreDock);
                ingestPositionIndex ++;
                break;
            case 1: //lower the outtake to intake position
                if(between( arm.getCurrentPosition(), armPositionPreDock +10, armPositionPreDock -10)) {
                    setTargetAngle(WRIST_DOCK_ANGLE);
                    ingestPositionTimer = futureTime(.75);
                    ingestPositionIndex++;
                }
                break;
            case 2: //give enough time to pull down flipper, then slide to intake dock
                if (isPast(ingestPositionTimer))
                {
                    setArmTargetPosition(slidePositionDocked);
                    ingestPositionIndex=0;
                    return true;
                }
                break;
        }
        return false;
    }

    long travelTimer = 0;
    int travelStage = 0;
    boolean travelFromIngest(){
        switch (travelStage){
            case 0: //begin undock
                setTargetAngle(WRIST_DOCK_ANGLE);
                setArmTargetPosition(armPositionPreDock);
                travelTimer = futureTime(.5);
                travelStage++;
                break;
            case 1: //complete undock
                if (isPast(travelTimer)){
                    setTargetAngle(WRIST_TRAVEL_ANGLE);
                    travelTimer = futureTime(.5);
                    travelStage++;
                }
                break;
            case 2: //tuck slide - todo this will get more complicated when outtake elevation is changeable
                if (isPast(travelTimer)){
                    setArmTargetPosition(slidePositionDocked);
                    robot.behave(Robot.Behavior.TRAVEL);
                    travelStage = 0;
                    return true;
                }
                break;
        }
        return false;
    }
    int travelStageBack = 0;
    boolean travelFromBackdrop(){
        switch (travelStageBack) { //robot should have already placed the intake into travel position
            case 0:
                setTargetAngle(WRIST_TRAVEL_ANGLE);
                setArmTargetPosition(0);
                travelTimer = futureTime(.5);
                travelStageBack++;
                break;
            case 1:
                if (isPast(travelTimer)) {
                    travelStageBack = 0;
                    return true;
                }
                break;
        }
        return false;
    }

    public void moveSlide(int distance) {
        armTargetPosition += distance * slideSpeed;
        if (armTargetPosition < armPositionMin) {
            armTargetPosition = armPositionMin;
        }
        if (armTargetPosition > armPositionMax) {
            armTargetPosition = armPositionMax;
        }
        arm.setTargetPosition(armTargetPosition);
    }

    public void adjustFlipper(int angle) {
        wristLeft.setTargetAngle(wristLeft.getCurrentAngle() +  angle);
    }


public void flipperTest(){
    if(TEMP_WRIST_TUNE) {
        wristLeft.setTargetAngle(wristLeft.getCurrentAngle() + 1);
    }
    else
        wristLeft.setTargetAngle(wristLeft.getCurrentAngle() - 1);
}

    public int getArmTargetPosition() {
        return armTargetPosition;
    }
    @Override
    public void update(Canvas fieldOverlay) {
        //compute the current articulation/behavior
        behave();

        //actually instruct actuators to go to desired targets
        wristLeft.update();
        arm.setTargetPosition(armTargetPosition);
    }

    @Override
    public void stop() {
        arm.setMotorDisable();
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("articulation", behavior.name());
        telemetryMap.put("slide target position", armTargetPosition);
        telemetryMap.put("slide actual position", arm.getCurrentPosition());
//        telemetryMap.put("flipper location", Utils.servoDenormalize(pixelFlipper.getPosition()));
        telemetryMap.put("flipper ticks", flipperPosition);
        telemetryMap.put("flipper angle", wristLeft.getCurrentAngle());
        telemetryMap.put("flipper target angle", wristLeft.getTargetAngle());
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "OUTTAKE";
    }
}
