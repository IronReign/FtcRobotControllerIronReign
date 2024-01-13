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

    private DcMotorEx shoulder = null;
    private Servo gripperInner, gripperOuter, triggerDrone;
    
    public Joint wristLeft, wristRight; //the wrist is a diffy mechanism - blend of pitch and roll

    public static int flipperPosition = 1888;


    public void setShoulderTargetPosition(int shoulderTargetPosition) {
        if (shoulderTargetPosition < shoulderPositionMin) {
            shoulderTargetPosition = shoulderPositionMin;
        }
        if (shoulderTargetPosition > shoulderPositionMax) {
            shoulderTargetPosition = shoulderPositionMax;
        }
        this.shoulderTargetPosition = shoulderTargetPosition;
    }

    int shoulderTargetPosition = 0;
    public static int shoulderPositionMax = 100; //todo find real shoulder Max
    public static int shoulderPositionMin = 0;
    public static int shoulderPositionPreDock = 0;
    public static int slidePositionDocked = 0;

    public static int GripInnerOpen = 1234;
    public static int GripInnerClosed = 900;
    public static int GripOuterOpen = 900;
    public static int GripOuterClosed = 1400;


    int shoulderSpeed = 20;

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
        wristRight = new Joint(hardwareMap, "wristRight", false, WRIST_HOME_POSITION, WRIST_PWM_PER_DEGREE, WRIST_MIN_ANGLE, WRIST_MAX_ANGLE, WRIST_START_ANGLE, WRIST_JOINT_SPEED);
        triggerDrone = hardwareMap.get(Servo.class, "triggerDrone");

        gripperInner = hardwareMap.get(Servo.class, "gripInner");
        gripperOuter = hardwareMap.get(Servo.class, "gripOuter");

        shoulder = this.hardwareMap.get(DcMotorEx.class, "motorShoulder");
        shoulder.setMotorEnable();
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setTargetPosition(0);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(1);
//        pixelFlipper = this.hardwareMap.get(Servo.class, "pixelFlipper");

        behavior = Behavior.MANUAL;
    }

    public static int ingestPositionIndex = 0;
    public long ingestPositionTimer = 0;
    public boolean ingestFromTravel() { //should only call this if Travel was previously set
        switch (ingestPositionIndex) {
            case 0: //position outtake to position scoopagon just clear of the intake belt
                setShoulderTargetPosition(shoulderPositionPreDock);
                ingestPositionIndex ++;
                break;
            case 1: //lower the outtake to intake position
                if(between( shoulder.getCurrentPosition(), shoulderPositionPreDock +10, shoulderPositionPreDock -10)) {
                    setTargetAngle(WRIST_DOCK_ANGLE);
                    ingestPositionTimer = futureTime(.75);
                    ingestPositionIndex++;
                }
                break;
            case 2: //give enough time to pull down flipper, then slide to intake dock
                if (isPast(ingestPositionTimer))
                {
                    setShoulderTargetPosition(slidePositionDocked);
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
                setShoulderTargetPosition(shoulderPositionPreDock);
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
                    setShoulderTargetPosition(slidePositionDocked);
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
                setShoulderTargetPosition(0);
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


    public void adjustShoulder(double gamepadSpeed) {
        setShoulderTargetPosition(shoulder.getCurrentPosition()+(int)(gamepadSpeed * shoulderSpeed));
    }


public void wristTune(){
    if(TEMP_WRIST_TUNE) {
        wristLeft.setTargetAngle(wristLeft.getCurrentAngle() + 1);
        wristRight.setTargetAngle(wristRight.getCurrentAngle() + 1);
    }
    else {
        wristLeft.setTargetAngle(wristLeft.getCurrentAngle() - 1);
        wristRight.setTargetAngle(wristRight.getCurrentAngle() - 1);
    }
}

public void GripInnerToggle(){
        if (gripperInner.getPosition() == GripInnerOpen) gripperInner.setPosition(GripInnerClosed);
        else //to open inner, outer has to open as well
            {
                gripperOuter.setPosition(GripOuterOpen);
                gripperInner.setPosition(GripInnerOpen);
            }
    }
    public void GripOuterToggle(){
        if (gripperOuter.getPosition() == GripOuterClosed) gripperOuter.setPosition(GripOuterOpen);
        else //to close outer, inner has to close as well
        {
            gripperInner.setPosition(GripInnerClosed);
            gripperOuter.setPosition(GripOuterClosed);

        }
    }

    public void GripBoth(){
        gripperInner.setPosition(GripInnerClosed);
        gripperOuter.setPosition(GripOuterClosed);
    }

    public void GripNeither(){
        gripperInner.setPosition(GripInnerOpen);
        gripperOuter.setPosition(GripOuterOpen);
    }


    public int getShoulderTargetPosition() {
        return shoulderTargetPosition;
    }
    @Override
    public void update(Canvas fieldOverlay) {
        //compute the current articulation/behavior
        behave();

        //actually instruct actuators to go to desired targets
        wristLeft.update();
        wristRight.update();
        shoulder.setTargetPosition(shoulderTargetPosition);
    }

    @Override
    public void stop() {
        shoulder.setMotorDisable();
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("articulation", behavior.name());
        telemetryMap.put("shoulder target position", shoulderTargetPosition);
        telemetryMap.put("shoulder actual position", shoulder.getCurrentPosition());
        telemetryMap.put("wristLeft angle", wristLeft.getCurrentAngle());
        telemetryMap.put("wristLeft target angle", wristLeft.getTargetAngle());
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "ARM";
    }
}
