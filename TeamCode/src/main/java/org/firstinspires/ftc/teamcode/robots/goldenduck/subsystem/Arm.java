package org.firstinspires.ftc.teamcode.robots.goldenduck.subsystem;

import static org.firstinspires.ftc.teamcode.robots.csbot.util.Utils.servoDenormalizeExtended;
import static org.firstinspires.ftc.teamcode.robots.csbot.util.Utils.servoNormalizeExtended;
import static org.firstinspires.ftc.teamcode.robots.csbot.util.Utils.withinErrorPercent;
import static org.firstinspires.ftc.teamcode.util.utilMethods.between;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Joint;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "GD_ARM")
public class Arm implements Subsystem {

    HardwareMap hardwareMap;
    Robot robot;

    public boolean ArmCycleStarted = false;

    private DcMotorEx shoulderRight, shoulderLeft = null;
    private Servo gripperInner, gripperOuter, triggerDrone;

    public static int gripInnerToggleTest = 0, gripOuterToggleTest = 0;
    public Servo wristServoLeft, wristServoRight;

    //todo for when we convert to Joint based wrist control
    public Joint wristJointLeft, wristJointRight; //the wrist is a diffy mechanism - blend of pitch and roll

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

    public static int shoulderTargetPositionTest = 0;
    int shoulderTargetPositionTestPrev;
    public static int shoulderPositionMax = 1500; //beyond vertical with 125:1 ultraplanetary
    public static int shoulderPositionMin = -50;
    public static int shoulderPositionPreDock = 0;
    public static int slidePositionDocked = 0;

    public static int shoulderInitPos = 963;

    public static int GripInnerOpen = 1234;
    public static int GripInnerClosed = 900;
    public static int GripOuterOpen = 900;
    public static int GripOuterClosed = 1400;
    public enum GripperState{
        BothOpen,
        BothClosed,
        OuterOpen;
    }
    GripperState gripperState = GripperState.BothOpen; //initialized position
    public enum ArmHeight{
        Init,
        Intake,
        Travel,
        Backdrop,
        Drone;
    }

    ArmHeight armHeight = ArmHeight.Init;


    int shoulderSpeed = 20;

    public static boolean TEMP_WRIST_TUNE = false;

    //WRIST JOINT VARIABLES
//    public static int WRIST_HOME_POSITION = 1888;
//    public static double WRIST_PWM_PER_DEGREE = -7.35;
//    //IN DEGREES PER SECOND
//    public static double WRIST_START_ANGLE = 45;
//
//    public static double WRIST_JOINT_SPEED = 60;
//
//    public static double WRIST_MIN_ANGLE = 0;
//    public static double WRIST_MAX_ANGLE = 145;
//    public static double WRIST_PRE_SCORE_ANGLE = 120;
//    public static double WRIST_TRAVEL_ANGLE = 30;
//    public static int WRIST_ADJUST_ANGLE = 10;
//    public static double WRIST_DOCK_ANGLE = 0;
//direct servo control of wrist
    public static double WRIST_R_PICKUP = 900;
    public static double WRIST_L_PICKUP = 2105;
    public static double WRIST_R_TUCK = 2105;
    public static double WRIST_L_TUCK = 900;
    public static double WRIST_R_BACKDROP = 1100;
    public static double WRIST_L_BACKDROP = 1300;

    public static int DRONE_SET = 1556;
    public static int DRONE_LAUNCH = 900;
    public static int DRONE_SHOULDER = 1200;

    private Behavior prevBehavior;

    public Behavior behave(Behavior articulation) {
        this.behavior = articulation;
        return articulation;
    }
    public Behavior behave() {
        switch (behavior) {
            case CALIBRATE:
                if (calibrate())
                    behavior=Behavior.MANUAL;
                break;
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
                //wristJointLeft.setTargetAngle(WRIST_PRE_SCORE_ANGLE);
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
        CALIBRATE
    }

    public Behavior getBehavior() {
        return behavior;
    }

    //LIVE STATES
    private Behavior behavior;

    public void setTargetAngle(double angle) {
        wristJointLeft.setTargetAngle(angle);
    }

    public Arm(HardwareMap hardwareMap, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        //wristJointLeft = new Joint(hardwareMap, "wristLeft", false, WRIST_HOME_POSITION, WRIST_PWM_PER_DEGREE, WRIST_MIN_ANGLE, WRIST_MAX_ANGLE, WRIST_START_ANGLE, WRIST_JOINT_SPEED);
        //wristJointRight = new Joint(hardwareMap, "wristRight", false, WRIST_HOME_POSITION, WRIST_PWM_PER_DEGREE, WRIST_MIN_ANGLE, WRIST_MAX_ANGLE, WRIST_START_ANGLE, WRIST_JOINT_SPEED);
        wristServoLeft = hardwareMap.get(Servo.class, "wristLeft");
        wristServoRight = hardwareMap.get(Servo.class, "wristRight");
        triggerDrone = hardwareMap.get(Servo.class, "triggerDrone");

        gripperInner = hardwareMap.get(Servo.class, "gripInner");
        gripperOuter = hardwareMap.get(Servo.class, "gripOuter");

        //GripBoth();

        shoulderRight = this.hardwareMap.get(DcMotorEx.class, "motorShoulderRight");
        shoulderRight.setMotorEnable();
        //shoulderRight.setDirection(DcMotor.Direction.REVERSE);
        shoulderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderRight.setTargetPosition(0);
        shoulderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderRight.setPower(1);

        shoulderLeft = this.hardwareMap.get(DcMotorEx.class, "motorShoulderLeft");
        shoulderLeft.setMotorEnable();
        shoulderLeft.setDirection(DcMotor.Direction.REVERSE);
        shoulderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderLeft.setTargetPosition(0);
        shoulderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderLeft.setPower(1);
//        pixelFlipper = this.hardwareMap.get(Servo.class, "pixelFlipper");

        behavior = Behavior.MANUAL;
    }

    int calibrateStage = 0;
    long calibrateTimer = 0;
    double sampleAmps = 0, sampleSpeed = 0;
    boolean calibrate(){
        switch (calibrateStage){
            case 0: //start shoulder motor downward - motor should be pretty high up
                GripNeither(); //open wide
                //shoulderRight.setTargetPosition(-1000);
                shoulderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shoulderRight.setPower(-0.35); // start running the motor downward
                calibrateTimer=futureTime(.25);
                calibrateStage++;
                break;
            case 1: //sample the current once arm has started moving
                if (isPast(calibrateTimer)) {
                    sampleAmps = shoulderRight.getCurrent(CurrentUnit.AMPS);
                    sampleSpeed = shoulderRight.getVelocity();
                    calibrateStage++;
                }
                break;
            case 2: //when shoulder stalls, get shoulder ready for RUN_TO_POSITION
                if (shoulderRight.getVelocity()>-10) {
                    shoulderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    shoulderRight.setTargetPosition(0);
                    shoulderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shoulderRight.setPower(1);
                    setShoulderTargetPosition(shoulderInitPos);
                    calibrateStage=0;
                    return true;
                }
                break;
        }
        return false;
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
                if(between( shoulderRight.getCurrentPosition(), shoulderPositionPreDock +10, shoulderPositionPreDock -10)) {
                    //setTargetAngle(WRIST_DOCK_ANGLE);
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
                //setTargetAngle(WRIST_DOCK_ANGLE);
                setShoulderTargetPosition(shoulderPositionPreDock);
                travelTimer = futureTime(.5);
                travelStage++;
                break;
            case 1: //complete undock
                if (isPast(travelTimer)){
                    //setTargetAngle(WRIST_TRAVEL_ANGLE);
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
                //setTargetAngle(WRIST_TRAVEL_ANGLE);
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
        setShoulderTargetPosition(shoulderRight.getCurrentPosition()+(int)(gamepadSpeed * shoulderSpeed));
    }

    public void setDroneShoulderTargetPosition(){
        setShoulderTargetPosition(DRONE_SHOULDER);
    }


public void wristTune(){
    if(TEMP_WRIST_TUNE) {
        //wristJointLeft.setTargetAngle(wristJointLeft.getCurrentAngle() + 1);
        //wristRight.setTargetAngle(wristRight.getCurrentAngle() + 1);
    }
    else {
        //wristJointLeft.setTargetAngle(wristJointLeft.getCurrentAngle() - 1);
        //wristRight.setTargetAngle(wristRight.getCurrentAngle() - 1);
    }
}

public void WristTuck(){
        wristServoLeft.setPosition(servoNormalizeExtended(WRIST_L_TUCK));
        wristServoRight.setPosition(servoNormalizeExtended(WRIST_R_TUCK));
}

    public void WristPickup(){
        wristServoLeft.setPosition(servoNormalizeExtended(WRIST_L_PICKUP));
        wristServoRight.setPosition(servoNormalizeExtended(WRIST_R_PICKUP));
    }

    public void WristBackdrop(){
        wristServoLeft.setPosition(servoNormalizeExtended(WRIST_L_BACKDROP));
        wristServoRight.setPosition(servoNormalizeExtended(WRIST_R_BACKDROP));
    }

public void GripInnerToggle(){ //to open inner, outer has to open as well
        if (gripperState==GripperState.BothClosed || gripperState == GripperState.OuterOpen)
            gripperState = GripperState.BothOpen;
        else
            gripperState = GripperState.BothClosed;
    }
    public void GripOuterToggle(){
        if (gripperState==GripperState.OuterOpen || gripperState==GripperState.BothOpen)
            gripperState = GripperState.BothClosed;
        else
            gripperState=GripperState.OuterOpen;
    }

    public void GripInnerOnly(){
        gripperInner.setPosition(servoNormalizeExtended(GripInnerClosed));
        gripperOuter.setPosition(servoNormalizeExtended(GripOuterOpen));
    }
    public void GripBoth(){
        gripperInner.setPosition(servoNormalizeExtended(GripInnerClosed));
        gripperOuter.setPosition(servoNormalizeExtended(GripOuterClosed));
    }

    public void GripNeither(){
        gripperInner.setPosition(servoNormalizeExtended(GripInnerOpen));
        gripperOuter.setPosition(servoNormalizeExtended(GripOuterOpen));
    }


    public int getShoulderTargetPosition() {
        return shoulderTargetPosition;
    }
    @Override
    public void update(Canvas fieldOverlay) {
        //compute the current articulation/behavior
        behave();

        //actually instruct actuators to go to desired targets
        //wristJointLeft.update();
        //wristJointRight.update();

        //allow toggling grippers via dashboard
        if (gripInnerToggleTest%2==1) {
            GripInnerToggle();
            gripInnerToggleTest = 0;
        }
        if (gripOuterToggleTest%2==1) {
            GripOuterToggle();
            gripOuterToggleTest = 0;
        }

        //allow setting a test target position via dashboard
        if (shoulderTargetPositionTest!=shoulderTargetPositionTestPrev)
        {
            shoulderTargetPositionTestPrev=shoulderTargetPositionTest;
            setShoulderTargetPosition(shoulderTargetPositionTest);
        }

        //actually update actuators
        switch(gripperState) {
            case BothOpen:
                GripNeither();
                break;
            case BothClosed:
                GripBoth();
                break;
            case OuterOpen:
                GripInnerOnly();
                break;
        }

        if(ArmCycleStarted)
        {
            switch (armHeight){
                case Intake:
                    armHeight = ArmHeight.Travel;
                    break;
                case Init:
                    armHeight = ArmHeight.Travel;
                    break;
                case Travel:
                    if (true)// prevArmHeight==ArmHeight.Backdrop)
                    armHeight = ArmHeight.Intake;
                    else armHeight = ArmHeight.Backdrop;
                    break;
                case Backdrop:
                    armHeight = ArmHeight.Travel;
                    break;
                    



            }
        }
        //shoulderLeft.setTargetPosition(shoulderTargetPosition);
        shoulderRight.setTargetPosition(shoulderTargetPosition);
    }

    public void droneSet() {
        triggerDrone.setPosition(servoNormalizeExtended(DRONE_SET));
    }
    public void droneLaunch(){
        triggerDrone.setPosition(servoNormalizeExtended(DRONE_LAUNCH));
    }

    @Override
    public void stop() {

        shoulderRight.setMotorDisable();
        shoulderLeft.setMotorDisable();
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String,                                                                                                                                                           Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("articulation", behavior.name());
        telemetryMap.put("shoulder target position", shoulderTargetPosition);
        telemetryMap.put("shoulder left actual position", shoulderLeft.getCurrentPosition());
        telemetryMap.put("shoulder rightactual position", shoulderRight.getCurrentPosition());
        telemetryMap.put("shoulder left Amps", shoulderLeft.getCurrent(CurrentUnit.AMPS));
        telemetryMap.put("shoulder right Amps", shoulderRight.getCurrent(CurrentUnit.AMPS));
        telemetryMap.put("shoulder sample Amps", sampleAmps);
        telemetryMap.put("shoulder right Speed", shoulderRight.getVelocity());
        telemetryMap.put("shoulder sample Speed", sampleSpeed);
        //telemetryMap.put("wristLeft angle", wristJointLeft.getCurrentAngle());
        //telemetryMap.put("wristLeft target angle", wristJointLeft.getTargetAngle());
        telemetryMap.put("grip Outer position", gripperOuter.getPosition());
        telemetryMap.put("grip Outer PWM", servoDenormalizeExtended( gripperOuter.getPosition()));
        telemetryMap.put("grip Inner position", gripperInner.getPosition());
        telemetryMap.put("grip Inner PWM", servoDenormalizeExtended( gripperInner.getPosition()));
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "ARM";
    }
}
