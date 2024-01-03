package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

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

@Config(value = "CS_OUTTAKE")
public class Outtake implements Subsystem {

    HardwareMap hardwareMap;
    Robot robot;

    private DcMotorEx slide = null;
    private Servo pixelFlipper = null;
    public Joint flipper;

    public static int flipperPosition = 1888;


    public void setSlideTargetPosition(int slideTargetPosition) {
        this.slideTargetPosition = slideTargetPosition;
    }

    int slideTargetPosition = 0;
    public static int slidePositionMax = 1600;
    public static int slidePositionMin = 0;
    public static int slidePositionPreDock = 500;
    public static int slidePositionDocked = 0;


    int slideSpeed = 20;
    public static int UNTUCK_SLIDE_POSITION = 500;
    public static boolean TEMP_FLIPPER_TUNE = false;

    //FLIPPER JOINT VARIABLES
    public static int FLIPPER_HOME_POSITION = 1888;
    public static double FLIPPER_PWM_PER_DEGREE = -7.35;
    //IN DEGREES PER SECOND
    public static double FLIPPER_START_ANGLE = 45;

    public static double FLIPPER_JOINT_SPEED = 60;

    public static double FLIPPER_MIN_ANGLE = 0;
    public static double FLIPPER_MAX_ANGLE = 145;
    public static double FLIPPER_PRE_SCORE_ANGLE = 120;
    public static double FLIPPER_TRAVEL_ANGLE = 30;
    public static int FLIPPER_ADJUST_ANGLE = 10;
    public static double FLIPPER_DOCK_ANGLE = 0;

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
            case BACKDROP:
                break;
            case TRAVEL:
                break;
            case TRAVEL_FROM_INGEST:
                if(travelFromIngest())
                    articulation=Articulation.TRAVEL;
                break;
            case INGEST_FROM_TRAVEL:
                if(ingestFromTravel()) {
                    articulation = Articulation.MANUAL;
                }
                break;
            case TRAVEL_FROM_BACKDROP:
                if(travelFromBackdrop()) {
                    articulation = Articulation.TRAVEL;
                }
                break;
            case BACKDROP_PREP:
                flipper.setTargetAngle(FLIPPER_PRE_SCORE_ANGLE);
                articulation = Articulation.BACKDROP;
                break;

            default:
                break;
        }
        prevArticulation = articulation;
        return articulation;
    }

    public enum Articulation {
        MANUAL, //does nothing - used for transition tracking
        TRAVEL, //does nothing - used for transition tracking
        TRAVEL_FROM_INGEST,
        INGEST_FROM_TRAVEL,
        TRAVEL_FROM_BACKDROP,
        BACKDROP_PREP,
        BACKDROP,
        FOLD
    }

    public enum FlipperLocation {
        TUCK,
        CLEAR,
        SCORE
    }

    FlipperLocation flipperLocation;


    //LIVE STATES
    public Articulation articulation;

    public void setTargetAngle(double angle) {
        flipper.setTargetAngle(angle);
    }

    public Outtake(HardwareMap hardwareMap, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        flipper = new Joint(hardwareMap, "pixelFlipper", false, FLIPPER_HOME_POSITION, FLIPPER_PWM_PER_DEGREE, FLIPPER_MIN_ANGLE, FLIPPER_MAX_ANGLE, FLIPPER_START_ANGLE, FLIPPER_JOINT_SPEED);
        slide = this.hardwareMap.get(DcMotorEx.class, "slide");
        slide.setMotorEnable();
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);
//        pixelFlipper = this.hardwareMap.get(Servo.class, "pixelFlipper");

        articulation = Articulation.MANUAL;
    }

    public static int ingestPositionIndex = 0;
    public long ingestPositionTimer = 0;
    public boolean ingestFromTravel() { //should only call this if Travel was previously set
        switch (ingestPositionIndex) {
            case 0: //position outtake to position scoopagon just clear of the intake belt
                setSlideTargetPosition(slidePositionPreDock);
                ingestPositionIndex ++;
                break;
            case 1: //lower the outtake to intake position
                if(between( slide.getCurrentPosition(), slidePositionPreDock +10, slidePositionPreDock -10)) {
                    setTargetAngle(FLIPPER_DOCK_ANGLE);
                    ingestPositionTimer = futureTime(.75);
                    ingestPositionIndex++;
                }
                break;
            case 2: //give enough time to pull down flipper, then slide to intake dock
                if (isPast(ingestPositionTimer))
                {
                    setSlideTargetPosition(slidePositionDocked);
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
                setTargetAngle(FLIPPER_DOCK_ANGLE);
                setSlideTargetPosition(slidePositionPreDock);
                travelTimer = futureTime(.5);
                travelStage++;
                break;
            case 1: //complete undock
                if (isPast(travelTimer)){
                    setTargetAngle(FLIPPER_TRAVEL_ANGLE);
                    travelTimer = futureTime(.5);
                    travelStage++;
                }
                break;
            case 2: //tuck slide - todo this will get more complicated when outtake elevation is changeable
                if (isPast(travelTimer)){
                    setSlideTargetPosition(slidePositionDocked);
                    robot.articulate(Robot.Articulation.TRAVEL);
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
                setTargetAngle(FLIPPER_TRAVEL_ANGLE);
                setSlideTargetPosition(0);
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
        slideTargetPosition += distance * slideSpeed;
        if (slideTargetPosition < slidePositionMin) {
            slideTargetPosition = slidePositionMin;
        }
        if (slideTargetPosition > slidePositionMax) {
            slideTargetPosition = slidePositionMax;
        }
        slide.setTargetPosition(slideTargetPosition);
    }

    public void adjustFlipper(int angle) {
        flipper.setTargetAngle(flipper.getCurrentAngle() +  angle);
    }


public void flipperTest(){
    if(TEMP_FLIPPER_TUNE) {
        flipper.setTargetAngle(flipper.getCurrentAngle() + 1);
    }
    else
        flipper.setTargetAngle(flipper.getCurrentAngle() - 1);
}

    public int getSlideTargetPosition() {
        return slideTargetPosition;
    }
    @Override
    public void update(Canvas fieldOverlay) {
        //compute the current articulation/behavior
        articulate();

        //actually instruct actuators to go to desired targets
        flipper.update();
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
        telemetryMap.put("slide actual position", slide.getCurrentPosition());
//        telemetryMap.put("flipper location", Utils.servoDenormalize(pixelFlipper.getPosition()));
        telemetryMap.put("flipper ticks", flipperPosition);
        telemetryMap.put("flipper angle", flipper.getCurrentAngle());
        telemetryMap.put("flipper target angle", flipper.getTargetAngle());
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "OUTTAKE";
    }
}
