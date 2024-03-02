package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import static org.firstinspires.ftc.teamcode.util.utilMethods.between;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.gameState;
import static org.firstinspires.ftc.teamcode.util.utilMethods.withinError;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.csbot.util.Joint;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Utils;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "AA_CS_OUTTAKE")
public class Outtake implements Subsystem {

    HardwareMap hardwareMap;
    Robot robot;

    public DcMotorEx slide = null;
    private Servo pixelFlipper = null;
    public Joint elbow;
    public Joint wrist;
    public Joint elevator;

    public static double ticksPerInch = 130.593132; // determined experimentally using a average distance and ticks

    //Kinematics values in inches
    public static double armBase = 15.75;
    public static double armHeight = 10.625;
    public static double armTheta;
    public static double elevatorBottomBone = 4.505;
    public static double elevatorTopBone = 4.505;
    public static double armLengthToElevator = 13;
    public static double elbowLength = 6;
    public static double wristLength = 9.5;
    public static double armLength = elbowLength+ wristLength;

    public static double armX, armZ;
    public static double elbowTargetAngle, wristTargetAngle;
    private double theta, top, bottom, frac;

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


    int slideSpeed = 15;
    public static int UNTUCK_SLIDE_POSITION = 500;
    public static boolean TEMP_FLIPPER_TUNE = false;

    //ELBOW JOINT VARIABLES
    public static int ELBOW_HOME_POSITION = 1888;
    public static double ELBOW_PWM_PER_DEGREE = -7.35;
    //IN DEGREES PER SECOND
    public static double ELBOW_START_ANGLE = 70;

    public static double ELBOW_JOINT_SPEED = 75;

    public static double ELBOW_MIN_ANGLE = -15;
    public static double ELBOW_SAFE_ANGLE = 15;
    public static double ELBOW_SCORE_ANGLE = 120;
    public static double ELBOW_MAX_ANGLE = 145;
    public static double ELBOW_PRE_SCORE_ANGLE = 120;
    public static double ELBOW_TRAVEL_ANGLE = 20;
    public static int ELBOW_ADJUST_ANGLE = 5;
    public static double ELBOW_DOCK_ANGLE = -15;

    //ELEVATOR JOINT VARIABLES TODO tune these value
    public static int ELEVATOR_HOME_POSITION = 1027;
    public static double ELEVATOR_PWM_PER_DEGREE = 7.35;
    //IN DEGREES PER SECOND
    public static double ELEVATOR_START_ANGLE = 0;

    public static double ELEVATOR_JOINT_SPEED = 75;

    public static double ELEVATOR_MIN_ANGLE = 0;
    public static double ELEVATOR_MAX_ANGLE = 145;

    //WRIST JOINT VARIABLES TODO tune these value
    public static int WRIST_HOME_POSITION = 950;
    public static double WRIST_PWM_PER_DEGREE = 7.35;
    //IN DEGREES PER SECOND
    public static double WRIST_START_ANGLE = 0;
    public static double WRIST_TRAVEL_ANGLE = 30;
    public static double WRIST_SCORE_ANGLE = 180;

    public static double WRIST_JOINT_SPEED = 75;

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
            case BACKDROP:
                elbow.setTargetAngle(ELBOW_SCORE_ANGLE);
                articulation = Articulation.TRAVEL;
                break;
            case TRAVEL:
                break;
            case TRAVEL_FROM_INGEST:
                if (travelFromIngest())
                    articulation = Articulation.TRAVEL;
                break;
            case INGEST_FROM_TRAVEL:
                if (ingestFromTravel()) {
                    articulation = Articulation.MANUAL;
                }
                break;
            case TRAVEL_FROM_BACKDROP:
                if (travelFromBackdrop()) {
                    articulation = Articulation.TRAVEL;
                }
                break;
            case BACKDROP_PREP:
                if (backdropPrep()) {
                    if (gameState.isAutonomous())
                        articulation = Articulation.BACKDROP;
                    else
                        articulation = Articulation.MANUAL;
                }
                break;

            default:
                break;
        }
        prevArticulation = articulation;
        return articulation;
    }

    public void cleanArticulations() {
        backdropPrepStage = 0;
        travelStage = 0;
        travelStageBack = 0;
        ingestPositionIndex = 0;

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

    public void setTargetAngle(double elbowAngle, double wristAngle, double elevatorAngle) {
        elbow.setTargetAngle(elbowAngle);
        wrist.setTargetAngle(wristAngle);
        elevator.setTargetAngle(elevatorAngle);
    }

    public Outtake(HardwareMap hardwareMap, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        elbow = new Joint(hardwareMap, "elbow", false, ELBOW_HOME_POSITION, ELBOW_PWM_PER_DEGREE, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE, ELBOW_START_ANGLE, ELBOW_JOINT_SPEED);
        elevator = new Joint(hardwareMap, "elevator", false, ELEVATOR_HOME_POSITION, ELEVATOR_PWM_PER_DEGREE, ELEVATOR_MIN_ANGLE, ELEVATOR_MAX_ANGLE, ELEVATOR_START_ANGLE, ELEVATOR_JOINT_SPEED);
        wrist = new Joint(hardwareMap, "wrist", false, WRIST_HOME_POSITION, WRIST_PWM_PER_DEGREE, WRIST_MIN_ANGLE, WRIST_MAX_ANGLE, WRIST_START_ANGLE, WRIST_JOINT_SPEED);
        slide = this.hardwareMap.get(DcMotorEx.class, "slide");
        slide.setMotorEnable();
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);
//        pixelFlipper = this.hardwareMap.get(Servo.class, "pixelFlipper");

        articulation = Articulation.MANUAL;
    }

    public double getArmTheta(double theta) {
        return (theta*(Math.sqrt(Math.pow(elevatorTopBone, 2) + Math.pow(elevatorBottomBone, 2) - 2*elevatorBottomBone*elevatorTopBone*Math.cos(theta))/(2*Math.sin(theta))))/armLengthToElevator;
    }

    public boolean elbowWristIK(int x, int z) {
        z-=armHeight;
        wristTargetAngle = Math.toDegrees(Math.acos((Math.pow(x, 2)+Math.pow(z, 2)-Math.pow(elbowLength, 2)-Math.pow(wristLength, 2))/(-2*elbowLength*wristLength)));
        double b = Math.toDegrees(Math.acos((Math.pow(wristLength, 2)-Math.pow(elbowLength, 2)-Math.pow(x, 2)-Math.pow(z, 2))/(-2*elbowLength*Math.hypot(x, z))));
        double a = Math.toDegrees(Math.atan(z/x));
        elbowTargetAngle = 180+Math.toDegrees(Math.atan(armHeight/armBase))-a-b;
        if(Double.isNaN(wristTargetAngle) || Double.isNaN(elbowTargetAngle)) {
            return false;
        }
        wrist.setTargetAngle(wristTargetAngle);
        elbow.setTargetAngle(elbowTargetAngle);
        return true;
    }

    //should use the calculated IK formula to move to the field coordinate in inches (x, z)
    public boolean goToPoint(int x, int z) {
        try {
            theta = 0;
            top = Math.pow(z - armHeight, 2) - Math.pow(Robot.sensors.outtakeSlideTicks * ticksPerInch * Math.sin(armTheta) - armHeight, 2);
            bottom = Math.pow(x - armBase, 2) - Math.pow(Robot.sensors.outtakeSlideTicks * ticksPerInch * Math.sin(armTheta) - armBase, 2);
            frac = Math.sqrt(top/bottom);
            if (top < 0 && bottom < 0 || top > 0 && bottom > 0)
                theta = Math.asin(frac);
            else if (top < 0) {
                int targetPos = (int) (Robot.sensors.outtakeSlideTicks + Math.sqrt(Math.abs(top)* ticksPerInch + 1 * ticksPerInch));
                if(targetPos > slidePositionMax) //if outside of the slides range fail
                    return false;
                slideTargetPosition = targetPos; //move the slide to make the final position possible
                top = Math.pow(z - armHeight, 2) - Math.pow(targetPos * ticksPerInch * Math.sin(armTheta) - armHeight, 2);
                bottom = Math.pow(x - armBase, 2) - Math.pow(targetPos * ticksPerInch * Math.sin(armTheta) - armBase, 2);
                frac = Math.sqrt(top/bottom);
                theta = Math.asin(frac);
            } else {
                int targetPos = (int) (Robot.sensors.outtakeSlideTicks + Math.sqrt(Math.abs(bottom) * ticksPerInch + 1 * ticksPerInch));
                if(targetPos > slidePositionMax) //if outside of the slides range fail
                    return false;
                slideTargetPosition = targetPos; //move the slide to make the final position possible
                top = Math.pow(z - armHeight, 2) - Math.pow(targetPos * ticksPerInch * Math.sin(armTheta) - armHeight, 2);
                bottom = Math.pow(x - armBase, 2) - Math.pow(targetPos * ticksPerInch * Math.sin(armTheta) - armBase, 2);
                frac = Math.sqrt(top/bottom);
                theta = Math.asin(frac);
            }
            if(theta != Double.NaN) {
//                flipper.setTargetAngle(Math.toDegrees(theta)); // move the flipper angle to the calculated angle
                return true;
            }
            else
                return false;
        }
        catch(ArithmeticException e) { //if the math throws an exception because of impossible trig fail
            return false;
        }
    }

    public int ingestPositionIndex = 0;
    public long ingestPositionTimer = 0;

    public boolean ingestFromTravel() { //should only call this if Travel was previously set
        switch (ingestPositionIndex) {
            case 0: //position outtake to position scoopagon just clear of the intake belt
                setSlideTargetPosition(slidePositionPreDock);
                ingestPositionIndex++;
                break;
            case 1: //lower the outtake to intake position
                if (between(Robot.sensors.outtakeSlideTicks, slidePositionPreDock + 10, slidePositionPreDock - 10)) {
                    elbow.setSpeed(ELBOW_JOINT_SPEED);
                    setTargetAngle(ELBOW_DOCK_ANGLE, WRIST_START_ANGLE, ELEVATOR_START_ANGLE);
                    ingestPositionTimer = futureTime(.85);
                    ingestPositionIndex++;
                }
                break;
            case 2: //give enough time to pull down flipper, then slide to intake dock
                if (isPast(ingestPositionTimer)) {
                    elbow.setSpeed(ELBOW_JOINT_SPEED);
                    setSlideTargetPosition(slidePositionDocked);
                    ingestPositionIndex = 0;
                    return true;
                }
                break;
        }
        return false;
    }

    long travelTimer = 0;
    int travelStage = 0;

    boolean travelFromIngest() {
        switch (travelStage) {
            case 0: //begin undock
                setTargetAngle(ELBOW_DOCK_ANGLE, WRIST_START_ANGLE, ELEVATOR_START_ANGLE);
//                setSlideTargetPosition(slidePositionPreDock);
                travelTimer = futureTime(.5);
                travelStage++;
                break;
            case 1: //complete undock
                if (isPast(travelTimer)) {
                    setTargetAngle(ELBOW_TRAVEL_ANGLE, WRIST_TRAVEL_ANGLE, ELEVATOR_START_ANGLE);
                    travelTimer = futureTime(.5);
                    travelStage++;
                }
                break;
            case 2: //tuck slide - todo this will get more complicated when outtake elevation is changeable
                if (isPast(travelTimer)) {
//                    setSlideTargetPosition(slidePositionDocked);
                    robot.articulate(Robot.Articulation.TRAVEL);
                    travelStage = 0;
                    return true;
                }
                break;
        }
        return false;
    }

    public static int backdropPrepStage = 0;
    long backdropPrepTimer = 0;

    boolean backdropPrep() {
        switch (backdropPrepStage) {
            case 0:
                slideTargetPosition = slidePositionPreDock;
                if (withinError(Robot.sensors.outtakeSlideTicks, slidePositionPreDock, 30)) {
                    backdropPrepTimer = futureTime(1);
                    backdropPrepStage++;
                }
                break;
            case 1:
//                elbow.setTargetAngle(ELBOW_PRE_SCORE_ANGLE);
//                wrist.setTargetAngle(WRIST_START_ANGLE);
                elbowWristIK(5, 13);
                if (isPast(backdropPrepTimer)) {
                    backdropPrepStage++;
                }
                break;
            case 2:
                slideTargetPosition = slidePositionScore;
                if (withinError(Robot.sensors.outtakeSlideTicks, slidePositionDocked, 30)) {
                    backdropPrepStage++;
                }
                break;
            case 3:
                backdropPrepStage = 0;
                backdropPrepTimer = 0;
                return true;

        }
        return false;
    }

    int travelStageBack = 0;

    boolean travelFromBackdrop() {
        switch (travelStageBack) { //robot should have already placed the intake into travel position
            case 0:
                ELBOW_PRE_SCORE_ANGLE = elbow.getCurrentAngle();
                setTargetAngle(ELBOW_TRAVEL_ANGLE, WRIST_TRAVEL_ANGLE, ELEVATOR_START_ANGLE);
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
        elbow.setTargetAngle(elbow.getCurrentAngle() + angle);
    }


    public void flipperTest() {
        if (TEMP_FLIPPER_TUNE) {
            elbow.setTargetAngle(elbow.getCurrentAngle() + 1);
        } else
            elbow.setTargetAngle(elbow.getCurrentAngle() - 1);
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
        //actually instruct actuators to go to desired targets
        elbow.update();
        wrist.update();
        slide.setTargetPosition(slideTargetPosition);
        armTheta = getArmTheta(elevator.getCurrentAngle());
        //compute values for kinematics
        double rotTheta = -elbow.getCurrentAngle() -180;
        double rotX = armLength*Math.cos(armTheta) + (Robot.sensors.outtakeSlideTicks/ticksPerInch)*Math.cos(armTheta);
        double rotZ = armLength*Math.sin(armTheta) + (Robot.sensors.outtakeSlideTicks/ticksPerInch)*Math.sin(armTheta);
        double x = (Robot.sensors.outtakeSlideTicks/ticksPerInch)*Math.cos(armTheta);
        double z = (Robot.sensors.outtakeSlideTicks/ticksPerInch)*Math.sin(armTheta);
        armX = (rotX-(z-rotZ)*Math.sin(Utils.degreeToRad(rotTheta))+(x-rotX)*Math.cos(Utils.degreeToRad(rotTheta)));
        armZ = (rotZ+(z-rotZ)*Math.cos(Utils.degreeToRad(rotTheta))+(x-rotX)*Math.sin(Utils.degreeToRad(rotTheta)));
    }

    @Override
    public void stop() {
        slide.setMotorDisable();
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("articulation", articulation.name());
        telemetryMap.put("ingest stage", ingestPositionIndex);
        telemetryMap.put("slide target position", slideTargetPosition);
        telemetryMap.put("slide actual position", Robot.sensors.outtakeSlideTicks);
        telemetryMap.put("slide actual position (inches)", Robot.sensors.outtakeSlideTicks/ticksPerInch);
//        telemetryMap.put("flipper location", Utils.servoDenormalize(pixelFlipper.getPosition()));
        telemetryMap.put("elbow ticks", flipperPosition);
        telemetryMap.put("elbow angle", elbow.getCurrentAngle());
        telemetryMap.put("elbow target angle", elbow.getTargetAngle());
        telemetryMap.put("wrist angle", wrist.getCurrentAngle());
        telemetryMap.put("wrist target angle", wrist.getTargetAngle());
        telemetryMap.put("arm location", "("+armX+", "+armZ+")");
        telemetryMap.put("arm theta", armTheta);
        telemetryMap.put("wrist target angle", wristTargetAngle);
        telemetryMap.put("elbow target angle", elbowTargetAngle);
        telemetryMap.put("IK variables", "top: "+top+" bottom: "+bottom+" theta: "+theta+" frac: "+frac);
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "OUTTAKE";
    }
}
