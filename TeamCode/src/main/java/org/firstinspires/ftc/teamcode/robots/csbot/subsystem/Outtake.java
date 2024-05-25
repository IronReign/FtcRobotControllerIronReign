package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.gameState;
import static org.firstinspires.ftc.teamcode.util.utilMethods.withinError;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.csbot.util.Joint;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Utils;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "AA_CS_OUTTAKE")
public class Outtake implements Subsystem {

    public static double TRAVEL_FROM_BACKDROP_TIMER = 1.2;
    public static double BACKDROP_PREP_TIMER = .6;
    HardwareMap hardwareMap;
    Robot robot;

    public DcMotorEx slide = null;

    public Joint elbow;
    public Joint wrist;
    public Joint elevator;

    public static double slideTicksPerInch = 130.593132; // determined experimentally using a average distance and ticks

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
    public static double elbowTargetAngle, wristTargetAngle, elevatorTargetAngle;
    private double theta, top, bottom, frac;

    public static int flipperPosition = 1888;
    public static boolean ikCalculated = false;


    public void setSlideTargetPosition(int slideTargetPosition) {
        this.slideTargetPosition = slideTargetPosition;
    }

    int slideTargetPosition = 0;
    public static int slidePositionMax = 1600;
    public static int slidePositionMin = 0;
    public static int slidePositionPreDock = 500;
    public static int slidePositionDocked = 0;
    public static int slidePositionScore = 400;
    public static double scoreZ = 16;
    public static double scoreX = 4;
    public static double elevatorIKAngle = 10;
    public static double wristIKAngle = 0;
    public static double elbowIKAngle = 0;


    public static int slideSpeed = 15;jhfjhfhyxxyrxyx
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

    //ELEVATOR JOINT VARIABLES TODO tune these value
    public static int ELEVATOR_HOME_POSITION = 950;
    public static double ELEVATOR_PWM_PER_DEGREE = 7.35*(4.0/3);
    //IN DEGREES PER SECOND
    public static double ELEVATOR_START_ANGLE = 0;

    public static double ELEVATOR_JOINT_SPEED = 75;

    public static double ELEVATOR_MIN_ANGLE = 0;
    public static double ELEVATOR_MAX_ANGLE = 120;

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
            case START:
                setTargetAngle(ELBOW_START_ANGLE, WRIST_INIT_ANGLE, ELEVATOR_START_ANGLE);
                break;
            case MANUAL:
                break;
            case BACKDROP:
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
        FOLD,
        START
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
        wrist.setTargetAngle(WRIST_INIT_ANGLE);
        slide.setMotorEnable();
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setVelocity(SLIDE_SPEED);
        elevatorIKAngle = 0;
        scoreX = 4;
        scoreZ = 16;

        articulation = Articulation.MANUAL;
    }

    public double getArmTheta(double theta) {
        return (theta*(Math.sqrt(Math.pow(elevatorTopBone, 2) + Math.pow(elevatorBottomBone, 2) - 2*elevatorBottomBone*elevatorTopBone*Math.cos(theta))/(2*Math.sin(theta))))/armLengthToElevator;
    }

    public boolean elbowWristIK(double x, double z) {
        armTheta = Math.atan2(armHeight, armBase);
        x = x/Math.cos(armTheta);
        z = z/Math.cos(armTheta);
//        x-=(slide.getCurrentPosition()/slideTicksPerInch)*Math.cos(armTheta);
        z-=(slide.getCurrentPosition()/slideTicksPerInch)*Math.sin(armTheta);
        z-=armHeight;
        wristTargetAngle = Math.toDegrees(Math.acos((Math.pow(x, 2)+Math.pow(z, 2)-Math.pow(elbowLength, 2)-Math.pow(wristLength, 2))/(2*elbowLength*wristLength)));
        double b = Math.toDegrees(Math.acos((Math.pow(wristLength, 2)-Math.pow(elbowLength, 2)-Math.pow(x, 2)-Math.pow(z, 2))/(-2*elbowLength*Math.hypot(x, z))));
        double a = Math.toDegrees(Math.atan2(z, x));
        elbowTargetAngle = 180+Math.toDegrees(Math.atan2(armHeight, armBase))-a-b;
        if(Double.isNaN(wristTargetAngle) || Double.isNaN(elbowTargetAngle)) {
            wristTargetAngle = 0;//maxes the height of the outtake
            elbowTargetAngle = 90 + Math.toDegrees(Math.atan2(armHeight, armBase));// maxes the height of the outtake
            wristIKAngle = wristTargetAngle;
            elbowIKAngle = elbowTargetAngle;
            return false;
        }
        else {
            wristIKAngle = wristTargetAngle;
            elbowIKAngle = elbowTargetAngle;
            return true;
        }
    }

    public void IKadjust(double x, double z) {
        armTheta = Math.atan2(armHeight, armBase);
        x = x/Math.cos(armTheta);
        z = z/Math.cos(armTheta);
//        x-=(slide.getCurrentPosition()/slideTicksPerInch)*Math.cos(armTheta);
        z-=(slide.getCurrentPosition()/slideTicksPerInch)*Math.sin(armTheta);
        z-=armHeight;
        wristTargetAngle = Math.toDegrees(Math.acos((Math.pow(x, 2)+Math.pow(z, 2)-Math.pow(elbowLength, 2)-Math.pow(wristLength, 2))/(2*elbowLength*wristLength)));
        double b = Math.toDegrees(Math.acos((Math.pow(wristLength, 2)-Math.pow(elbowLength, 2)-Math.pow(x, 2)-Math.pow(z, 2))/(-2*elbowLength*Math.hypot(x, z))));
        double a = Math.toDegrees(Math.atan2(z, x));
        elbowTargetAngle = 180+Math.toDegrees(Math.atan2(armHeight, armBase))-a-b;
        if(Double.isNaN(wristTargetAngle) || Double.isNaN(elbowTargetAngle)) {
            wristTargetAngle = 0;//maxes the height of the outtake
            elbowTargetAngle = 90 + Math.toDegrees(Math.atan2(armHeight, armBase));// maxes the height of the outtake
            wrist.setTargetAngle(wristTargetAngle);
            elbow.setTargetAngle(elbowTargetAngle);
        }
        else {
            wrist.setTargetAngle(wristTargetAngle);
            elbow.setTargetAngle( elbowTargetAngle);
        }
    }
    public int ingestPositionIndex = 0;
    public long ingestPositionTimer = 0;

    public boolean ingestFromTravel() { //should only call this if Travel was previously set
        switch (ingestPositionIndex) {
            case 0: //lower the outtake to intake position
                ELBOW_JOINT_SPEED = 60;
                WRIST_JOINT_SPEED = 50;
                setTargetAngle(ELBOW_DOCK_ANGLE, WRIST_START_ANGLE, ELEVATOR_START_ANGLE);
                ingestPositionTimer = futureTime(.85);
                ingestPositionIndex++;
                break;
            case 1: //give enough time to pull down flipper, then slide to intake dock
                if (isPast(ingestPositionTimer)) {
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
                setSlideTargetPosition(slidePositionPreDock);
                travelTimer = futureTime(.5);
                travelStage++;
                break;
            case 1: //complete undock
                if (isPast(travelTimer)) {
                    ELBOW_JOINT_SPEED = 40;
                    WRIST_JOINT_SPEED = 50;
                    setTargetAngle(ELBOW_TRAVEL_ANGLE, WRIST_TRAVEL_ANGLE, ELEVATOR_START_ANGLE);
                    travelTimer = futureTime(1);
                    travelStage++;
                }
                break;
            case 2: //tuck slide - todo this will get more complicated when outtake elevation is changeable
                if (isPast(travelTimer)) {
                    setSlideTargetPosition(slidePositionDocked);
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
//                    Sensors.distanceSensorsEnabled = true;
                    ELBOW_JOINT_SPEED = 120;
                    WRIST_JOINT_SPEED = 150;
                    scoreX=7;
                    elbowWristIK(scoreX, scoreZ);
                    backdropPrepStage++;
                }
                break;
            case 1:
                elevator.setTargetAngle(elevatorIKAngle);
                wrist.setTargetAngle(wristIKAngle);
                backdropPrepTimer = futureTime(1);
                backdropPrepStage ++;
//                scoreX = (Robot.sensors.averageDistSensorValue);
                break;
            case 2:

                if(isPast(backdropPrepTimer)) {
                    elbow.setTargetAngle(elbowIKAngle);
                    backdropPrepStage++;
                }
                break;
            case 3:
                slideTargetPosition = slidePositionScore;
                if (withinError(Robot.sensors.outtakeSlideTicks, slidePositionScore, 30)) {
                    backdropPrepStage++;
                }
                break;
            case 4:
                if(withinError(elbow.getCurrentAngle(), elbowIKAngle, 2) && withinError(wrist.getCurrentAngle(), wristIKAngle, 2)) {
                    backdropPrepStage = 0;
                    backdropPrepTimer = 0;
                    ELBOW_JOINT_SPEED = 60;
                    WRIST_JOINT_SPEED = 50;
                }
                return true;

        }
        return false;
    }

    int travelStageBack = 0;

    boolean travelFromBackdrop() {
        switch (travelStageBack) { //robot should have already placed the intake into travel position
            case 0:
                ELBOW_JOINT_SPEED = 80;
                WRIST_JOINT_SPEED = scoreZ > 30 ? 50: scoreZ > 18 ? 60 : 70;;
                ELBOW_PRE_SCORE_ANGLE = elbow.getCurrentAngle();
//                setTargetAngle(ELBOW_TRAVEL_ANGLE, wrist.getCurrentAngle()+40, ELEVATOR_START_ANGLE);
//                travelTimer = futureTime(.5);
                travelStageBack++;
                break;
            case 1:
                if (isPast(travelTimer)) {
                    setTargetAngle(ELBOW_TRAVEL_ANGLE, WRIST_TRAVEL_ANGLE, ELEVATOR_START_ANGLE);
                    travelTimer = futureTime(TRAVEL_FROM_BACKDROP_TIMER);
                    travelStageBack++;
                }
                break;
            case 2:
                if(isPast(travelTimer)) {
                    setSlideTargetPosition(0);
                    ELBOW_JOINT_SPEED = 60;
                    WRIST_JOINT_SPEED = 50;
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

    public void adjustElbow(double angle) {
        elbow.setTargetAngle(elbow.getCurrentAngle() + angle);
    }

    public void adjustWrist(double angle) {
        wrist.setTargetAngle(wrist.getCurrentAngle() + angle);
    }

    public void adjustElevator(double angle) { elevator.setTargetAngle(elevator.getCurrentAngle() + angle); }


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
        wrist.setSpeed(WRIST_JOINT_SPEED);
        elevator.setSpeed(ELEVATOR_JOINT_SPEED);
        //actually instruct actuators to go to desired targets
        elbow.update();
        wrist.update();
        elevator.update();
        slide.setTargetPosition(slideTargetPosition);
        armTheta = Math.atan2(armHeight, armBase);
//        armTheta = getArmTheta(elevator.getCurrentAngle());
        //compute values for kinematics
        double rotTheta = -elbow.getCurrentAngle() -180;
        double rotX = armLength*Math.cos(armTheta) + (Robot.sensors.outtakeSlideTicks/ slideTicksPerInch)*Math.cos(armTheta);
        double rotZ = armLength*Math.sin(armTheta) + (Robot.sensors.outtakeSlideTicks/ slideTicksPerInch)*Math.sin(armTheta);
        double x = (Robot.sensors.outtakeSlideTicks/ slideTicksPerInch)*Math.cos(armTheta);
        double z = (Robot.sensors.outtakeSlideTicks/ slideTicksPerInch)*Math.sin(armTheta);
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
        telemetryMap.put("slide actual position (inches)", Robot.sensors.outtakeSlideTicks/ slideTicksPerInch);
//        telemetryMap.put("flipper location", Utils.servoDenormalize(pixelFlipper.getPosition()));
        telemetryMap.put("elbow ticks", flipperPosition);
        telemetryMap.put("elbow angle", elbow.getCurrentAngle());
        telemetryMap.put("elbow target angle", elbow.getTargetAngle());
        telemetryMap.put("elbow joint speed", ELBOW_JOINT_SPEED);
        telemetryMap.put("wrist angle", wrist.getCurrentAngle());
        telemetryMap.put("wrist target angle", wrist.getTargetAngle());
        telemetryMap.put("wrist joint speed", WRIST_JOINT_SPEED);
        telemetryMap.put("elevator angle", elevator.getCurrentAngle());
        telemetryMap.put("elevator target angle", elevator.getTargetAngle());
        telemetryMap.put("elevator joint speed", ELEVATOR_JOINT_SPEED);
        telemetryMap.put("arm location", "("+armX+", "+armZ+")");
        telemetryMap.put("arm theta", armTheta);
        telemetryMap.put("wrist target angle", wristTargetAngle);
        telemetryMap.put("elbow target angle", elbowTargetAngle);
        telemetryMap.put("elevator target angle", elevatorTargetAngle);
        telemetryMap.put("ScoreX, ScoreZ", scoreX+", "+scoreZ);
        telemetryMap.put("IK variables", "top: "+top+" bottom: "+bottom+" theta: "+theta+" frac: "+frac);
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "OUTTAKE";
    }
}
