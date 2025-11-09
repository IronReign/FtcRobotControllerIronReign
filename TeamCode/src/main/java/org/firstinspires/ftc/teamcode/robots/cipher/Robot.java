package org.firstinspires.ftc.teamcode.robots.cipher;

import static org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils.wrapAngle;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.LinkedHashMap;
import java.util.Map;
@Config(value = "Cipher")
public class Robot implements Subsystem {
    private final Gamepad gamepad1;
    HardwareMap hardwareMap;
    DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public StickyGamepad spad1;
    Gamepad gamepad;
    long totalRunTime=0;
    long startTime;

    //pid stuff
    BNO055IMU imu;
    public static PIDController headingPID;
    public static PIDCoefficients HEADING_PID_PWR = new PIDCoefficients(0.03, 0.04, 0);
    public static double HEADING_PID_TOLERANCE = .08;           //.08 //this is a percentage of the input range .063 of 2PI is 1 degree
    private double PIDCorrection, PIDError, targetHeading, targetDistance;
    boolean imuTurnDone = false;

    public DistanceSensor sensorDistance;
    public static PIDController distancePID;
    public static PIDCoefficients DISTANCE_PID_PWR = new PIDCoefficients(0.03, 0.04, 0);
    public static double DISTANCE_PID_TOLERANCE = .01; //this is a percentage of the input range .063 of 2PI is 1 degree
    private double distPIDCorrection, distPIDError;
    boolean distDriveDone;

    public static PIDController travelPID; //straight line travel using odopods
    public static PIDCoefficients TRAVEL_PID_PWR = new PIDCoefficients(0.03, 0.04, 0);
    public static double TRAVEL_PID_TOLERANCE = .01; //this is a percentage of the input range .063 of 2PI is 1 degree
    private double travelPIDCorrection, travelPIDError;
    boolean travelDriveDone;

    public Robot(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad;
        spad1 = new StickyGamepad(gamepad1);

        // Call motors from hardwareMap
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Restart motors
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        initIMU();

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorDistance");

        // init PID controllers
        headingPID = new PIDController(HEADING_PID_PWR); // for turning
        headingPID.setInputRange(0, 360);
        headingPID.setOutputRange(-1, 1);
        headingPID.setIntegralCutIn(4);
        headingPID.setContinuous(true);
        headingPID.setTolerance(HEADING_PID_TOLERANCE);
        headingPID.enable();

        distancePID = new PIDController(DISTANCE_PID_PWR); // for distance sensor
        distancePID.setInputRange(0, 200); //2 meter distance sensor
        distancePID.setOutputRange(-1, 1);
        distancePID.setIntegralCutIn(3);
        distancePID.setContinuous(false);
        distancePID.setTolerance(DISTANCE_PID_TOLERANCE);
        distancePID.enable();

        travelPID = new PIDController(TRAVEL_PID_PWR); // for travel sensor
        travelPID.setInputRange(0, 200); //2 meter travel sensor
        travelPID.setOutputRange(-1, 1);
        travelPID.setIntegralCutIn(3);
        travelPID.setContinuous(false);
        travelPID.setTolerance(TRAVEL_PID_TOLERANCE);
        travelPID.enable();


        totalRunTime=0;
        startTime = System.currentTimeMillis();
    }

    public void mecanumDrive(double forward, double strafe, double turn) {
        double negS = -strafe;
        double r = Math.hypot(negS, forward);
        double robotAngle = Math.atan2(forward, negS) - Math.PI / 4;
        leftFront.setPower(-((r * Math.cos(robotAngle) - turn)));
        rightFront.setPower(-((r * Math.sin(robotAngle) + turn)));
        leftBack.setPower(-((r * Math.sin(robotAngle) - turn)));
        rightBack.setPower(-((r * Math.cos(robotAngle) + turn)));
    }

    public float getZorient(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public void initIMU(){
        imu = hardwareMap.get(BNO055IMU.class, "eimu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(parameters);
    }

    public void testDistanceSensor(){
        driveDistance(50, 1);
    }

    int testTurnAngle;
    public void testTurn(){
        turnUntilDegreesIMU(testTurnAngle, 1);
    }

    @Override
    public void update(Canvas fieldOverlay) {
        spad1.update();
        updateMotors();
        mecanumDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
    }

    Orientation angles = new Orientation();
    //request a turn in degrees units
    //this is an absolute (non-relative) implementation.
    //the direction of the turn will favor the shortest approach
    public boolean turnUntilDegreesIMU(double turnAngle, double maxSpeed) {
        targetHeading = wrapAngle(turnAngle);
        headingPID.setPID(HEADING_PID_PWR);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        headingPID.setInput(angles.firstAngle); // todo - is this yaw?
        headingPID.setSetpoint(targetHeading);
        headingPID.setOutputRange(-maxSpeed, maxSpeed);
        headingPID.setTolerance(HEADING_PID_TOLERANCE);
        double correction = headingPID.performPID();
        PIDCorrection = correction;
        PIDError = headingPID.getError();
        if (headingPID.onTarget()) {
            //turn meets accuracy requirement
            mecanumDrive(0,0,0);
            return imuTurnDone = true;
        } else {
            headingPID.enable();
            mecanumDrive(0,0,-correction);
            return imuTurnDone = false;
        }
    }

    // pid to drive to a target distance sensor value
    public boolean driveDistance(double distance, double maxSpeed) {
        targetDistance = wrapAngle(distance);
        distancePID.setPID(DISTANCE_PID_PWR);
        distancePID.setInput(sensorDistance.getDistance(DistanceUnit.CM));  // distance reported by distance sensor
        distancePID.setSetpoint(targetDistance);
        distancePID.setOutputRange(-maxSpeed, maxSpeed);
        distancePID.setTolerance(DISTANCE_PID_TOLERANCE);
        double correction = distancePID.performPID();
        distPIDCorrection = correction;
        distPIDError = distancePID.getError();
        if (distancePID.onTarget()) {
            //distance meets accuracy requirement
            mecanumDrive(0,0,0);
            return distDriveDone = true;
        } else {
            distancePID.enable();
            mecanumDrive(correction,0,0);
            return distDriveDone = false;
        }
    }

    public boolean strafeDistance(double distance, double maxSpeed) {
        targetDistance = wrapAngle(distance);
        distancePID.setPID(DISTANCE_PID_PWR);
        distancePID.setInput(sensorDistance.getDistance(DistanceUnit.CM));  // distance reported by distance sensor
        distancePID.setSetpoint(targetDistance);
        distancePID.setOutputRange(-maxSpeed, maxSpeed);
        distancePID.setTolerance(DISTANCE_PID_TOLERANCE);
        double correction = distancePID.performPID();
        distPIDCorrection = correction;
        distPIDError = distancePID.getError();
        if (distancePID.onTarget()) {
            //distance meets accuracy requirement
            mecanumDrive(0,0,0);
            return distDriveDone = true;
        } else {
            distancePID.enable();
            mecanumDrive(0,-correction,0);
            return distDriveDone = false;
        }
    }

    @Override
    public void stop() {

    }

    @Override
    public void resetStates() {

    }

    public void init() {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        // Set motor run modes
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //shoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("LeftBack", leftBack.getCurrentPosition());
        telemetry.put("RightBack", rightBack.getCurrentPosition());
        telemetry.put("RightFront", rightFront.getCurrentPosition());

        return telemetry;
    }

    public void updateMotors() {

    }

    @Override
    public String getTelemetryName() {
        return "Reign Robot";
    }}
