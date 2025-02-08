package org.firstinspires.ftc.teamcode.robots.core;

import static org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils.joysticksActive;
import static org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils.notJoystickDeadZone;
import static org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils.wrapAngle;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.LinkedHashMap;
import java.util.Map;
@Config(value = "CORE_ROBOT")
public class Robot implements Subsystem {
    HardwareMap hardwareMap;
    DcMotorEx leftFront, leftBack, rightFront, rightBack;
    DcMotorEx vertical, horizontal;
    Servo claw;
    Gamepad gamepad1;
    DcMotorEx shoulder;
    DcMotorEx slide;
    public static final double CURRENT_THRESHOLD = 2.2;
    public static int CALIBRATE_POSITION = Integer.MAX_VALUE;
    long autonTimer = 0;

    long totalRunTime=0;
    long startTime;

    public boolean clawOpen = false;
    public double clawOpenPosition = 1;
    public double clawClosePosition = .55;
    public static int shoulderTargetPosition = 0;
    public static int slideTargetPosition = 0;
    public int rotaterPosition = 0;
    private boolean motorUpdated;
    public StickyGamepad spad1;
    
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
    
    

    public Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad;
        spad1 = new StickyGamepad(gamepad1);

        // Call motors from hardwareMap
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        claw = hardwareMap.get(Servo.class, "claw");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        vertical = hardwareMap.get(DcMotorEx.class, "vertical");
        horizontal = hardwareMap.get(DcMotorEx.class, "horizontal");

        // Restart motors
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vertical.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        shoulder.setPower(1);
        shoulder.setVelocity(50);
        shoulder.setTargetPosition(shoulder.getCurrentPosition());
        shoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slide.setPower(1);
        slide.setVelocity(50);
        slide.setTargetPosition(0);
        slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        horizontal.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        vertical.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        claw.setPosition(clawClosePosition);

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

    @Override
    public void update(Canvas fieldOverlay) {
        spad1.update();
        motorUpdated = false;
        // Claw Controls
        if(spad1.b) {
            clawOpen = !clawOpen;
        }

        if(spad1.a) { //preset for ground
            if (shoulderTargetPosition < 400) // over sub barrier
                shoulderTargetPosition = 560;
            else
                shoulderTargetPosition = 330; //ground
            slideTargetPosition = 440;
        }

        if(spad1.x) { //preset for wall
            shoulderTargetPosition= 824;
            slideTargetPosition = 30;
        }
        if(spad1.y) { //preset for specimen score
            shoulderTargetPosition = 1900;
            slideTargetPosition = 280;
        }
        if(clawOpen) {
            claw.setPosition(clawOpenPosition);
        } else {
            claw.setPosition(clawClosePosition);
        }

        //TODO: driver testing (8:00-9:30)

        // Attaching the specimen on the high bar preset (test)
        if(spad1.left_bumper){
            shoulderTargetPosition=1800;
            slideTargetPosition=350;
            autonTimer = futureTime(1);
            if(isPast(autonTimer)) {
                shoulderTargetPosition=1360;
            }
        }

        // Attaching picking up the specimen from the wall (test)
        if(spad1.right_bumper){
            shoulderTargetPosition=824;
            slideTargetPosition=30;
            autonTimer=futureTime(1);
            if(isPast(autonTimer)){
                claw.setPosition(clawClosePosition);
                shoulderTargetPosition=1800;
            }
        }

        if(gamepad1.left_trigger >= 0.3){
            if (shoulder.getCurrentPosition() < 2000){
                shoulderTargetPosition = shoulder.getCurrentPosition() + 75;
            } else {
                shoulderTargetPosition = 2000;
            }
        }
        else if (gamepad1.right_trigger >= 0.3){
            if (shoulder.getCurrentPosition() > 0){
                shoulderTargetPosition = shoulder.getCurrentPosition() - 75;
            } else {
                shoulderTargetPosition = 0;
            }

        }

        if (spad1.dpad_up){
            if (slide.getCurrentPosition() < 1200){
                slideTargetPosition = slide.getCurrentPosition() + 60;
            } else {
                slide.setTargetPosition(1200);
            }
        }
        if (spad1.dpad_down){
            if(slide.getCurrentPosition() > 60){
                slideTargetPosition = slide.getCurrentPosition() - 60;
            } else {
                slide.setTargetPosition(0);
            }
        }

        updateMotors();
        if (joysticksActive(gamepad1))
            mecanumDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
    }

    static boolean calibrated = true;
    public void initloopDrive(){
        spad1.update();
        if (spad1.guide) {
            calibrateStage = 0;
            calibrated = false;
        }
        if (! calibrated){
            {
                if (calibrate()) {
                    calibrateStage=0;
                    calibrated = true;
                }
            }
        }

        if (gamepad1.dpad_up) //hold dpad to test distance keeping
            testDistanceSensor();

        if (gamepad1.dpad_down) // hold dpad down to test pid turns
            testTurn();

        if (spad1.dpad_left) // hit left to change test targe
            testTurnAngle = (testTurnAngle - 90) % 360;

        if (spad1.dpad_right) // hit left to change test targe
            testTurnAngle = (testTurnAngle + 90) % 360;

        updateMotors();
        if (joysticksActive(gamepad1))
            mecanumDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
    }

    public int debugAuton(int autonIndex){
        spad1.update();
        if(spad1.a){
            autonIndex++;
        }
        if(spad1.x){
            autonIndex--;
        }
        return autonIndex;
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
    public void updateMotors() {

        if (clawOpen) {
            claw.setPosition(clawOpenPosition);
        } else {
            claw.setPosition(clawClosePosition);
        }
        shoulder.setTargetPosition(shoulderTargetPosition);
        motorUpdated = true;
        slide.setTargetPosition(slideTargetPosition);
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
        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        claw = hardwareMap.get(Servo.class, "claw");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        horizontal = hardwareMap.get(DcMotorEx.class, "horizontal");
        vertical = hardwareMap.get(DcMotorEx.class, "vertical");
        // Set motor run modes
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vertical.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        shoulder.setPower(1);
        shoulder.setVelocity(300);
        shoulder.setTargetPosition(1467);
        shoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slide.setPower(1);
        slide.setVelocity(300);
        slide.setTargetPosition(44);
        slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        vertical.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        claw.setPosition(clawOpenPosition);
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

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Claw Open", clawOpen);
        telemetry.put("Rotater", rotaterPosition);
        telemetry.put("Shoulder Power", shoulder.getCurrent(CurrentUnit.AMPS));
        telemetry.put("Shoulder Position", shoulder.getCurrentPosition());
        telemetry.put("Shoulder Target Position", shoulderTargetPosition);
        telemetry.put("Shoulder runMode", shoulder.getMode());
        telemetry.put("motor updated", motorUpdated);

        telemetry.put("Slide Position", slide.getCurrentPosition());
        telemetry.put("Slide Target Position", slideTargetPosition);
        telemetry.put("Horizontal", horizontal.getCurrentPosition());
        telemetry.put("Vertical", vertical.getCurrentPosition());
        telemetry.put("Calibrate Stage", calibrateStage);
        telemetry.put("testTurnAngle", testTurnAngle);
        telemetry.put("turn pid err", headingPID.getError());
        telemetry.put("imu", angles.firstAngle);
        telemetry.put("dist pid err", distancePID.getError());
        telemetry.put("distance", sensorDistance.getDistance(DistanceUnit.CM));

        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "Core Robot";
    }

    public int calibrateStage = 0;
    public boolean calibrate() {
        switch (calibrateStage) {
            case 0:
                shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shoulder.setPower(-.4);
                calibrateStage++;
                break;
            case 1:
                if (CALIBRATE_POSITION == shoulder.getCurrentPosition()) { //detect motion stopped
                    shoulder.setPower(0);
                    shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    calibrateStage++;
                }
                CALIBRATE_POSITION = shoulder.getCurrentPosition();
                break;
            case 2:
                shoulder.setPower(1);
                shoulder.setTargetPosition(1500);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                calibrateStage++;
                break;
            case 3:
                shoulder.setTargetPosition(1500); //1647
                slide.setTargetPosition(0); //44
                calibrateStage=0;
                return true;
        }
        return false;
    }
}