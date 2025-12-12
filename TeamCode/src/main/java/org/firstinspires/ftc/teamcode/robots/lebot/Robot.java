package org.firstinspires.ftc.teamcode.robots.lebot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils.wrapAngle;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddressableDevice;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class Robot implements Subsystem {
    public DcMotor leftBack, rightBack;         //testing only

    static FtcDashboard dashboard = FtcDashboard.getInstance();
    tankDrive drivetrain = new tankDrive();
    private BNO055IMU imu;
    double throttle, spin;

    boolean turning = false;
    boolean turningT = false;
    double refrenceAngle = Math.toRadians(45);
    double integralSum = 0;
    private double lastError = 0;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;

    ElapsedTime timer = new ElapsedTime();

    //public DcMotorEx leftOdo, rightOdo;
    public DcMotorEx leftFront, rightFront;
    public DcMotorEx intake, conveyor, shooter;


    public Servo paddle;
    private final int paddleUp =1420;       //1420
    private final int paddleDown = 985;
    private final int paddleClear=1420;


   public Rev2mDistanceSensor backDist;
    public double dist;
    public double ballNotThere=14;     //change to be whatever dist shows when ball in front of sensor& at paddle

    public int tagIndex=0;

    public int index=0;
    public double minShooterSpeed=935;       //change to speed of flywheel     //1020 for full power
    public ElapsedTime time = new ElapsedTime();
    public double shootTime=.5;      //change to seconds it takes to shoot ball

    public boolean channelDistFull = false;
    HardwareMap hardwareMap;
    StickyGamepad g1 = null;
    Gamepad gamepad1;


    public Servo tilt;
    public int servoUp=1600;
    public int servoDown=1000;
    public Limelight3A limelight;
    public static boolean allianceRed = true;
    public int pipe=0;

    public double distFromTag=0;
    //TODO: find how far limelight from flywheel
    public double c=.10;
    //TODO: get height of camera above floor
    public double h1=15;
    public double h2=0;
    //TODO: get mounting angle
    public double a1=0;
    public double a2=0;


    public boolean shooting = false;
    //private shootingState shootingState;

    /*
    Gamepad Controls:
    - Button to adjust limelight servo
    - Button for fireBall()
    - PID turning
     */

    private int numBalls = 0;
    private final int maxBalls = 3;
    private final double ballThreshold = 2; //TODO: inches adjust
    private boolean frontBlocked = false;
    private boolean backBlocked = false;
    private boolean wasFrontBlocked = false;
    private boolean wasBackBlocked = false;


    public void init() {
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        g1 = new StickyGamepad(gamepad1);

        drivetrain.init(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        conveyor = hardwareMap.get(DcMotorEx.class, "conveyor");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        paddle = hardwareMap.get(Servo.class, "paddle");
        tilt = hardwareMap.get(Servo.class, "tilt");
 //       adjustor = hardwareMap.get(Servo.class, "adjustor");
//
//        frontDist = hardwareMap.get(DistanceSensor.class, "frontDist");
        backDist = hardwareMap.get(Rev2mDistanceSensor.class, "backDist");

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (allianceRed) {
            limelight.pipelineSwitch(1);
            pipe=1;
        } else {
            limelight.pipelineSwitch(0);
            pipe=0;
        }         //pipeline 6 reads blue(20) and red(24)
        limelight.start();

        shooting=false;
        index=0;
        //closedChannel();

        paddle.setPosition(paddleDown);
        intake.setPower(0);
        conveyor.setPower(0);
        shooter.setPower(0);
        tilt.setPosition(servoUp);

        //shootingState = shootingState.RESET;
    }

    public Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad;
    }

    @Override
    public void update(Canvas fieldOverlay) {

        updateDistance();
        if(shooting){
            shootSequence();
        }

//        if(turningT){
//            turnToTag();
//        }
        if(turningT){
            if(tx()){
                drivetrain.turnToTag(0,1);
            }
            //drivetrain.turnUntilDegreesIMU(,1);
        }
        if(drivetrain.gettxTurnDone()){
            turningT=false;
        }
    }
    public void setShoot(boolean x){shooting=x;}

    public void shoot(boolean x) {
        if (x)
            shooter.setVelocity(minShooterSpeed,AngleUnit.DEGREES);
        else
            shooter.setPower(0);
    }

    public void setShoot(double x){shooter.setVelocity(x,AngleUnit.DEGREES);}
    public void resetShootIndex(){index=0;}


    public void shootSequence(){
        //if(tagCenteringSequene()) {
            switch (index) {
                case 0:
                    shooter.setVelocity(minShooterSpeed,AngleUnit.DEGREES);
                    conveyor.setPower(1);
                    intake.setPower(1);
                    index++;
                    break;
                case 1:
                    if (dist < 9) {
                        conveyor.setPower(0);
                        intake.setPower(0);
                        time.reset();
                        index++;
                    }
                    break;
                case 2:
                    if(time.seconds()>1){
                        index++;
                    }
                    break;
                case 3:
                    if (shooter.getVelocity(AngleUnit.DEGREES) >= minShooterSpeed && shooter.getVelocity(AngleUnit.DEGREES)<=minShooterSpeed+8) {
                        setPaddleClear();
                        time.reset();
                        index++;
                    }
                    break;
                case 4:
                    if (time.seconds()>shootTime) {
                        setPaddleDown();
                        shooting=false;
                        index++;
                    }
                    break;
            }
    }

    public boolean getShoot(){
        return shooting;
    }
    public void resetTagIndex(){
        tagIndex=0;
    }

    public boolean tagCenteringSequene(){
        if(tx()) {
            switch (tagIndex) {
                case 0:
                    turningT=true;
                    tagIndex++;
                    break;
                case 1:
                    if(!turningT){
                        tagIndex++;
                    }
                    break;
            }
            return true;
        }
        return false;
    }

    public double angleDif() {
        return getPoseHeading() - getCurrentIMU();
    }

    public void setPaddleUp(){
        paddle.setPosition(servoNormalize(paddleUp));
    }
    public void setPaddleDown(){
        paddle.setPosition(servoNormalize(paddleDown));
    }
    public void setPaddleClear(){
        paddle.setPosition(servoNormalize(paddleClear));
    }
    public void setServoUp(){
        tilt.setPosition(servoUp);
    }
    public void setServoDown(){
        tilt.setPosition(servoDown);
    }

    public void setIntakeSpeed(double x){
        conveyor.setPower(x);
        intake.setPower(x);
    }

    public double getPoseHeading() {
        double headingRadians = getBotPose().getOrientation().getYaw();
        return Math.toRadians(headingRadians);
    }

    public Pose3D getBotPose() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            return llResult.getBotpose();
        }
        return null;
    }

    public double getDistFromTag(){
        distFromTag=(.711-.235)/Math.tan(Math.toRadians(a1+a2))+c;
        //distFromTag=(h2-h1)/Math.tan(Math.toRadians(a1+a2));
        return distFromTag;
    }

    public boolean tx() {
        if (limelight.getLatestResult() != null && limelight.getLatestResult().isValid()) {
            return true;
        }
        return false;
    }

    public double gettx() {
        return limelight.getLatestResult().getTx();
    }

    public double getCurrentIMU() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    public double getRefrenceAngle() {
        return refrenceAngle;
    }

    public void setTurning(boolean x) {
        turning = x;
    }

    public void setTurningT(boolean x) {
        turningT = x;
    }

    public boolean getTurning() {
        return turning;
    }

    public boolean getTurningT() {
        return turningT;
    }

    public void setDrivetrain(double t, double s) {
        drivetrain.drive(t, s);
    }

    public void setRedALliance(boolean x) {
        allianceRed = x;
    }
    public boolean getRedALliance() {
        return allianceRed;
    }

    public void switchPipeline(int x) {
        limelight.pipelineSwitch(x);        //0 is blue 1 is red
        //limelight.start();
    }

    public void intakeOn() {
        conveyor.setPower(1);
        intake.setPower(1);
    }
    public void intakeOff(){
        intake.setPower(0);
        conveyor.setPower(0);
    }

    public void updateDistance(){
        dist=backDist.getDistance(DistanceUnit.CM);
    }
    public static double servoNormalize(int pulse) {
        double normalized = (double) pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }
    @Override
    public void stop() {
    }
    @Override
    public void resetStates() {
    }
    @Override
    public String getTelemetryName() {
        return "lebot robot";
    }
    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry2 = new LinkedHashMap<>();
        TelemetryPacket p = new TelemetryPacket();
        telemetry2.put("Red Alliance??? ", allianceRed);
        telemetry2.put("shooter speed: ",shooter.getVelocity(AngleUnit.DEGREES));
        telemetry2.put("back distance sensor: ",dist);
        telemetry2.put("shooting state ", index);
        telemetry2.put("shooting boolean: ",shooting);
        telemetry2.put("paddle ticks: ", paddle.getPosition());
        telemetry2.put("turnIt? ", turning);
        telemetry2.put("turnToAprilTag? ", turningT);
        telemetry2.put("fly wheel shooting? ", shooting);
//        telemetry.put("colorsensor hsv ",  HSVasString());
//        telemetry.put("what color we got? ", updateColorSensor());

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            telemetry2.put("tx: ", llResult.getTx());
            telemetry2.put("ty: ", llResult.getTy());
            telemetry2.put("ta: ", llResult.getTa());
            telemetry2.put("bot pose heading: ", getPoseHeading());
            telemetry2.put("angle diff: ", angleDif());
        }
//        telemetry.addData("Target IMU Angle", Math.toDegrees(refrenceAngle));
//        telemetry.addData("Current IMU Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//
        telemetry2.put("IMU turn done? ", drivetrain.getIMUTurnDone());
        telemetry2.put("error: ", drivetrain.getErrorPID());
        telemetry2.put("Kp: ", drivetrain.getKpp());
        telemetry2.put("current IMU: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        //telemetry.update();
        //telemetry.put()
        dashboard.sendTelemetryPacket(p);
        return telemetry2;
    }
}
