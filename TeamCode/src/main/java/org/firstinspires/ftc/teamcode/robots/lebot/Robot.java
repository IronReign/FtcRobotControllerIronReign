package org.firstinspires.ftc.teamcode.robots.lebot;

import static org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils.wrapAngle;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
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

//    public NormalizedColorSensor colorSensor = null;
//    public static int colorSensorGain = 12;
//
//    public enum CurrentSample {
//        RED, BLUE, NEUTRAL, NO_SAMPLE
//    }


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

//    private enum shootingState {
//        IDLE, SPIN, FEED, FIRE, RESET
//    }
    public String shootingCase="";

    public boolean shooting = false;
    //private shootingState shootingState;

    public boolean sucking=false;
    /*
    Gamepad Controls:
    - Button to adjust limelight servo
    - Button for fireBall()
    - PID turning
     */

    private final double intakeSpeed = 1.0;
    private final double conveyorSpeed = 1.0;
    private final double conveyorFeedingSpeed = 0.35;
    private final double shooterPower = 2;



    private int numBalls = 0;
    private final int maxBalls = 3;
    private final double ballThreshold = 2; //TODO: inches adjust
    private boolean frontBlocked = false;
    private boolean backBlocked = false;
    private boolean wasFrontBlocked = false;
    private boolean wasBackBlocked = false;

    private long timer2;

    double turninput=0;
    boolean turningR=false;
    boolean turningL=false;
    //public StaticHeading turn=new StaticHeading();


    public void init() {

        /*rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //TESTING^^^^
*/
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
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
        //conveyor.setDirection(DcMotorSimple.Direction.REVERSE);

        paddle = hardwareMap.get(Servo.class, "paddle");
        tilt = hardwareMap.get(Servo.class, "tilt");
 //       adjustor = hardwareMap.get(Servo.class, "adjustor");
//
//        frontDist = hardwareMap.get(DistanceSensor.class, "frontDist");
        backDist = hardwareMap.get(Rev2mDistanceSensor.class, "backDist");
        //colorSensor = this.hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (allianceRed) {
            limelight.pipelineSwitch(1);
        } else {
            limelight.pipelineSwitch(0);
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
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad;
    }

    @Override
    public void update(Canvas fieldOverlay) {

        updateDistance();
        if(shooting){
            shootSequence();
        }

        if(turningT){
            turnToTag();
        }
//        if(sucking){
//            intakeOn();
//        }else{
//            intakeOff();
//        }
//
//        if (shooting) {
//            shooter.setPower(1);
//        } else {
//            shooter.setPower(0);
//        }

//        if(shootingState== shootingState.IDLE){
//            shootingCase="idle";
//        }
//        if(shootingState== shootingState.FEED){
//            shootingCase="feed";
//        }
//        if(shootingState== shootingState.FIRE){
//            shootingCase="fire";
//        }
//        if(shootingState== shootingState.RESET){
//            shootingCase="reset";
//        }
//        if(shootingState== shootingState.SPIN){
//            shootingCase="spin";
//        }

    }
    public void setShoot(boolean x){shooting=x;}
    //public boolean getShoot(){return shooting;}

    public void shoot(boolean x) {
        if (x)
            //shooting=x;
            shooter.setVelocity(minShooterSpeed,AngleUnit.DEGREES);
            //shooter.setPower(1);
        else
            shooter.setPower(0);
    }
    public void resetShootIndex(){index=0;}


    public void shootSequence(){
        //resetTagIndex();
        //if(tagCenteringSequene()) {
            switch (index) {
                case 0:
                    //shoot(true);
                    shooter.setVelocity(minShooterSpeed,AngleUnit.DEGREES);
                    conveyor.setPower(1);
                    intake.setPower(1);
                    //littlePush(true);
                    index++;
                    break;
                case 1:
                    if (dist < 9) {
                        conveyor.setPower(0);
                        intake.setPower(0);
                        //littlePush(false);
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
                    //double x= shooter.getVelocity()
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
//                case 4:
//                    if (time.seconds() > shootTime) {
//                        shooter.setPower(0);
//                        index++;
//                    }
//                    break;

            }
        //}
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







    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();
        TelemetryPacket p = new TelemetryPacket();
        telemetry.put("Red Alliance??? ", allianceRed);
        telemetry.put("shooter speed: ",shooter.getVelocity(AngleUnit.DEGREES));
        telemetry.put("back distance sensor: ",dist);
        telemetry.put("shooting state ", index);
        telemetry.put("shooting boolean: ",shooting);
        telemetry.put("paddle ticks: ", paddle.getPosition());
        telemetry.put("turnIt? ", turning);
        telemetry.put("turnToAprilTag? ", turningT);
        telemetry.put("fly wheel shooting? ", shooting);
//        telemetry.put("colorsensor hsv ",  HSVasString());
//        telemetry.put("what color we got? ", updateColorSensor());

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            telemetry.put("tx: ", llResult.getTx());
            telemetry.put("ty: ", llResult.getTy());
            telemetry.put("ta: ", llResult.getTa());
            telemetry.put("bot pose heading: ", getPoseHeading());
            telemetry.put("angle diff: ", angleDif());
        }


        telemetry.put("current IMU: ", getCurrentIMU());

        //telemetry.put()
        dashboard.sendTelemetryPacket(p);
        return telemetry;
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
    public void littlePush(boolean x){
        if(x){
            conveyor.setPower(.8);
            intake.setPower(0);
        }
        else{
            conveyor.setPower(0);
            intake.setPower(0);
        }

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

    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

//    public void calculateTurn(){
//        double rightx = gamepad1.right_stick_x;
//        double righty = gamepad1.right_stick_y;
//
//        if (Math.hypot(rightx, righty)>0.15) {
//            refrenceAngle = Math.atan2(-rightx, -righty);
//            turning = true;
//            integralSum = 0;
//            lastError = 0;
//            timer.reset();
//        }
//    }

    public void turnIt() {

        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        double error = angleWrap(refrenceAngle - currentAngle);

        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);

        drivetrain.power(output);

        if (Math.abs(error) < Math.toRadians(1.5)) {
            drivetrain.power(0);
            turning = false;
        }
    }

    public void turnItShoot() {

        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        refrenceAngle = currentAngle + gettx();
        double error = angleWrap(refrenceAngle - currentAngle);

        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);

        drivetrain.power(output);

        if (Math.abs(error) < Math.toRadians(1.5)) {
            drivetrain.power(0);
            turningT = false;
        }
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



    public void turnToTag() {
        integralSum = 0;
        lastError = 0;
        double currentAngle = Math.toRadians(gettx());
        double error = angleWrap(0 - currentAngle);

        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();

        double output = (error * (Kp)) + (derivative * Kd) + (integralSum * Ki);

        drivetrain.power(output);

        if (Math.abs(error) < Math.toRadians(2)) {
            drivetrain.power(0);
            turningT = false;
//            long timer=futureTime(.5);
//            if(isPast(timer)){
//                if(Math.abs(error) < Math.toRadians(3)){
//                    drivetrain.power(0);
//                    turningT = false;
//                }
//            }

        }
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
        limelight.start();
    }


    public void intakeOn() {
        conveyor.setPower(1);
        intake.setPower(1);

//        if (!channelFull()){
//            intake.setPower(intakeSpeed);
//            conveyor.setPower(conveyorSpeed);
//        } else {
//            intakeOff();
//        }
    }

    public void intakeOff(){
        intake.setPower(0);
        conveyor.setPower(0);
    }

    public void updateDistance(){
        dist=backDist.getDistance(DistanceUnit.CM);
    }


//    public void countBalls(){
//        /* int ballsDetected = 0;
//
//        limelight.pipelineSwitch(0); // green
//        LLResult greenResult = limelight.getLatestResult();
//        if (greenResult.getColorResults() != null){
//            ballsDetected += greenResult.getColorResults().size();
//        }
//
//        limelight.pipelineSwitch(1); // purple
//        LLResult purpleResult = limelight.getLatestResult();
//        if (purpleResult.getColorResults() != null){
//            ballsDetected += purpleResult.getColorResults().size();
//        }
//
//        int newBalls = ballsDetected - lastBallCount;
//
//        if (newBalls > 0){
//            numBalls = numBalls + newBalls;
//        }
//
//        lastBallCount = ballsDetected; */
//
//        frontBlocked = frontDist.getDistance(DistanceUnit.INCH) < ballThreshold;
//        backBlocked = backDist.getDistance(DistanceUnit.INCH) < ballThreshold;
//
//        if(frontBlocked && !wasFrontBlocked){
//            if(numBalls<maxBalls){
//                numBalls++;
//            }
//        }
//
//        if(!frontBlocked && wasFrontBlocked){
//            if(numBalls>0){
//                numBalls--;
//            }
//        }
//
//        if(backBlocked && !wasBackBlocked){
//            if(numBalls<maxBalls){
//                numBalls++;
//            }
//        }
//
//        if(!backBlocked && wasBackBlocked){
//            if(numBalls>0){
//                numBalls--;
//            }
//        }
//
//        wasFrontBlocked = frontBlocked;
//        wasBackBlocked = backBlocked;
//
//    }
//
//    public boolean channelFull() {
//        return (numBalls == maxBalls);
//    }
    public void setChannelDistFull(boolean x){
        channelDistFull=x;
    }

//    public String updateColorSensor() {
//        double hue = getHSV()[0];
//        if (hue < 35 && hue > 20) {     //90    70
//            currentSample = org.firstinspires.ftc.teamcode.robots.giant.Robot.CurrentSample.NEUTRAL;
//            return "NEUTRAL";
//        } else if (hue < 360 && hue > 350) {        //60    20
//            currentSample = org.firstinspires.ftc.teamcode.robots.giant.Robot.CurrentSample.RED;
//            return "RED";
//        } else if (hue < 220 && hue > 210) {
//            currentSample = org.firstinspires.ftc.teamcode.robots.giant.Robot.CurrentSample.BLUE;
//            return "BLUE";
//        } else {
//            currentSample = org.firstinspires.ftc.teamcode.robots.giant.Robot.CurrentSample.NO_SAMPLE;
//            return "NO SAMPLE";
//        }
//    }
//
//
//    public String HSVasString () {
//        float[] hsv = getHSV();
//        return hsv[0] + " " + hsv[1] + " " + hsv[2];
//    }
//    public float[] getHSV() {
//        float[] hsv = new float[3];
//        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);
//        return hsv;
//    }

    public static double servoNormalize(int pulse) {
        double normalized = (double) pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }
}
