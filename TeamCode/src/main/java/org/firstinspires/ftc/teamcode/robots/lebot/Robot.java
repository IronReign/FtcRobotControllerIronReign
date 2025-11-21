package org.firstinspires.ftc.teamcode.robots.lebot;

import static org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils.wrapAngle;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    double refrenceAngle=Math.toRadians(45);
    double integralSum = 0;
    private double lastError = 0;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;

    ElapsedTime timer = new ElapsedTime();


    public DcMotor leftFront, rightFront;
    public DcMotor intake, conveyor, shooter;
    public Servo paddle, adjustor;
    public DistanceSensor frontDist, backDist;
    HardwareMap hardwareMap;
    StickyGamepad g1 = null;
    Gamepad gamepad1;

    public Limelight3A limelight;
    public boolean allianceRed=true;

    private enum shootingState {
        IDLE, SPIN, FEED, FIRE, RESET
    }
    public static boolean shooting=false;
    private shootingState shootingState;

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

    private final double paddleUp = 0.8;
    private final double paddleDown = 0.2;

    private int numBalls = 0;
    private final int maxBalls = 3;
    private final double ballThreshold = 2; //TODO: inches adjust
    private boolean frontBlocked = false;
    private boolean backBlocked = false;
    private boolean wasFrontBlocked = false;
    private boolean wasBackBlocked = false;

    //private long timer;

    //public StaticHeading turn=new StaticHeading();


    public void init (HardwareMap hardwareMap){

        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //TESTING^^^^

        g1 = new StickyGamepad(gamepad1);

        drivetrain.init(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);


        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");

//        intake = hardwareMap.get(DcMotor.class, "intake");
//        conveyor = hardwareMap.get(DcMotor.class, "conveyor");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        paddle = hardwareMap.get(Servo.class, "paddle");
//        adjustor = hardwareMap.get(Servo.class, "adjustor");
//
//        frontDist = hardwareMap.get(DistanceSensor.class, "frontDist");
//        backDist = hardwareMap.get(DistanceSensor.class, "backDist");

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if(allianceRed){
            limelight.pipelineSwitch(1);
        }else{
            limelight.pipelineSwitch(0);
        }         //pipeline 6 reads blue(20) and red(24)
        limelight.start();

        //closedChannel();

//        intake.setPower(0);
//        conveyor.setPower(0);
        shooter.setPower(0);

//        shootingState = shootingState.RESET;
    }

    public Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad;
    }

    @Override
    public void update(Canvas fieldOverlay) {
        if(shooting){
            shooter.setPower(1);
        }else{
            shooter.setPower(0);
        }


    }

    public void shoot(boolean x){
        if(x)
            shooter.setPower(1);
        else
            shooter.setPower(0);
    }




    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();
        TelemetryPacket p = new TelemetryPacket();
        telemetry.put("Red Alliance??? ",allianceRed);
        telemetry.put("turnIt? ",turning);
        telemetry.put("turnToAprilTag? ",turningT);
        LLResult llResult=limelight.getLatestResult();
        if(llResult!=null && llResult.isValid()){
            telemetry.put("tx: ",llResult.getTx());
            telemetry.put("ty: ",llResult.getTy());
            telemetry.put("ta: ",llResult.getTa());
            telemetry.put("bot pose heading: ", getPoseHeading());
            telemetry.put("angle diff: ",angleDif());
        }


        telemetry.put("current IMU: ",getCurrentIMU());

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

    public double angleDif(){
        return getPoseHeading() - getCurrentIMU();
    }

    public double getPoseHeading(){
        double headingRadians=getBotPose().getOrientation().getYaw();
        return Math.toRadians(headingRadians);
    }

    public Pose3D getBotPose(){
        LLResult llResult=limelight.getLatestResult();
        if(llResult!=null && llResult.isValid()) {
            return llResult.getBotpose();
        }
        return null;
    }

    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
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

    public void turnIt(){

        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        double error = angleWrap(refrenceAngle - currentAngle);

        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);

        drivetrain.power(output);

        if (Math.abs(error) < Math.toRadians(1.5)){
            drivetrain.power(0);
            turning = false;
        }
    }
    public void turnItShoot(){

        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        refrenceAngle=currentAngle+gettx();
        double error = angleWrap(refrenceAngle - currentAngle);

        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);

        drivetrain.power(output);

        if (Math.abs(error) < Math.toRadians(1.5)){
            drivetrain.power(0);
            turningT = false;
        }
    }

    public boolean tx(){
        if(limelight.getLatestResult()!=null && limelight.getLatestResult().isValid()) {
            return true;
        }
        return false;
    }
    public double gettx(){
        return limelight.getLatestResult().getTx();
    }

    public void turnToTag(){
        integralSum=0;
        lastError=0;
        double currentAngle = Math.toRadians(gettx());
        double error = angleWrap(0 - currentAngle);

        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();

        double output = (error * (Kp+.55)) + (derivative * Kd) + (integralSum * Ki);

        drivetrain.power(output);

        if (Math.abs(error) < Math.toRadians(4)){
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

    public double getCurrentIMU(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    public double getRefrenceAngle(){
        return refrenceAngle;
    }

    public void setTurning(boolean x){
        turning=x;
    }
    public void setTurningT(boolean x){
        turningT=x;
    }
    public boolean getTurning(){return turning;}
    public boolean getTurningT(){return turningT;}
    public void setDrivetrain(double t,double s){
        drivetrain.drive(t,s);
    }

    public void setRedALliance(boolean x){
        allianceRed=x;
    }
    public boolean getRedALliance(){return allianceRed;}
    public void switchPipeline(int x){
        limelight.pipelineSwitch(x);        //0 is blue 1 is red
        limelight.start();
    }



//    public void intakeOn() {
//        if (!channelFull()){
//            intake.setPower(intakeSpeed);
//            conveyor.setPower(conveyorSpeed);
//        } else {
//            intakeOff();
//        }
//    }
//
//    public void intakeOff(){
//        intake.setPower(0);
//        conveyor.setPower(0);
//    }
//
//    public void openChannel(){
//        paddle.setPosition(paddleUp);
//    }
//
//    public void closedChannel(){
//        paddle.setPosition(paddleDown);
//    }
//
//    public void feedPaddle(){
//        conveyor.setPower(conveyorFeedingSpeed);
//        intake.setPower(0);
//    }

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

//    public void fireBall(){
//        countBalls();
//
//        switch (shootingState){
//            case RESET:
//                closedChannel();
//                intakeOn();
//                if (channelFull()){
//                    shootingState = shootingState.IDLE;
//                }
//                break;
//
//            case IDLE:
//                intakeOff();
//                shooter.setPower(shooterPower);
//                timer = futureTime(5);
//                shootingState = shootingState.SPIN;
//                break;
//
//            case SPIN:
//                if(isPast(timer)){
//                    feedPaddle();
//                    shootingState = shootingState.FEED;
//                }
//                break;
//
//            case FEED:
//                openChannel();
//                conveyor.setPower(0);
//                timer = futureTime(3);
//                shootingState = shootingState.FIRE;
//                break;
//
//            case FIRE:
//                if (isPast(timer)){
//                    closedChannel();
//                    if(numBalls == 0){
//                        shooter.setPower(0);
//                        shootingState = shootingState.RESET;
//                    } else {
//                        shootingState = shootingState.SPIN;
//                    }
//                }
//                break;
//        }
//    }
}