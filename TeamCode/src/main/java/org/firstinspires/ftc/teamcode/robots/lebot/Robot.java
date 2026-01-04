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

    Pose3D botpose =null;

    static FtcDashboard dashboard = FtcDashboard.getInstance();
    tankDrive drivetrain = new tankDrive();
    private BNO055IMU imu;
    double throttle, spin;

    boolean turning = false;
    boolean turningT = false;
    boolean turningAuto=false;
    double autoAngle=45;
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
    private final int paddleDown = 1000;
    private final int paddleClear=1420;


   public Rev2mDistanceSensor backDist;
    public Rev2mDistanceSensor frontDist;
    public double dist;
    public double distFront=0;
    public double ballNotThere=14;     //change to be whatever dist shows when ball in front of sensor& at paddle

    public int tagIndex=0;

    public int index=0;
    public double minShooterSpeed=935;       //change to speed of flywheel     //1020 for full power
    public ElapsedTime time = new ElapsedTime();
    public double shootTime=.25;      //change to seconds it takes to shoot ball
    public boolean shootingAll=false;

    public boolean channelDistFull = false;
    HardwareMap hardwareMap;
    StickyGamepad g1 = null;
    Gamepad gamepad1;

    public boolean suck=false;

    public Servo tilt;
    public int servoUp=1600;
    public int servoDown=1000;
    public Limelight3A limelight;
    public static boolean allianceRed = true;
    public int pipe=0;

    public double distFromTag=0;
    //TODO: find how far limelight from flywheel
    public double c=.23;        //.27
    public double d=0;
    public double theta=60;
    public double launchSpeed=0;

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

        rightFront = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftRear");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        conveyor = hardwareMap.get(DcMotorEx.class, "conveyor");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        paddle = hardwareMap.get(Servo.class, "paddle");
        //tilt = hardwareMap.get(Servo.class, "tilt");
 //       adjustor = hardwareMap.get(Servo.class, "adjustor");
//
//        frontDist = hardwareMap.get(DistanceSensor.class, "frontDist");
        backDist = hardwareMap.get(Rev2mDistanceSensor.class, "backDist");
        frontDist = hardwareMap.get(Rev2mDistanceSensor.class, "frontDist");

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
        shooter.setVelocity(0);
        resetDrive();
        //tilt.setPosition(servoUp);

        //shootingState = shootingState.RESET;
    }

    public Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad;
    }

    @Override
    public void update(Canvas fieldOverlay) {

        LLResult llResult= limelight.getLatestResult();
        if(llResult!=null && llResult.isValid()){
            botpose=llResult.getBotpose_MT2();
            //distFromTag=getDistFromTag(llResult.getTa());
        }
//        if(tx()){
//            a1=getty();
//            getDistFromTag();
//            getShootingSpeed();
//        }

        if(numBalls<3){
            updateBallCount();
        }
        if(!shootingAll){
            shooting=false;
        }

        updateDistance();
        if(shootingAll){
            shootALLSequence();
            //shootSequence();
        }
        if(shooting){
            shootSequence();
        }
        if(turningAuto){
            drivetrain.turnUntilDegreesIMU(autoAngle,1);
        }

        if(turning){
            //drivetrain.turnUntilDegreesIMU(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle+180,1);
            drivetrain.turnUntilDegreesIMU(0,1);
        }
        if(drivetrain.getIMUTurnDone()){
            turning=false;
            turningAuto=false;
        }
        if(turningT){
            if(tx()){
                drivetrain.turnToTag(0,1);
            }
            //drivetrain.turnUntilDegreesIMU(,1);
        }
        if(drivetrain.gettxTurnDone()){
            turningT=false;
        }
//        else{
//            index=0;
//        }

//        if(turningT){
//            turnToTag();
//        }

    }
    public void setShoot(boolean x){shooting=x;}
    public void setShootAll(boolean x){shootingAll=x;}
    public boolean getShootAll(){return shootingAll;}

    public void shoot(boolean x) {
        if (x)
            shooter.setVelocity(minShooterSpeed,AngleUnit.DEGREES);
        else
            shooter.setVelocity(0);
    }
    public double getMinShooterSpeed(){return minShooterSpeed;}

    public void setShoot(double x){shooter.setVelocity(x,AngleUnit.DEGREES);}
    public void resetShootIndex(){index=0;}

    double llmountAngleDegrees = 29.5;
    /*double llLenseHeightInches = 12;
    double goalHeightInches = 29.8;*/

    double llLenseHeightMeters = 12*2.54/100;
    double goalHeightMeters = 29.8*2.54/100;

    public double getDist(){
        double targetOffsetAngle_Vertical = getty();
        double angleTOGoalRadians = Math.toRadians(llmountAngleDegrees + targetOffsetAngle_Vertical);
        return ((goalHeightMeters - llLenseHeightMeters)/Math.tan(angleTOGoalRadians));
    }
    public double getBackDist(){return dist;}

    public void updateBallCount(){
        //if(numBalls<3){
            if(distFront < 8.7){
                numBalls++;
            }
        //}

    }
    public int indexAll=0;
//    public void shootALLSequence(){
//        switch(indexAll){
//            case 0:
//                resetShootIndex();
//                shooting=true;
//
//                indexAll++;
//                break;
//            case 1:
//                if(!shooting){
//                    indexAll++;
//                    time.reset();
//                }
//                break;
//            case 2:
//                if(time.seconds()>.25) {
////                    resetShootIndex();
////                    shooting=true;
////                    //intakeOn();
////                    time.reset();
//                    indexAll++;
//                }
//                break;
//            case 3:
//                //if(time.seconds()>.3){
//                    indexAll++;
//                //}
//                break;
//            case 4:
////                if(dist < ballNotThere-1) {
////                    indexAll=10;
////                }
//                indexAll++;
//                break;
//            case 5:
//                resetShootIndex();
//                shooting=true;
//                indexAll++;
//                break;
//            case 6:
//                if(!shooting){
//                    time.reset();
//                    indexAll++;
//                }
//                break;
//            case 7:
//                if(time.seconds()>.25) {
////                    //intakeOn();
////                    conveyor.setPower(1);
////                    intake.setPower(1);
////                    //suck=true;
////                    time.reset();
//                    indexAll++;
//                }
//                break;
//            case 8:
//                //if(time.seconds()>.3){
//                    indexAll++;
//               // }
//                break;
//            case 9:
////                if(dist < ballNotThere-1) {
////                    indexAll=10;
////                }
//                indexAll++;
//                break;
//            case 10:
//                resetShootIndex();
//                shooting=true;
//                time.reset();
//                indexAll++;
//                break;
//            case 11:
//                shootingAll=false;
//                conveyor.setPower(0);
//                intake.setPower(0);
//                //suck=false;
//                indexAll=0;
//                intakeOff();
//                break;
//
//        }
//    }
    public void shootALLSequence(){
        switch(indexAll){
            case 0:         //call shoot 1 ball sequence
                resetShootIndex();
                shooting=true;

                indexAll++;
                break;
            case 1:     //check if 1 ball sequence done
                if(!shooting){
                    indexAll++;
                    time.reset();
                }
                break;
            case 2:     //give time for paddle to go down then turn on intake for next ball
                if(time.seconds()>.25) {
                    intakeOn();
//                    conveyor.setPower(1);
//                    intake.setPower(1);
                    time.reset();
                    indexAll++;
                }
                break;
            case 3:     //keep intake on this long for ball to move back to paddle
                if(time.seconds()>3){
                    indexAll=5;
                }
                break;
            case 4:     //check if a ball is there otherwise end sequence early
                if(dist < (ballNotThere-1)) {
                    indexAll=10;
                }
                indexAll++;
                break;
            case 5: //start 2nd ball, call shoot 1 ball sequence
                resetShootIndex();
                shooting=true;
                indexAll++;
                break;
            case 6: //check if 1 ball sequence done
                if(!shooting){
                    time.reset();
                    indexAll++;
                }
                break;
            case 7:     //give time for paddle to go down then turn on intake for next ball
                if(time.seconds()>.25) {
                    intakeOn();
//                    conveyor.setPower(1);
//                    intake.setPower(1);
                    time.reset();
                    indexAll++;
                }
                break;
            case 8:     //keep intake on this long for ball to move back to paddle
                if(time.seconds()>3){
                    indexAll++;
                }
                break;
            case 9:     //check if a ball is there otherwise end sequence early
                if(dist < ballNotThere-1) {
                    indexAll=10;
                }
                indexAll++;
                break;
            case 10:
                resetShootIndex();
                shooting=true;
                time.reset();
                indexAll++;
                break;
            case 11:
                shootingAll=false;
                intakeOff();
//                conveyor.setPower(0);
//                intake.setPower(0);
                //suck=false;
                indexAll=0;
                break;

        }
    }

    public void resetIndexAll(){indexAll=0;}


    public void shootSequence(){
        //if(tagCenteringSequene()) {
            switch (index) {
                case 0:     //ignore
                    //turningT=true;
                    index++;
                    break;
                case 1:     //start flywheel and start intake
                    //if(drivetrain.gettxTurnDone()){
                    minShooterSpeed=getShootingSpeed()-50;      //-60
                    shooter.setVelocity(minShooterSpeed,AngleUnit.DEGREES);

                    //shooter.setVelocity(getShootingSpeed(),AngleUnit.DEGREES);
                    //suck=true;
                    conveyor.setPower(1);
                    intake.setPower(1);
                    index++;
                    //}
                    break;
                case 2:     //check if ball is there to shoot then turn off intake
                    if (dist < 12) {
                       // suck=false;
                        conveyor.setPower(0);
                        intake.setPower(0);
                        time.reset();
                        index++;
                    }
                    break;
                case 3:     //skip
                    //if(time.seconds()>.01){
                        index++;
                    //}
                    break;
                case 4:     //check if flywheel up to speed then lift paddle to shoot
                    if (shooter.getVelocity(AngleUnit.DEGREES) >= minShooterSpeed-8) { //&& shooter.getVelocity(AngleUnit.DEGREES)<=minShooterSpeed+10
                        numBalls--;
                        setPaddleClear();
                        time.reset();
                        index++;
                    }
                    break;
                case 5: //give paddle time to go up all the way then go back down and set shooting false
                    if (time.seconds()>shootTime+.25) {
                        setPaddleDown();
                        //turningT=false;
                        shooting=false;
                        index=0;
                    }
                    break;
            }
    }

    public void setMinShootSpeed(double x){
        minShooterSpeed=x;
    }
    public double getDistFromTag(){
        return distFromTag;
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


    public void setPaddleUp(){
        paddle.setPosition(servoNormalize(paddleUp));
    }
    public void setPaddleDown(){
        paddle.setPosition(servoNormalize(paddleDown));
    }
    public void setPaddleClear(){
        paddle.setPosition(servoNormalize(paddleClear));
    }
//    public void setServoUp(){
//        tilt.setPosition(servoUp);
//    }
//    public void setServoDown(){
//        tilt.setPosition(servoDown);
//    }

    public void setIntakeSpeed(double x){
        conveyor.setPower(x);
        intake.setPower(x);
    }

    public void setAutoAngle(double x){
        autoAngle=x;
    }
    public boolean getTurningAuto(){return turningAuto;}
    public void setTurningAuto(boolean x){
        turningAuto=x;
    }
//    public double getPoseHeading() {
//        double headingRadians = getBotPose().getOrientation().getYaw();
//        return Math.toRadians(headingRadians);
//    }

//    public Pose3D getBotPose() {
//        LLResult llResult = limelight.getLatestResult();
//        if (llResult != null && llResult.isValid()) {
//            return llResult.getBotpose();
//        }
//        return null;
//    }


    public double getShootingSpeed(){
        launchSpeed=(((distFromTag+c))*(1090))/(6.67*(Math.cos(Math.toRadians(theta)))*Math.sqrt(((Math.tan(Math.toRadians(theta))*((distFromTag+c)))-.711)/(4.905)))+105;      //120
        //launchSpeed=(((distFromTag+c)/100)*(1090))/(6.67*(Math.cos(Math.toRadians(theta)))*Math.sqrt(((Math.tan(Math.toRadians(theta))*((distFromTag+c)/100))-.711)/(4.905)))*1.25;
        return launchSpeed;
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
    public double getty() {
        return limelight.getLatestResult().getTy();
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
        drivetrain.setLimelight(x);
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
        distFront=frontDist.getDistance(DistanceUnit.CM);
        //distFromTag=frontDist.getDistance(DistanceUnit.CM);
        if(tx()){
            distFromTag=getDist();
        }

    }
    //public void getB
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

    public void resetDrive(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double getDriveEncoder(){
        return leftFront.getCurrentPosition();
    }

    @Override
    public String getTelemetryName() {
        return "lebot robot";
    }
    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry2 = new LinkedHashMap<>();
        TelemetryPacket p = new TelemetryPacket();
        telemetry2.put("numballs: ",numBalls);
        telemetry2.put("drive odometry right: ", rightFront.getCurrentPosition());
        telemetry2.put("drive odometry left: ", leftFront.getCurrentPosition());
        telemetry2.put("andas calculations speed: ", launchSpeed);
        telemetry2.put("min velocity: ", minShooterSpeed);
        telemetry2.put("distance to goal: ", distFromTag);
        telemetry2.put("Red Alliance??? ", allianceRed);
        telemetry2.put("shooter speed: ",shooter.getVelocity(AngleUnit.DEGREES));
        telemetry2.put("back distance sensor: ",dist);
        telemetry2.put("shooting state ", index);
        telemetry2.put("shooting ALL state: ", indexAll);
        telemetry2.put("shootingAll boolean: ", shootingAll);
        telemetry2.put("shooting boolean: ",shooting);
        telemetry2.put("paddle ticks: ", paddle.getPosition());
        telemetry2.put("turnIt? ", turning);
        telemetry2.put("turnToAprilTag? ", turningT);
//        telemetry.put("colorsensor hsv ",  HSVasString());
//        telemetry.put("what color we got? ", updateColorSensor());

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose=llResult.getBotpose_MT2();
            telemetry2.put(" dist from april tag for anda calculation ", (distFromTag+c));

            telemetry2.put(" dist from april tag (inches) variable: ", distFromTag);
            telemetry2.put("tx: ", llResult.getTx());
            telemetry2.put("ty: ", llResult.getTy());
            telemetry2.put("ta: ", llResult.getTa());
            //telemetry2.put("Botpose, ", botPose.toString());
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
