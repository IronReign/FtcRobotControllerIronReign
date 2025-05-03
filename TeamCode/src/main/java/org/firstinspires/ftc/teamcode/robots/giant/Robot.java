package org.firstinspires.ftc.teamcode.robots.giant;


import static org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils.wrapAngle;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;


import android.graphics.Color;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;


import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;




//public class Robot implements Subsystem {
//    static FtcDashboard dashboard = FtcDashboard.getInstance();
//
//    HardwareMap hardwareMap;
//    DcMotorEx leftFront, leftBack, rightFront, rightBack;
//    DcMotorEx shoulder,arm;
//
//    public NormalizedColorSensor colorSensor = null;
//    public static int colorSensorGain = 12;
//    public enum CurrentSample {
//        RED, BLUE, NEUTRAL, NO_SAMPLE
//    }
//    public Robot.CurrentSample currentSample = Robot.CurrentSample.NO_SAMPLE;
//    public List<Robot.CurrentSample> targetSamples = new ArrayList<>();
//
//    int rotate=0;
//    int extend=0;
//    double targetF, targetS, targetT;
//    boolean STOP=false;
//   // VisionProvider camera;
//    //red 110-140
//
//    Servo claw;
//    StickyGamepad g1=null;
//    Gamepad gamepad1;
//    public static final double objectWidthInRealWorldUnits= 3.4;
//    public static final double focalLength=1430;
//    double forward = 0;
//    double strafe = 0;
//    double turn = 0;
//    int clawTicks=0;
//    //public boolean clawOpen = false;
//
//    public Robot(HardwareMap hardwareMap, Gamepad gamepad) {
//        this.hardwareMap = hardwareMap;
//        this.gamepad1 = gamepad;
//    }
//
//    @Override
//    public void update(Canvas fieldOverlay) {
//        g1.update();
//
//        //camera.update(true);
//        shoulder.setTargetPosition(rotate);
//        claw.setPosition(servoNormalize(clawTicks));
//        arm.setTargetPosition(extend);
//
//        updateColorSensor();
//        colorSensor.setGain(colorSensorGain);
//
//        mecanumDrive(targetF,targetS,targetT);
//    }
//
//    public void mecanumDrive(double forwardX, double strafeX, double turnX) {
//        forward = forwardX;
//        strafe =  strafeX;
//        turn = turnX*.65;
//        double r = Math.hypot(strafe, forward);
//        double robotAngle = Math.atan2(forward, strafe) - Math.PI/4;
//        double rightX = -turn;
//        leftFront.setPower((r * Math.cos(robotAngle) - rightX));
//        rightFront.setPower((r * Math.sin(robotAngle) + rightX));
//        leftBack.setPower((r * Math.sin(robotAngle) - rightX));
//        rightBack.setPower((r * Math.cos(robotAngle) + rightX));
//    }
//
//    public void setDrive(double forwardX, double strafeX, double turnX) {
//        targetF=forwardX;
//        targetT=turnX;
//        targetS=strafeX;
//    }
//    //open and close claw
//    public void claw(int x){
//        clawTicks=x;
//    }
//
//    public void grabBlock(){clawTicks=500;}
//
//    public void dropBlock(){clawTicks=2000;}
//
//
//    public void extendTop(){extend=9100;}
//
//
//    public void rotateBar(){
//        rotate=1550;
//    }
//    public void reachUp(){
//        extend=5090;
//    }
//    public void pullDown(){
//        extend=1960;
//    }
//
//    //rotate entire arm
//    public void tu(int x){
//        rotate+=x;
//    }
//    //extend linear slide
//    public void extend(int x)
//    {
//        extend+=x;
//    }
//    public void setExtend(int x)
//    {
//        extend=x;
//    }
//    public void setStop(boolean x){
//        STOP=x;
//    }
//
//    public void resetR() {
//        rotate=10;
//    }
//    public void resetE(){
//        extend=2110;
//    }
//
//    public int getRotate() {
//        return shoulder.getCurrentPosition();
//    }
//
//    public void setRotate(int x) {
//        rotate=x;
//    }
//
//    public void unstickArm(){
//        extend=7850;
//    }
//    public int getClaw() {
//        return clawTicks;
//    }
//    public void setShoulderSpeed(double x){
//        shoulder.setPower(x);
//    }
//    public void setArmSpeed(double x){
//        arm.setPower(x);
//    }
//
//
//    public int getExtend()
//    {
//        return arm.getCurrentPosition();
//    }
//
//    public void doit() {
//
//        extend=2110;
//        rotate=1550;
//    }
//    public void attatch(){
//        extend=5;
//    }
//    @Override
//    public void stop() {
//
//    }
//
//    public void init() {
//        g1 = new StickyGamepad(gamepad1);
//
////        try{
////            camera = VisionProviders.VISION_PROVIDERS[6].newInstance();
////        }catch(Exception e) {
////            throw new RuntimeException(e);
////        }
//       // camera.initializeVision(hardwareMap,this,false);
//        arm = hardwareMap.get(DcMotorEx.class, "arm");
//        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
//        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
//        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
//        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
//        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
//        claw = hardwareMap.get(Servo.class, "claw");
//
//        colorSensor = this.hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");
//
//        // Set motor runmodes
//        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        shoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        shoulder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//
//        rightBack.setDirection(DcMotor.Direction.REVERSE);
//        rightFront.setDirection(DcMotor.Direction.REVERSE);
//
//        shoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        shoulder.setTargetPosition(0);
//        shoulder.setPower(1);
//
//        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        arm.setTargetPosition(0);
//        arm.setPower(1);
//
////        imu = hardwareMap.get(BNO055IMU.class, "imu");
////        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
////        parameters.mode = BNO055IMU.SensorMode.IMU;
////        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
////        imu.initialize(parameters);
//    }
//
//    public String updateColorSensor() {
//        double hue = getHSV()[0];
//        if (hue < 90 && hue > 70) {
//            currentSample = Robot.CurrentSample.NEUTRAL;
//            return "NEUTRAL";
//        } else if (hue < 60 && hue > 20) {
//            currentSample = Robot.CurrentSample.RED;
//            return "RED";
//        } else if (hue < 250 && hue > 200) {
//            currentSample = Robot.CurrentSample.BLUE;
//            return "BLUE";
//        } else {
//            currentSample = Robot.CurrentSample.NO_SAMPLE;
//            return "NO SAMPLE";
//        }
//    }
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
//
//
//    public static double servoNormalize(int pulse) {
//        double normalized = (double) pulse;
//        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
//    }
//
//    @Override
//    public Map<String, Object> getTelemetry(boolean debug) {
//        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();
//        TelemetryPacket p = new TelemetryPacket();
//        telemetry.put("FORWARD: ", targetF);
//        telemetry.put("TURN: ", targetT);
//        telemetry.put("STRAFE: ", targetS);
//        telemetry.put("shoudler guess: " , rotate);
//        telemetry.put("shoulder actual: " , shoulder.getCurrentPosition());
//
//        telemetry.put("claw guess: ", clawTicks);
//        telemetry.put("claw actual:", claw.getPosition());
//
//        telemetry.put("arm guess: ", extend);
//        telemetry.put("actual extend: ", arm.getCurrentPosition());
//
////        telemetry.put("autonindex: ",autonIndex);
////        telemetry.put("stop???   ",STOP);
//
//        telemetry.put("colorsensor hsv", "" + HSVasString());
////        telemetry.put("Current IMU Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
////        telemetry.put("vision class", camera.getClass());
////        telemetry.put("vision", camera.getTelemetryName());
//        dashboard.sendTelemetryPacket(p);
//        return telemetry;
//    }
//
//    @Override
//    public String getTelemetryName() {
//        return "Giant Robot";
//    }
//
//}

/////NEW STUFF
public class Robot implements Subsystem {
    static FtcDashboard dashboard = FtcDashboard.getInstance();
    HardwareMap hardwareMap;

    private double averageLoopTime;
    long loopClockTime = System.nanoTime();
    long lastLoopClockTime;
    public static long totalRunTime;
    long startTime;

    boolean calibrate=false;


    //0:leftback
    //1:leftfront
    //2:rightfront

    //2250-750 for shoulder servo

    public NormalizedColorSensor colorSensor = null;
    public static int colorSensorGain = 12;
//    VisionProvider camera;

    public DistanceSensor sensorDistance;
    public DistanceSensor sensorDistSide;
    //public DistanceSensor dist2;

    public double distSide=0;
   // public double disttwo=0;
    public double dist=0;


    public enum CurrentSample {
        RED, BLUE, NEUTRAL, NO_SAMPLE
    }
    public Robot.CurrentSample currentSample = Robot.CurrentSample.NO_SAMPLE;
    public List<Robot.CurrentSample> targetSamples = new ArrayList<>();
    public Robot.CurrentSample opp;
    public String op;

    DcMotorEx leftFront, leftBack, rightFront, rightBack;
    DcMotorEx outExtend, upExtend2, upExtend1;
    DcMotorEx suck;

    public static final double objectWidthInRealWorldUnits= 3.4;
    public static final double focalLength=1430;

    Servo tilt;
    Servo claw;
    Servo shoulder;

    boolean sucking=false;
    boolean eject=false;
    boolean slurp=false;

    StickyGamepad g1=null;
    StickyGamepad g2=null;
    Gamepad gamepad1;
    Gamepad gamepad2;

    double vertical=0;
    double horizontal=0;

    double forward = 0;
    double strafe = 0;
    double turn = 0;
    double targetF, targetS, targetT;

    int tiltTicks=1200; //800     //780
    int shoulderTicks=750+420;

    int outExtendTicks=0;
    int upExtendTicks=0;

    int clawTicks=1460;
    boolean clawOpen=false;

    boolean hook=true;
    String mode="";

    int transferUpTicks=0;
    int transferShoulderTicks=0;

    static boolean allianceRed=true;

    boolean blockSucked=false;
    String block="";
    String ally="";

    boolean strafesensor = true;

    double strafelim=0;
    double turnlim=0;

    double first=0;
    double second=0;
    double third=0;

    double correct=0;

    boolean thisTurn=false;
    boolean turning=false;


//shoulder grab intake ticks 870
    //oscilate value .129       .0645
    public static double pval=0.027;    //.027   //shake&oscillate .037    //.025
    public static PIDController headingPID;
    public static PIDCoefficients HEADING_PID_PWR = new PIDCoefficients(pval, 0.12, 0.00);      //.012  //i: .015
    public static double HEADING_PID_TOLERANCE = .21;           //.21 //this is a percentage of the input range .063 of 2PI is 1 degree
    private double PIDCorrection, PIDError, targetHeading, targetDistance;
    boolean imuTurnDone = false;
//penis
    public static PIDController distancePID;
    public static PIDCoefficients DISTANCE_PID_PWR = new PIDCoefficients(0.023, 0.01, 0);       //p=.023
    public static double DISTANCE_PID_TOLERANCE = 2; //this is a percentage of the input range .063 of 2PI is 1 degree
    private double distPIDCorrection, distPIDError;
    boolean distDriveDone;

    BNO055IMU imu;

    public Robot(HardwareMap hardwareMap, Gamepad gamepad, Gamepad gamepad2two) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad;
        this.gamepad2 = gamepad2two;
        targetSamples.add(Robot.CurrentSample.NEUTRAL);
        if(targetSamples.contains(Robot.CurrentSample.RED)){
            opp=Robot.CurrentSample.BLUE;
            op="BLUE";
            ally="RED";
        }
        else{
            opp=Robot.CurrentSample.RED;
            op="RED";
            ally="BLUE";
        }

        if(block.equals(ally)) {
            blockSucked = true;
        }else{
            blockSucked=false;
        }

        if(hook){
            turnlim=1;
            strafelim=1;
        }else{
            turnlim=.7;
            strafelim=.5;
        }

        // init PID
        headingPID = new PIDController(HEADING_PID_PWR);
        headingPID.setInputRange(0, 360);
        headingPID.setOutputRange(-1, 1);
        headingPID.setIntegralCutIn(4);
        headingPID.setContinuous(true);
        headingPID.setTolerance(HEADING_PID_TOLERANCE);
        headingPID.enable();

        distancePID = new PIDController(DISTANCE_PID_PWR);
        distancePID.setInputRange(0, 200); //2 meter distance sensor
        distancePID.setOutputRange(-1, 1);
        distancePID.setIntegralCutIn(3);
        distancePID.setContinuous(false);
        distancePID.setTolerance(DISTANCE_PID_TOLERANCE);
        distancePID.enable();
        totalRunTime=0;
        startTime = System.currentTimeMillis();
    }

    @Override
    public void update(Canvas fieldOverlay) {
        lastLoopClockTime = System.nanoTime();
        totalRunTime = (System.currentTimeMillis() - startTime) / 1000;
       // HEADING_PID_PWR = new PIDCoefficients(pval, 0, 0);

        if(allianceRed){
            targetSamples.add(Robot.CurrentSample.RED);
            targetSamples.remove(Robot.CurrentSample.BLUE);
        }else{
            targetSamples.add(Robot.CurrentSample.BLUE);
            targetSamples.remove(Robot.CurrentSample.RED);
        }

        dist=sensorDistance.getDistance(DistanceUnit.CM);
        distSide=sensorDistSide.getDistance(DistanceUnit.CM);

//        camera.update(true);
        colorSensor.setGain(colorSensorGain);
        g1.update();
        g2.update();

//        updateColorSensor();
//        colorSensor.setGain(colorSensorGain);
        mecanumDrive(targetF,targetS,targetT);
        moving();
        if(hook){
            mode="hook";
        }else{
            mode="basket";
        }

        //TEST THIS THING
//        if(turning){
//            if(turnUntilDegreesIMU(whereTurn(),1)){
//                turning=false;
//            }
//        }
        //^^^TEST THIS THING



        claw.setPosition(servoNormalize(clawTicks));
        tilt.setPosition(servoNormalize(tiltTicks));
        shoulder.setPosition(servoNormalize(shoulderTicks));
        upExtend1.setTargetPosition(upExtendTicks);
        outExtend.setTargetPosition(outExtendTicks);

        horizontal = leftBack.getCurrentPosition();
        vertical = leftFront.getCurrentPosition();

        if(hook){
            transferShoulderTicks=1610;
           // transferUpTicks= ???         figure out value
        }else{
             transferShoulderTicks= 1850;
             transferUpTicks= 3150;
        }


        if(clawOpen){dropBlock();
        }else{grabBlock();}


        if(!eject&&!slurp) {
            if (sucking) {
                grab();
               // pullBack();
            }
            else{
                suck.setPower(0);
            }
        }else if(eject&&!slurp){
            suck.setPower(-1);
        } else{
            suck.setPower(1);
        }

        imustuff();
    }

    public static void p(double x){        pval+=x;
    }


    public void autonMotors(){
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void imustuff(){
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        first=angles.firstAngle;
        second=angles.secondAngle;
        third=angles.thirdAngle;
    }
    //request a turn in degrees units
    //this is an absolute (non-relative) implementation.
    //the direction of the turn will favor the shortest approach
    public boolean turnUntilDegreesIMU(double turnAngle, double maxSpeed) {
        targetHeading = wrapAngle(turnAngle);
        headingPID.setPID(HEADING_PID_PWR);
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        headingPID.setInput(angles.firstAngle); // todo - is this yaw?
        headingPID.setSetpoint(targetHeading);
        headingPID.setOutputRange(-maxSpeed, maxSpeed);
        headingPID.setTolerance(HEADING_PID_TOLERANCE);
        double correction = headingPID.performPID();

        PIDCorrection = correction;
        PIDError = headingPID.getError();
        correct= headingPID.getError();
        if (headingPID.onTarget()) {
            //turn meets accuracy requirement

            setDrive(0,0,0);
            return imuTurnDone = true;
        } else {
            headingPID.enable();
            setDrive(0,0,-correction);
            return imuTurnDone = false;
        }
       // return true;
    }

    // pid to drive to a target distance sensor value
    public boolean driveDistance(double distance, double maxSpeed) {
        targetDistance = wrapAngle(distance);
        distancePID.setPID(DISTANCE_PID_PWR);
        distancePID.setInput(dist);  // dist gets new value in update()
        distancePID.setSetpoint(targetDistance);
        distancePID.setOutputRange(-maxSpeed, maxSpeed);
        distancePID.setTolerance(DISTANCE_PID_TOLERANCE);
        double correction = distancePID.performPID();
        distPIDCorrection = correction;
        distPIDError = distancePID.getError();
        correct=distancePID.getError();

        if (distancePID.onTarget()) {
            //distance meets accuracy requirement
            setDrive(0,0,0);
            return distDriveDone = true;
        } else {
            distancePID.enable();
            setDrive(-correction,0,0);
            return distDriveDone = false;
        }
    }

    public boolean strafe(double distance, double maxSpeed) {
        targetDistance = wrapAngle(distance);
        distancePID.setPID(DISTANCE_PID_PWR);
        distancePID.setInput(distSide);     //true is left sensor looking at robot from front

         // dist gets new value in update()
        distancePID.setSetpoint(targetDistance);
        distancePID.setOutputRange(-maxSpeed, maxSpeed);
        distancePID.setTolerance(DISTANCE_PID_TOLERANCE);
        double correction = distancePID.performPID();
        distPIDCorrection = correction;
        distPIDError = distancePID.getError();
        correct=distancePID.getError();
        if (distancePID.onTarget()) {
            //distance meets accuracy requirement
            setDrive(0,0,0);
            return distDriveDone = true;
        } else {
            distancePID.enable();
            setDrive(0,-correction,0);

            return distDriveDone = false;
        }
    }


    public void grab(){
        //if find opponent block reverse motor to spit back out in same direction picked up
        if(!eject&&!slurp) {
            if (keepBlock()) {
                upExtendTicks=0;

                long autonTimer = 0;
                autonTimer = futureTime(.02);
                suck.setPower(-1);
                if(isPast(autonTimer)) {
                    suck.setPower(0);
                }

                    if(currentSample.equals(Robot.CurrentSample.NEUTRAL)){
                        //hook=false;
                        outExtend.setPower(.7);
                        yellow();

                    }else{
                        //hook=true;
                        outExtend.setPower(.7);
                        goodBlock();       //place holder till figure out the fuck we doing
                    }
            }
            else {
                suck.setPower(1);       //1
            }
        }else if(eject&&!slurp){
            suck.setPower(-.3);      //-1
        }
        else {
            suck.setPower(1);
        }
    }

    public void yellow(){
        tiltTicks=1240;//1440
        outExtendTicks=50;
        upExtendTicks=30;

    }

    public void goodBlock(){
        tiltTicks=780;//1440
        outExtendTicks=50;     //-25
//        shoulderTicks=1250;
    }

    public void plsnobad(){
        tiltTicks=780;
    }

    public void suck(){
//        outExtendTicks=3300;
        tiltTicks=740;
        sucking = true;

    }

    public void prep(){
//        open();
//        setUpExtend(350);
//        setShoulder(950);     //930
        //setTilt(970);
        outExtendTicks=1050;        //2750;
    }

    public void scooch(){
        upExtendTicks=1950;
    }

    public void dunk(){
        upExtendTicks=3150;
        shoulderTicks=950;      //
    }

    public void pullBack(){
        outExtendTicks-=25;
    }

    public void hookit(){
//        upExtendTicks=1800; //1950
//        shoulderTicks=1510;
        upExtendTicks=2275;      //2280
        //shoulderTicks=1150;
    }

    public void downHook(){
        upExtendTicks=1275;     //1275
    }

    public void wallGrab(){
        open();
        shoulderTicks=1450+420; //1510  1450
        upExtendTicks=154;    //5
    }

    public void setOutPower(double x){
        outExtend.setPower(x);
    }
    public void turnIt(){
        thisTurn=!thisTurn;
        turning=true;
    }


    public int whereTurn(){
        if(thisTurn){return 180;}
        return 0;
    }



    public void suckPower(double x){suck.setPower(x);}
    public void setSuck(boolean x){sucking=x;}
    public void spit(boolean x){eject=x;}
    public void setSlurp(boolean x){slurp=x;}
//    public double getSuck(){
//        return suck.getPower();
//    }

    public void eject(double x){
        suck.setPower(x);
    }

    public boolean getSuck(){
        return sucking;
    }


    public boolean haveBlock(){
        return blockSucked;
    }

    public boolean keepBlock() {
        if(targetSamples.contains(currentSample)){
            sucking=false;
            return true;
        }
        return false;
    }



    public void init() {
        g1 = new StickyGamepad(gamepad1);
        g2 = new StickyGamepad(gamepad2);

//        try{
//            camera = VisionProviders.VISION_PROVIDERS[6].newInstance();
//        }catch(Exception e) {
//            throw new RuntimeException(e);
//        }
//        camera.initializeVision(hardwareMap,this,false);

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorDistance");
        sensorDistSide = hardwareMap.get(DistanceSensor.class, "sensorDistSide");
        //dist2= hardwareMap.get(DistanceSensor.class, "dist2");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        upExtend1 = hardwareMap.get(DcMotorEx.class, "upExtend1");
        upExtend2 = hardwareMap.get(DcMotorEx.class, "upExtend2");
        outExtend = hardwareMap.get(DcMotorEx.class, "outExtend");

        suck = hardwareMap.get(DcMotorEx.class, "suck");

        claw = hardwareMap.get(Servo.class, "claw");
        tilt = hardwareMap.get(Servo.class, "tilt");
        shoulder = hardwareMap.get(Servo.class, "shoulder");

        // Set motor runmodes
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        upExtend1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        upExtend2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        upExtend1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        upExtend2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        upExtend2.setDirection(DcMotor.Direction.REVERSE);
        outExtend.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outExtend.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        outExtend.setDirection(DcMotor.Direction.REVERSE);

        upExtend1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        upExtend1.setTargetPosition(0);
        upExtend1.setPower(1);
        upExtend2.setPower(0);
        outExtend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        outExtend.setTargetPosition(0);
        outExtend.setPower(1);

        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        suck.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        suck.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
         colorSensor = this.hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();
        TelemetryPacket p = new TelemetryPacket();

        telemetry.put("ERROR ", correct);
        telemetry.put("RECALLIBRATE: ",calibrate);

        telemetry.put("GAMEMODE: ", mode);
        telemetry.put("RED ALLIANCE", allianceRed);
        telemetry.put("DISTANCE SENSOR: ", dist);
        telemetry.put("AVERAGE LOOP TIME: " , averageLoopTime);

        telemetry.put("FIRST ANGLE: ", first);
//        telemetry.put("SECOND ANGLE: ", second);
//        telemetry.put("THIRD ANGLE: ", third);


        telemetry.put("Horizontal", leftBack.getCurrentPosition());
        telemetry.put("Vertical", leftFront.getCurrentPosition());

        telemetry.put("out extend", outExtendTicks);
        telemetry.put("out extend GET POSITION", outExtend.getCurrentPosition());
        telemetry.put("up extend", upExtendTicks);
        telemetry.put("up extend GET POSITION", upExtend1.getCurrentPosition());

        telemetry.put("FORWARD: ", targetF);
        telemetry.put("TURN: ", targetT);
        telemetry.put("STRAFE: ", targetS);

        telemetry.put("claw guess: ", clawTicks);
        telemetry.put("tilt guess: ", tiltTicks);
        telemetry.put("shoulder guess: ", shoulderTicks);

        telemetry.put("colorsensor hsv ",  HSVasString());
        telemetry.put("what color we got? ", updateColorSensor());
//        telemetry.put("keep the block: ", keepBlock());
        telemetry.put("ejecting???", eject);
        dashboard.sendTelemetryPacket(p);
        return telemetry;
    }


    public String updateColorSensor() {
        double hue = getHSV()[0];
        if (hue < 35 && hue > 20) {     //90    70
            currentSample = Robot.CurrentSample.NEUTRAL;
            return "NEUTRAL";
        } else if (hue < 360 && hue > 350) {        //60    20
            currentSample = Robot.CurrentSample.RED;
            return "RED";
        } else if (hue < 220 && hue > 210) {
            currentSample = Robot.CurrentSample.BLUE;
            return "BLUE";
        } else {
            currentSample = Robot.CurrentSample.NO_SAMPLE;
            return "NO SAMPLE";
        }
    }


    public String HSVasString () {
        float[] hsv = getHSV();
        return hsv[0] + " " + hsv[1] + " " + hsv[2];
    }
    public float[] getHSV() {
        float[] hsv = new float[3];
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);
        return hsv;
    }

    @Override
    public String getTelemetryName() {
        return "Giant Robot";
    }

    @Override
    public void stop() {

    }

    @Override
    public void resetStates() {

    }

    public static double servoNormalize(int pulse) {
        double normalized = (double) pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

    public void mecanumDrive(double forwardX, double strafeX, double turnX) {
        forward = forwardX;
        strafe =  strafeX*strafelim;
        turn = turnX*turnlim;       //*.65
        double r = Math.hypot(strafe, forward);
        double robotAngle = Math.atan2(forward, strafe) - Math.PI/4;
        double rightX = -turn;
        leftFront.setPower((r * Math.cos(robotAngle) - rightX));
        rightFront.setPower((r * Math.sin(robotAngle) + rightX));
        leftBack.setPower((r * Math.sin(robotAngle) - rightX));
        rightBack.setPower((r * Math.cos(robotAngle) + rightX));
    }
    public void setDrive(double forwardX, double strafeX, double turnX) {
        targetF=forwardX;
        targetT=turnX;
        targetS=strafeX;
    }
    public double getVert(){return leftFront.getCurrentPosition();}
    public double getHor(){return leftBack.getCurrentPosition();}
    public void resetDrive(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }


    //MANUAL VERTICLE SLIDES RECALLIBRATION
    public boolean getlimit(){return calibrate;}
    public void recallibrate(){calibrate=!calibrate;}
    public void calibrateUp(){
        upExtend1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upExtend1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        upExtend1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        calibrate=false;
    }

    //TILT THINGS
    public int getTilt(){return tiltTicks;}
    public void addTilt(int x){tiltTicks+=x;}
    public void setTilt(int x){tiltTicks=x;}

    //SHOULDER THINGS
    public int getShoulder(){return shoulderTicks;}
    public void addShoulder(int x){shoulderTicks+=x;}
    public void setShoulder(int x){shoulderTicks=x;}

    //OUT SLIDES THINGS
    public void addOutExtend(int x){outExtendTicks+=x;}
    public void setOutExtend(int x){outExtendTicks=x;}
    public int getOutExtend(){return outExtendTicks;}
    public int getOutMotor(){return outExtend.getCurrentPosition();}

    //UP SLIDES THINGS
    public void addUpExtend(int x){upExtendTicks+=x;}
    public void setUpExtend(int x){upExtendTicks=x;}
    public int getUpExtend(){return upExtendTicks;}     //maybe change to return upExtend1.getCurrentPosition();
    public int getUpMotor(){return upExtend1.getCurrentPosition();}

    //CLAW THINGS
    public void grabBlock(){clawTicks=1460;}
    public void dropBlock(){clawTicks=2050;}
    public void setClaw(int x){clawTicks+=x;}
    public void setClawP(){clawOpen=!clawOpen;}
    public void open(){clawOpen=true;}
    public void close(){clawOpen=false;}
    public boolean isClawOpen(){return clawOpen;}
    public int getClaw(){return clawTicks;}

    //GAMEMODE HOOK VS BASKET
    public void mode(){hook=!hook;}
    public boolean getMode(){return hook;}

    public static void changeAlly(){
        allianceRed=!allianceRed;
    }

    public double getDistance(){
        return dist;
    }

    //VERTICLE SLIDE MOVEMENT TOGETHER
    public boolean up(){
        int old=upExtend1.getCurrentPosition()-upExtend1.getTargetPosition();
        if(old<0){
            return true;
        }
        return false;
    }
    public void moving(){
        if(upExtend1.isBusy()){
            int old=upExtend1.getCurrentPosition()-upExtend1.getTargetPosition();
            if(Math.abs(old)>30){
                if(up()){
                    upExtend2.setPower(1);
                }else{
                    upExtend2.setPower(-1);
                }
            }else{
                upExtend2.setPower(0);
            }
        }
        else{
            upExtend2.setPower(0);
        }
    }

}
