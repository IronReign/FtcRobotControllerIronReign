package org.firstinspires.ftc.teamcode.robots.giant;


import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;


import android.content.pm.LabeledIntent;
import android.graphics.Color;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Trident;
import org.firstinspires.ftc.teamcode.robots.deepthought.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.deepthought.vision.VisionProviders;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.ArrayList;
import java.util.Collections;
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


    //0:leftback
    //1:leftfront
    //2:rightfront


    //public NormalizedColorSensor colorSensor = null;
    public static int colorSensorGain = 12;
    public enum CurrentSample {
        RED, BLUE, NEUTRAL, NO_SAMPLE
    }
    public Robot.CurrentSample currentSample = Robot.CurrentSample.NO_SAMPLE;
    public List<Robot.CurrentSample> targetSamples = new ArrayList<>();
    public Robot.CurrentSample opp;
    public String op;




    DcMotorEx leftFront, leftBack, rightFront, rightBack;
    DcMotorEx shoulder,arm;
    DcMotorEx suck;

    Servo claw;

    boolean sucking=false;
    boolean eject=false;
    boolean slurp=false;

    StickyGamepad g1=null;
    Gamepad gamepad1;

    double forward = 0;
    double strafe = 0;
    double turn = 0;
    double targetF, targetS, targetT;
    int rotate=0;
    int extend=0;
    int clawTicks=1100;
    boolean clawOpen=false;

    public Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad;
        targetSamples.add(Robot.CurrentSample.NEUTRAL);
        targetSamples.add(Robot.CurrentSample.RED);//CHANGE
        if(targetSamples.contains(Robot.CurrentSample.RED)){
            opp=Robot.CurrentSample.BLUE;
            op="BLUE";
        }
        else{
            opp=Robot.CurrentSample.RED;
            op="RED";
        }
    }


    @Override
    public void update(Canvas fieldOverlay) {
        g1.update();
        mecanumDrive(targetF,targetS,targetT);
        shoulder.setTargetPosition(rotate);
        claw.setPosition(servoNormalize(clawTicks));
        arm.setTargetPosition(extend);
        if(clawOpen){
            dropBlock();
        }else{
            grabBlock();
        }
        //updateColorSensor();
        //colorSensor.setGain(colorSensorGain);
//        if(!eject&&!slurp) {
//            if (sucking) {
//                grab();
//            }
//            else{
//                suck.setPower(0);
//            }
//        }else if(eject&&!slurp){
//            suck.setPower(-1);
//        } else{
//            suck.setPower(.8);
//        }

    }


    public void mecanumDrive(double forwardX, double strafeX, double turnX) {
        forward = forwardX;
        strafe =  strafeX;
        turn = turnX*.65;
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


    public void init() {
        g1 = new StickyGamepad(gamepad1);
//        try{
//            camera = VisionProviders.VISION_PROVIDERS[0].newInstance();
//        }catch(Exception e) {
//            throw new RuntimeException(e);
//        }
//        camera

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        //suck = hardwareMap.get(DcMotorEx.class, "suck");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        claw = hardwareMap.get(Servo.class, "claw");



        // Set motor runmodes
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);



        arm.setDirection(DcMotor.Direction.REVERSE);
        shoulder.setDirection(DcMotor.Direction.REVERSE);

        shoulder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        shoulder.setTargetPosition(0);
        shoulder.setPower(1);

        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(0);
        arm.setPower(1);



        //suck.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
       // suck.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // suck.setDirection(DcMotor.Direction.REVERSE);


       // colorSensor = this.hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");
    }

    //rotate entire arm
    public void tu(int x){
        rotate+=x;
    }
    public void setRotate(int x) {
        rotate=x;
    }
    //extend linear slide
    public void extend(int x) {
        extend+=x;
    }
    public void setExtend(int x) {
        extend=x;
    }

    public void setClaw(int x){
        clawTicks+=x;
    }

    public void setShoulderSpeed(double x){
        shoulder.setPower(x);
    }

    public void setClawP(){
        clawOpen=!clawOpen;
    }

    public int getClaw(){
        return clawTicks;
    }



    public void grabBlock(){clawTicks=1100;}

    public void dropBlock(){clawTicks=1700;}

    public int getExtend() {return arm.getCurrentPosition();}

    public int getRotate() {return shoulder.getCurrentPosition();}


//    public void grab(){
//        //if find opponent block reverse motor to spit back out in same direction picked up
//        if(!eject&&!slurp) {
//            if (keepBlock()) {
//                long autonTimer = 0;
//                autonTimer = futureTime(.4);
//                if(isPast(autonTimer)){
//                    suck.setPower(0);
//                    // autonTimer = futureTime(.05);
//                }
////                if (isPast(autonTimer)) {
////                    suck.setPower(0);
////                }
//            }
////            else if (updateColorSensor().equals(op)) {
////                long autonTimer = 0;
////                autonTimer = futureTime(.2);
////                suck.setPower(-1);
////                if (isPast(autonTimer)) {
////                    suck.setPower(1);
////                }
////            }
//            else {
//                suck.setPower(1);
//            }
//        }else if(eject&&!slurp){
//            suck.setPower(-1);
//        }
//        else{
//            suck.setPower(.8);
//        }
//    }
//
//
//    public void eject(double x){
//        suck.setPower(x);
//    }
//    public void spit(boolean x){eject=x;}
//    public void setSlurp(boolean x){slurp=x;}
//
//
//    public void suck(){
//        sucking=!sucking;
//    }
//
//
//
//
//    public boolean keepBlock() {
//        if(targetSamples.contains(currentSample)){
//            sucking=false;
//            return true;
//        }
//        return false;
//    }
//
//
//    //110-140
//    public String updateColorSensor() {
//        double hue = getHSV()[0];
//        if (hue < 40 && hue > 15) {         //90    70
//            currentSample = Robot.CurrentSample.NEUTRAL;
//            return "NEUTRAL";
//        } else if (hue < 360 && hue > 350) {      //60    20
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

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();
        TelemetryPacket p = new TelemetryPacket();
        telemetry.put("FORWARD: ", targetF);
        telemetry.put("TURN: ", targetT);
        telemetry.put("STRAFE: ", targetS);
        telemetry.put("shoudler guess: " , rotate);
        telemetry.put("shoulder actual: " , shoulder.getCurrentPosition());

        telemetry.put("claw guess: ", clawTicks);
        telemetry.put("claw actual:", claw.getPosition());

        telemetry.put("arm guess: ", extend);
        telemetry.put("actual extend: ", arm.getCurrentPosition());

        telemetry.put("CLAW: ", clawTicks);

//        telemetry.put("colorsensor hsv ",  HSVasString());
//        telemetry.put("what color we got? ", updateColorSensor());
////        telemetry.put("keep the block: ", keepBlock());
//        telemetry.put("ejecting???", eject);
        dashboard.sendTelemetryPacket(p);
        return telemetry;
    }


    @Override
    public String getTelemetryName() {
        return "Giant Robot";
    }


    @Override
    public void stop() {


    }
    public static double servoNormalize(int pulse) {
        double normalized = (double) pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }


}

