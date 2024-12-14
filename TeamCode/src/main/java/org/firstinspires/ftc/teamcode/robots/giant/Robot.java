package org.firstinspires.ftc.teamcode.robots.giant;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import android.content.pm.LabeledIntent;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.deepthought.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.deepthought.vision.VisionProviders;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;


public class Robot implements Subsystem {
    static FtcDashboard dashboard = FtcDashboard.getInstance();

    HardwareMap hardwareMap;
    DcMotorEx leftFront, leftBack, rightFront, rightBack;
    DcMotorEx shoulder,arm;
    int rotate=0;
    int extend=0;
    double targetF, targetS, targetT;
    boolean STOP=false;
   // VisionProvider camera;
    //red 110-140

    Servo claw;
    StickyGamepad g1=null;
    Gamepad gamepad1;
    public static final double objectWidthInRealWorldUnits= 3.4;
    public static final double focalLength=1430;
    double forward = 0;
    double strafe = 0;
    double turn = 0;

    int clawTicks=0;


    public static double FORWARD=.69;
    public static double L=.91;
    public static double TURN_FORWARD_1 = .6;
    public static double BACK=.3;
    public static double TIME=.1;
    int autonIndex = 0;
    long autonTimer = 0;

    //public boolean clawOpen = false;

    public Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad;
    }

    @Override
    public void update(Canvas fieldOverlay) {
        g1.update();

        //camera.update(true);
        shoulder.setTargetPosition(rotate);
        claw.setPosition(servoNormalize(clawTicks));
        arm.setTargetPosition(extend);

        mecanumDrive(targetF,targetS,targetT);
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

    public void setDrive(double forwardX, double strafeX, double turnX)
    {
        targetF=forwardX;
        targetT=turnX;
        targetS=strafeX;
    }
    //open and close claw
    public void claw(int x){
        clawTicks=x;
    }

    public void grabBlock(){clawTicks=500;}

    public void dropBlock(){clawTicks=2000;}

    public void rotateUpBasket(){rotate=1950;}

    public void extendTop(){extend=9100;}

    //rotate entire arm
    public void tu(int x){
        rotate+=x;
    }
    //extend linear slide
    public void extend(int x)
    {
        extend+=x;
    }
    public void setExtend(int x)
    {
        extend=x;
    }
    public void setStop(boolean x){
        STOP=x;
    }

    public void resetR() {
        rotate=10;
    }
    public void resetE(){
        extend=2110;
    }

    public int getRotate()
    {
        return shoulder.getCurrentPosition();
    }

    public void setRotate(int x) {
        rotate=x;
    }

    public void unstickArm(){
        extend=7850;
    }
    public void rotateUpBar(){
        rotate=1470+80;
    }
    public void aim(){
        rotate=1295;
    }
    public void safeRotate() {
        extend=3200;
    }
    public void reach() {
        extend=7780; //5460
    }
    public void pull() {
        extend=4450;
    }
    public int getClaw() {
        return clawTicks;
    }
    public void setShoulderSpeed(double x){
        shoulder.setPower(x);
    }

    //2110

    //up good arm=5460 extend
    //up safe rotating arm=3200 extend
    //go down rotate= 1320
    //pull block down extend=4750


    public int getExtend()
    {
        return arm.getCurrentPosition();
    }

    public void doit() {

        extend=2110;
        rotate=1550;
    }
    public void attatch(){
        extend=5;
    }



    public void redo() {
        autonIndex = 0;
        autonTimer = 0;
        if(STOP){
            autonIndex=12;
            shoulder.setPower(1);
            STOP=false;
        }
        switch (autonIndex) {
            case 0:
                autonTimer=futureTime(TIME);
                extend=5;
                autonIndex++;
                break;
            case 1:
                if(isPast(autonTimer)){
                    autonTimer=futureTime(TIME);
                    rotateUpBar();
                    autonIndex++;
                }

                break;

            case 2:
                if(rotate<1450) {
                    autonTimer = futureTime(L);
                    setDrive(-.5, 0, 0);
                    setShoulderSpeed(1);
                    autonIndex++;
                }
                break;
            case 3:
                if(isPast(autonTimer)){
                    // autonTimer = futureTime(TURN_TIMER);
                    setDrive(0, 0, 0);
                    rotateUpBar();
                    autonIndex++;
                }
                break;
            case 4:
                if(getRotate()>1390){
                    autonTimer = futureTime(TURN_FORWARD_1);
                    extend=7850-2110;

                    autonIndex++;
                }
                break;
            case 5:
                if(getExtend()>7750-2110){
                    extend=5460-2110;

                    autonIndex++;
                }
                break;
            case 6:
                if(isPast(autonTimer)) {
                    aim();
                    autonIndex++;
                }
                break;
            case 7:
                if(getRotate()<1280) {
                    autonTimer=futureTime(TIME);
                    extend=4750-2110;
                    autonIndex++;
                }
                break;
            case 8:
                if(getExtend()<4760-2110) {
                    autonTimer=futureTime(BACK);
                    dropBlock();
                    setDrive(.6,0,0);
                    autonIndex++;
                }
                break;
            case 9:
                if(isPast(autonTimer)) {
                    autonTimer=futureTime(TIME);
                    setDrive(0,0,0);
                }
            case 10:
                if(isPast(autonTimer)){
                    setShoulderSpeed(.4);
                    rotate=5;
                    autonIndex++;
                }

                break;
            case 11:
                if(getRotate()<10){
                    setShoulderSpeed(1);
                    autonIndex++;
                }

                break;
            case 12:

                break;

        }
    }



    @Override
    public void stop() {

    }

    public void init() {
        g1 = new StickyGamepad(gamepad1);

//        try{
//            camera = VisionProviders.VISION_PROVIDERS[6].newInstance();
//        }catch(Exception e) {
//            throw new RuntimeException(e);
//        }
       // camera.initializeVision(hardwareMap,this,false);
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
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
        shoulder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        shoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        shoulder.setTargetPosition(0);
        shoulder.setPower(1);

        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(0);
        arm.setPower(1);

//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.mode = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        imu.initialize(parameters);
    }
    public static double servoNormalize(int pulse) {
        double normalized = (double) pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

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

        telemetry.put("autonindex: ",autonIndex);
        telemetry.put("stop???   ",STOP);
       // telemetry.put("Current IMU Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
      //  telemetry.put("vision class", camera.getClass());
       // telemetry.put("vision", camera.getTelemetryName());
        dashboard.sendTelemetryPacket(p);
        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "Giant Robot";
    }

}
