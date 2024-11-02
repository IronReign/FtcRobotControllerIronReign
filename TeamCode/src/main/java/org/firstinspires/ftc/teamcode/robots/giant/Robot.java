package org.firstinspires.ftc.teamcode.robots.giant;

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
    private BNO055IMU imu;
    HardwareMap hardwareMap;
    DcMotorEx leftFront, leftBack, rightFront, rightBack;
    VisionProvider camera;
    //red 110-140



    //ask where/when to use IMU



    Servo claw;
    StickyGamepad g1=null;
    Gamepad gamepad1;
    public static final double objectWidthInRealWorldUnits= 3.4;
    public static final double focalLength=1430;
    double forward = 0;
    double strafe = 0;
    double turn = 0;

    int clawTicks=0;

    //public boolean clawOpen = false;

    public Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad;
    }

    @Override
    public void update(Canvas fieldOverlay) {
        camera.update(true);
       // claw.setPosition(servoNormalize(clawTicks));

//        if(g1.left_bumper) {
//            clawTicks-=15;
//        }
//        if(g1.right_bumper) {
//            clawTicks+=15;
//        }



        mecanumDrive();
    }

    public void mecanumDrive() {
        forward = gamepad1.left_stick_y;
        strafe =  -gamepad1.left_stick_x;
        turn = -gamepad1.right_stick_x;
        double r = Math.hypot(strafe, forward);
        double robotAngle = Math.atan2(forward, strafe) - Math.PI/4;
        double rightX = -turn;
        leftFront.setPower((r * Math.cos(robotAngle) - rightX));
        rightFront.setPower((r * Math.sin(robotAngle) + rightX));
        leftBack.setPower((r * Math.sin(robotAngle) - rightX));
        rightBack.setPower((r * Math.cos(robotAngle) + rightX));
    }


    @Override
    public void stop() {

    }

    public void init() {
        g1 = new StickyGamepad(gamepad1);
        try{
            camera = VisionProviders.VISION_PROVIDERS[6].newInstance();
        }catch(Exception e) {
            throw new RuntimeException(e);
        }
        camera.initializeVision(hardwareMap,this,false);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
      //  claw = hardwareMap.get(Servo.class, "claw");
        // Set motor runmodes
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();
        TelemetryPacket p = new TelemetryPacket();
        //p.addLine("distance: ", getDistance());
      //  telemetry.put("Claw Open", clawOpen);
        telemetry.put("its", "working");
        telemetry.put("forward:" , forward);
        telemetry.put("strafe" , strafe);
        telemetry.put("turn" , turn);
        //telemetry.put("servo", clawTicks);
        telemetry.put("Current IMU Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.put("vision class", camera.getClass());
        telemetry.put("vision", camera.getTelemetryName());
        dashboard.sendTelemetryPacket(p);

        return telemetry;
    }

    public static double servoNormalize(int pulse) {
        double normalized = (double) pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

//    public void power(double output){
//        leftFront.setPower(-output);
//        leftBack.setPower(-output);
//        rightFront.setPower(output);
//        rightBack.setPower(output);
//    }

    @Override
    public String getTelemetryName() {
        return "Giant Robot";
    }

}
