package org.firstinspires.ftc.teamcode.robots.giant;

import android.content.pm.LabeledIntent;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

public class Robot implements Subsystem {
    static FtcDashboard dashboard = FtcDashboard.getInstance();
    HardwareMap hardwareMap;
    DcMotorEx leftFront, leftBack, rightFront, rightBack;

    //Servo claw;
    //StickeyGamepad g1;
    Gamepad gamepad1;
    double forward = 0;
    double strafe = 0;
    double turn = 0;

    //public boolean clawOpen = false;

    public Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad;
    }

    @Override
    public void update(Canvas fieldOverlay) {
//        if(gamepad1.b) {
//            clawOpen = !clawOpen;
//        }
//
//        if(clawOpen) {
//            claw.setPosition(1);
//        } else {
//            claw.setPosition(0);
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
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
       // claw = hardwareMap.get(Servo.class, "claw");
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
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();

      //  telemetry.put("Claw Open", clawOpen);
        telemetry.put("its", "working");
        telemetry.put("forward:" , forward);
        telemetry.put("strafe" , strafe);
        telemetry.put("turn" , turn);

        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "Giant Robot";
    }

}
