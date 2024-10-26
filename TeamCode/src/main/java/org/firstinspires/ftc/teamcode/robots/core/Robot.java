package org.firstinspires.ftc.teamcode.robots.core;

import android.content.pm.LabeledIntent;

import com.acmerobotics.dashboard.canvas.Canvas;
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
    HardwareMap hardwareMap;
    DcMotorEx leftFront, leftBack, rightFront, rightBack;
    Servo claw;
    Gamepad gamepad1;
    public boolean clawOpen = false;

    public Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad;
    }

    @Override
    public void update(Canvas fieldOverlay) {
        /*if(gamepad1.b) {
            clawOpen = !clawOpen;
        }

        if(clawOpen) {
            claw.setPosition(1);
        } else {
            claw.setPosition(0);
        }*/

        mecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

    public void mecanumDrive(double forward, double strafe, double turn) {
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
        //claw = hardwareMap.get(Servo.class, "claw");
        // Set motor runmodes
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Claw Open", clawOpen);
        telemetry.put("hi", "mom");

        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "Core Robot";
    }

}
