package org.firstinspires.ftc.teamcode.robots.core;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.deepthought.field.Field;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;

import java.util.LinkedHashMap;
import java.util.Map;

public class Robot implements Subsystem {
    HardwareMap hardwareMap;
    DcMotorEx leftFront, leftBack, rightFront, rightBack;
    Servo claw;
    Gamepad gamepad1;
    DcMotorEx shoulder;
    DcMotorEx elbow;
    DcMotorEx slide;
    public boolean clawOpen = false;
    public double clawOpenPosition = 1;
    public double clawClosePosition = 0.4;
    public int shoulderTargetPosition = 0;
    public int elbowTargetPosition = 0;
    public int slideTargetPosition = 0;

    public Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad;
    }

    @Override
    public void update(Canvas fieldOverlay) {

        if(gamepad1.a) {
            shoulderTargetPosition = 50;
            elbowTargetPosition = 75;
            slideTargetPosition = 50;
            clawOpen = true;
        }

        if(gamepad1.b) {
            clawOpen = !clawOpen;
        }
        if(clawOpen) {
            claw.setPosition(clawOpenPosition);
        } else {
            claw.setPosition(clawClosePosition);
        }
        if(gamepad1.right_trigger >= 0.3){
            if (shoulder.getCurrentPosition() < 1000){
                shoulder.setTargetPosition(shoulder.getCurrentPosition() + 100);
            } else {
                shoulder.setTargetPosition(1010);
            }
        }
        else if (gamepad1.left_trigger >= 0.3){
            if (shoulder.getCurrentPosition() > 0){
                shoulder.setTargetPosition(shoulder.getCurrentPosition() - 100);
            } else {
                shoulder.setTargetPosition(0);
            }

        }
        if(gamepad1.right_bumper) {
            if (elbow.getCurrentPosition() < 1000){
                elbow.setTargetPosition(elbow.getCurrentPosition() + 100);
            } else {
                elbow.setTargetPosition(1010);
            }
        }
        if (gamepad1.left_bumper) {
            if(elbow.getCurrentPosition() > 0){
                elbow.setTargetPosition(elbow.getCurrentPosition() - 100);
            } else {
                elbow.setTargetPosition(0);
            }
        }

        if (gamepad1.dpad_up){
            if (slide.getCurrentPosition() < 1000){
                slide.setTargetPosition(slide.getCurrentPosition() + 100);
            } else {
                slide.setTargetPosition(1010);
            }
        }
        if (gamepad1.dpad_down){
            if(slide.getCurrentPosition() > 0){
                slide.setTargetPosition(slide.getCurrentPosition() - 100);
            } else {
                slide.setTargetPosition(0);
            }
        }

        updateMotors();
        mecanumDrive(gamepad1.left_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_y);

    }

    public void updateMotors() {
        if(clawOpen) {
            claw.setPosition(clawOpenPosition);
        } else {
            claw.setPosition(clawClosePosition);
        }
        shoulder.setTargetPosition(shoulderTargetPosition);
        elbow.setTargetPosition(elbowTargetPosition);
        slide.setTargetPosition(slideTargetPosition);
    }

    public void mecanumDrive(double forward, double strafe, double turn) {
        double negS = -strafe;
        double r = Math.hypot(negS, forward);
        double robotAngle = Math.atan2(forward, negS) - Math.PI / 4;
        leftFront.setPower((r * Math.cos(robotAngle) - turn));
        rightFront.setPower((r * Math.sin(robotAngle) + turn));
        leftBack.setPower((r * Math.sin(robotAngle) - turn));
        rightBack.setPower((r * Math.cos(robotAngle) + turn));
    }

    @Override
    public void stop() {

    }

    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        claw = hardwareMap.get(Servo.class, "claw");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        // Set motor runmodes
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        shoulder.setPower(1);
        shoulder.setVelocity(800);
        shoulder.setTargetPosition(0);
        shoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        elbow.setPower(1);
        elbow.setVelocity(800);
        elbow.setTargetPosition(0);
        elbow.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slide.setPower(1);
        slide.setVelocity(150);
        slide.setTargetPosition(0);
        slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Claw Open", clawOpen);
        telemetry.put("Shoulder Position", shoulder.getCurrentPosition());
        telemetry.put("Shoulder Target Position", shoulderTargetPosition);
        telemetry.put("Elbow Position", elbow.getCurrentPosition());
        telemetry.put("Elbow Target Position", elbowTargetPosition);
        telemetry.put("Slide Position", slide.getCurrentPosition());
        telemetry.put("Slide Target Position", slideTargetPosition);


        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "Core Robot";
    }

}
