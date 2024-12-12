package org.firstinspires.ftc.teamcode.robots.core;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;

import java.util.LinkedHashMap;
import java.util.Map;

public class Robot implements Subsystem {
    HardwareMap hardwareMap;
    DcMotorEx leftFront, leftBack, rightFront, rightBack;
    Servo claw;
    Gamepad gamepad1;
    DcMotorEx shoulder;
    DcMotorEx slide;
    public boolean clawOpen = false;
    public double clawOpenPosition = 1;
    public double clawClosePosition = 0.7;
    public int shoulderTargetPosition = 0;
    public int slideTargetPosition = 0;
    public int gearboxpick = 0;

    public Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad;
    }

    @Override
    public void update(Canvas fieldOverlay) {
        if(gamepad1.a) {
            shoulderTargetPosition = 50;
            slideTargetPosition = 50;
            clawOpen = true;
        }

        if(gamepad1.x){
            pickup();
        }

        // Claw Open
        if (gamepad1.b) {
            claw.setPosition(clawOpenPosition);
            clawOpen = true;
        }

        // Claw Close
        if (gamepad1.a){
            claw.setPosition(clawClosePosition);
        }
        if(gamepad1.right_trigger >= 0.3){
            if (shoulder.getCurrentPosition() < 900){
                shoulderTargetPosition = shoulder.getCurrentPosition() + 20;
            } else {
                shoulderTargetPosition = 900;
            }
        }
        else if (gamepad1.left_trigger >= 0.3){
            if (shoulder.getCurrentPosition() > 0){
                shoulderTargetPosition = shoulder.getCurrentPosition() - 20;
            } else {
                shoulderTargetPosition = 0;
            }

        }

        if (gamepad1.right_bumper){
            if (slide.getCurrentPosition() < 1450){
                slideTargetPosition = slide.getCurrentPosition() + 10;
            } else {
                slide.setTargetPosition(1450);
            }
        }
        if (gamepad1.left_bumper){
            if(slide.getCurrentPosition() > 0){
                slideTargetPosition = slide.getCurrentPosition() - 10;
            } else {
                slide.setTargetPosition(0);
            }
        }


        updateMotors();
        mecanumDrive(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_stick_x);
    }

    public void updateMotors() {
        if (clawOpen) {
            claw.setPosition(clawOpenPosition);
        } else {
            claw.setPosition(clawClosePosition);
        }
        shoulder.setTargetPosition(shoulderTargetPosition);
        slide.setTargetPosition(slideTargetPosition);
    }

    public void mecanumDrive(double forward, double strafe, double turn) {
        double negS = -strafe;
        double r = Math.hypot(negS, -forward);
        double robotAngle = Math.atan2(-forward, negS) - Math.PI / 4;
        leftFront.setPower(-((r * Math.cos(robotAngle) - turn)));
        rightFront.setPower(-((r * Math.sin(robotAngle) + turn)));
        leftBack.setPower(-((r * Math.sin(robotAngle) - turn)));
        rightBack.setPower(-((r * Math.cos(robotAngle) + turn)));
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
        claw = hardwareMap.get(Servo.class, "claw");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        // Set motor run modes
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        shoulder.setPower(10);
        shoulder.setVelocity(50);
        shoulder.setTargetPosition(gearboxTargetPosition);
        shoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slide.setPower(1);
        slide.setVelocity(150);
        slide.setTargetPosition(0);
        slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        claw.setPosition(clawOpenPosition);
    }

    public void pickup(){
        // Move back a smidge
        mecanumDrive(-1, 0, 0);

        if (leftFront.getCurrentPosition() == 75){
            // Angle gearbox
            gearbox.setTargetPosition(gearboxpick);

            // Close claw
            claw.setPosition(clawClosePosition);
        }
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Claw Open", clawOpen);
        telemetry.put("Shoulder Position", shoulder.getCurrentPosition());
        telemetry.put("Shoulder Target Position", shoulderTargetPosition);
        telemetry.put("Slide Position", slide.getCurrentPosition());
        telemetry.put("Slide Target Position", slideTargetPosition);

        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "Core Robot";
    }
}