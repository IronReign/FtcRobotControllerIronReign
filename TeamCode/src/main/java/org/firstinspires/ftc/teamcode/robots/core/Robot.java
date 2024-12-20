package org.firstinspires.ftc.teamcode.robots.core;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;

import java.util.LinkedHashMap;
import java.util.Map;

public class Robot implements Subsystem {
    HardwareMap hardwareMap;
    DcMotorEx leftFront, leftBack, rightFront, rightBack;
    DcMotorEx vertical, horizontal;
    Servo claw;
    Gamepad gamepad1;
    DcMotorEx shoulder;
    DcMotorEx slide;
    public static final double CURRENT_THRESHOLD = 2.2;
    public static int CALIBRATE_POSITION = Integer.MAX_VALUE;

    public boolean clawOpen = false;
    public double clawOpenPosition = 1;
    public double clawClosePosition = 0.6;
    public int shoulderTargetPosition = 0;
    public int slideTargetPosition = 0;
    public int gearboxpick = 0;

    public Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad;
    }

    @Override
    public void update(Canvas fieldOverlay) {
        /*if(gamepad1.a) {
            shoulderTargetPosition = 50;
            slideTargetPosition = 50;
            clawOpen = true;
        }

        if(gamepad1.x){
            pickup();
        }*/

        // Claw Controls
        if(gamepad1.b) {
            clawOpen = !clawOpen;
        }
        if(clawOpen) {
            claw.setPosition(clawOpenPosition);
        } else {
            claw.setPosition(clawClosePosition);
        }

        if(gamepad1.right_trigger >= 0.3){
            if (shoulder.getCurrentPosition() < 1200){
                shoulderTargetPosition = shoulder.getCurrentPosition() + 75;
            } else {
                shoulderTargetPosition = 1200;
            }
        }
        else if (gamepad1.left_trigger >= 0.3){
            if (shoulder.getCurrentPosition() > 0){
                shoulderTargetPosition = shoulder.getCurrentPosition() - 75;
            } else {
                shoulderTargetPosition = 0;
            }

        }

        if (gamepad1.right_bumper){
            if (slide.getCurrentPosition() < 1450){
                slideTargetPosition = slide.getCurrentPosition() + 60;
            } else {
                slide.setTargetPosition(1450);
            }
        }
        if (gamepad1.left_bumper){
            if(slide.getCurrentPosition() > 0){
                slideTargetPosition = slide.getCurrentPosition() - 60;
            } else {
                slide.setTargetPosition(0);
            }
        }


        updateMotors();
        mecanumDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
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
        double r = Math.hypot(negS, forward);
        double robotAngle = Math.atan2(forward, negS) - Math.PI / 4;
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
        horizontal = hardwareMap.get(DcMotorEx.class, "horizontal");
        vertical = hardwareMap.get(DcMotorEx.class, "vertical");
        // Set motor run modes
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vertical.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        shoulder.setPower(1);
        shoulder.setVelocity(300);
        shoulder.setTargetPosition(700);
        shoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slide.setPower(1);
        slide.setVelocity(300);
        slide.setTargetPosition(slideTargetPosition);
        slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        vertical.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        claw.setPosition(clawOpenPosition);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Claw Open", clawOpen);
        telemetry.put("Shoulder Power", shoulder.getCurrent(CurrentUnit.AMPS));
        telemetry.put("Shoulder Position", shoulder.getCurrentPosition());
        telemetry.put("Shoulder Target Position", shoulderTargetPosition);
        telemetry.put("Slide Position", slide.getCurrentPosition());
        telemetry.put("Slide Target Position", slideTargetPosition);
        telemetry.put("Horizontal", horizontal.getCurrentPosition());
        telemetry.put("Vertical", vertical.getCurrentPosition());
        telemetry.put("Calibrate Stage", calibrateStage);

        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "Core Robot";
    }

    public static int calibrateStage = 0;
    public boolean calibrate() {
        switch (calibrateStage) {
            case 0:
                shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shoulder.setPower(-.2);
                calibrateStage++;
                break;
            case 1:
                if (CALIBRATE_POSITION == shoulder.getCurrentPosition()) {
                    shoulder.setPower(0);
                    shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    calibrateStage++;
                }
                CALIBRATE_POSITION = shoulder.getCurrentPosition();
                break;
            case 2:
                shoulder.setPower(1);
                shoulder.setTargetPosition(0);
                shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                calibrateStage++;
                break;
            case 3:
                shoulder.setTargetPosition(700);
                return true;
        }
        return false;
    }
}