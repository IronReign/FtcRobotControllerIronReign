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
    public double clawClosePosition = 0.7;
    public int shoulderTargetPosition = 250;
    public int elbowTargetPosition = 10;
    public int slideTargetPosition = 0;
    public int shoulderLimit = 255;
    public int elbowLimit = 100;


    public static int autonIndex = 0;
    long autonTimer = futureTime(10);
    public static double adjust_time = 1.0;
    public static double powerOne = 0.1;
    public static int timeOne = 1;
    public static int timeTurn = 1;
    public static int slideUp = 1500;
    public static int shoulderUp = 250;
    public static int elbowUp = 150;
    public static int slideRe = 0;
    public static int shoulderDown = 75;
    public static int elbowDown = 50;

    public Robot(HardwareMap hardwareMap, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad;
    }
    /*public void starting(){
        slide.setTargetPosition(slideTargetPosition);
        shoulder.setTargetPosition(shoulderTargetPosition);
        elbow.setTargetPosition(elbowTargetPosition);

    }*/

    public void execute(){

        switch(autonIndex){
            case 0:
                // Move strafe to the left side
                leftFront.setPower(powerOne); //RIGHTFRONT
                rightFront.setPower(-powerOne); //LEFT FRONT
                leftBack.setPower(-powerOne); //RIGHTBACK
                rightBack.setPower(powerOne);
                if (rightBack.getCurrentPosition() == rightBack.getTargetPosition()){
                    autonIndex ++;
                }
                // NUMBER OF TICKS (AMOUNT) IT MOVES PER FOOT

                break;

            case 1:
                // Turn -135 degrees
                leftFront.setPower(-powerOne);
                rightFront.setPower(powerOne);
                leftBack.setPower(-powerOne);
                rightBack.setPower(powerOne);
                rightBack.setTargetPosition(100);

                if (rightBack.getCurrentPosition() == rightBack.getTargetPosition()){
                    autonIndex ++;
                }

                break;

            case 2:
                // Extend Linear Slide
                if (isPast(autonTimer)){
                    slide.setTargetPosition(slideUp);
                    shoulder.setTargetPosition(shoulderUp);
                    elbow.setTargetPosition(elbowUp);
                    if ((slide.getCurrentPosition() == slide.getTargetPosition())&(shoulder.getCurrentPosition()==shoulder.getTargetPosition())&(elbow.getCurrentPosition()==elbow.getTargetPosition())){
                        autonIndex++;
                    }
                }
                break;

            case 3:
                // Open claw to release block
                claw.setPosition(clawOpenPosition);
                autonIndex++;
                break;

            case 4:
                // Strafe back
                leftFront.setPower(-powerOne); //RIGHTFRONT
                rightFront.setPower(powerOne); //LEFT FRONT
                leftBack.setPower(powerOne); //RIGHTBACK
                rightBack.setPower(-powerOne);
                rightBack.setTargetPosition(100);

                if (rightBack.getCurrentPosition() == rightBack.getTargetPosition()){
                    autonIndex ++;
                }
                break;

            case 5:
                // Move forward
                leftFront.setPower(powerOne);
                rightFront.setPower(powerOne);
                leftBack.setPower(powerOne);
                rightBack.setPower(powerOne);
                rightBack.setTargetPosition(100);

                if (rightBack.getCurrentPosition() == rightBack.getTargetPosition()){
                    autonIndex ++;
                }
                break;

            case 6:
                // Strafe right
                leftFront.setPower(-powerOne); //RIGHTFRONT
                rightFront.setPower(powerOne); //LEFT FRONT
                leftBack.setPower(powerOne); //RIGHTBACK
                rightBack.setPower(-powerOne);
                rightBack.setTargetPosition(100);

                if (rightBack.getCurrentPosition() == rightBack.getTargetPosition()){
                    autonIndex ++;
                }

                break;

            case 7:
                // Return to normal
                slide.setTargetPosition(slideTargetPosition);
                shoulder.setTargetPosition(shoulderTargetPosition);
                elbow.setTargetPosition(elbowTargetPosition);

                leftFront.setPower(0); //RIGHTFRONT
                rightFront.setPower(0); //LEFT FRONT
                leftBack.setPower(0); //RIGHTBACK
                rightBack.setPower(0);

                break;

            /*case 8:
                // Move a unit
                if (isPast(autonTimer)){
                    leftFront.setPower(powerOne);
                    rightFront.setPower(-powerOne);
                    leftBack.setPower(powerOne);
                    rightBack.setPower(-powerOne);
                    autonTimer = futureTime(timeOne);
                    autonIndex++;
                }
                break;

            case 9:
                // Extend Slide angled down
                if (isPast(autonTimer)) {
                    slide.setTargetPosition(slideUp);
                    shoulder.setTargetPosition(shoulderDown);
                    elbow.setTargetPosition(elbowDown);
                    if ((slide.getCurrentPosition() == slide.getTargetPosition()) & (shoulder.getCurrentPosition() == shoulder.getTargetPosition()) & (elbow.getCurrentPosition() == elbow.getTargetPosition())) {
                        autonIndex++;
                    }
                }
                break;

            case 10:
                // Close claw (hold brick)
                claw.setPosition(clawClosePosition);
                autonIndex++;
                break;

            case 11:
                // Retract slide
                slide.setTargetPosition(slideRe);
                if ((slide.getCurrentPosition() == slide.getTargetPosition())){
                    autonIndex++;
                }
                break;

            case 12:
                // Move back a unit
                leftFront.setPower(-powerOne);
                rightFront.setPower(powerOne);
                leftBack.setPower(-powerOne);
                rightBack.setPower(powerOne);
                autonTimer = futureTime(timeOne);
                autonIndex++;
                break;

            case 13:
                // Turn -180 degrees
                if (isPast(autonTimer)) {
                    leftFront.setPower(powerOne);
                    rightFront.setPower(powerOne);
                    leftBack.setPower(powerOne);
                    rightBack.setPower(powerOne);
                    autonTimer = futureTime((float) (timeOne * 4) /3);
                    autonIndex++;
                    //autonIndex=autonIndex-13;
                    //adjust_time = 0.75;
                }
                break;*/

            default: break;
        }
    }

    @Override
    public void update(Canvas fieldOverlay) {
        if (gamepad1.a) {
            shoulderTargetPosition = 50;
            elbowTargetPosition = 75;
            slideTargetPosition = 50;
            clawOpen = true;
        }

        if (gamepad1.b) {
            clawOpen = !clawOpen;
        }

        if (clawOpen) {
            claw.setPosition(clawOpenPosition);
        } else {
            claw.setPosition(clawClosePosition);
        }
        if (gamepad1.right_trigger >= 0.3) {
            //if (shoulder.getCurrentPosition() < 1000){
                shoulderTargetPosition = shoulder.getCurrentPosition() + 100;
            //} else {
             //   shoulder.setTargetPosition(1010);
            //}
        }
        if (gamepad1.left_trigger >= 0.3) {
            //if (shoulder.getCurrentPosition() > 0){
                shoulderTargetPosition = shoulder.getCurrentPosition() - 100;
            //} else {
              //  shoulder.setTargetPosition(0);
            //}
        }

        if (gamepad1.right_bumper) {
            //if (elbow.getCurrentPosition() < 1000){
                elbowTargetPosition = elbow.getCurrentPosition() + 100;
            //} else {
            //elbow.setTargetPosition(1010);
            //}
        }
        if (gamepad1.left_bumper) {
            //if(elbow.getCurrentPosition() > 0){
                elbowTargetPosition = elbow.getCurrentPosition() - 100;
            //} else {
            //  elbow.setTargetPosition(0);
            //}
        }

        if (gamepad1.dpad_up) {
            if (shoulder.getCurrentPosition() <= shoulderLimit || elbow.getCurrentPosition() <= elbowLimit) {
                if (slide.getCurrentPosition() < 1100) {
                    slideTargetPosition = slide.getCurrentPosition() + 100;
                } else {
                    slide.setTargetPosition(1200);
                }
            } else {
                slideTargetPosition = slide.getCurrentPosition() + 100;
            }
        }
        if (gamepad1.dpad_down) {
            if(slide.getCurrentPosition() >= 100){
                slideTargetPosition = slide.getCurrentPosition() - 100;
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
        elbow.setTargetPosition(elbowTargetPosition);
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
        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        claw = hardwareMap.get(Servo.class, "claw");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        // Set motor runmodes
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        shoulder.setPower(1);
        shoulder.setVelocity(50);
        shoulder.setTargetPosition(shoulderTargetPosition);
        shoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        elbow.setPower(1);
        elbow.setVelocity(50);
        elbow.setTargetPosition(elbowTargetPosition);
        elbow.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slide.setPower(1);
        slide.setVelocity(5);
        slide.setTargetPosition(slideTargetPosition);
        slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        claw.setPosition(clawOpenPosition);
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