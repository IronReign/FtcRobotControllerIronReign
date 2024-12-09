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
    DcMotorEx gearbox;
    DcMotorEx slide;
    public boolean clawOpen = false;
    public double clawOpenPosition = 1;
    public double clawClosePosition = 0.7;
    public int gearboxTargetPosition = 10;
    public int slideTargetPosition = 0;

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

    @Override
    public void update(Canvas fieldOverlay) {
        // Initialize
        if (gamepad1.y) {
            gearbox.setTargetPosition(gearboxTargetPosition);
            slide.setTargetPosition(slideTargetPosition);
            clawOpen = true;
        }

        if(gamepad1.x){
            // Call pickup func from AutonCode2 - Ask Jai
        }

        // Claw Open
        if (gamepad1.b) {
            claw.setPosition(clawOpenPosition);
            clawOpen = true;
        }

        // Claw Close
        if (gamepad1.a){
            claw.setPosition(clawClosePosition);
            clawOpen = false;
        }

        // Extend Linear Slide
        if (gamepad1.dpad_up) {
            gearboxTargetPosition = gearbox.getCurrentPosition() + 100;
        }

        // Retract Linear Slide
        if (gamepad1.dpad_down){
            gearboxTargetPosition = gearbox.getCurrentPosition() - 100;
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
        gearbox.setTargetPosition(gearboxTargetPosition);
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
        gearbox = hardwareMap.get(DcMotorEx.class, "gearbox");
        claw = hardwareMap.get(Servo.class, "claw");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        // Set motor run modes
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        gearbox.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        gearbox.setPower(10);
        gearbox.setVelocity(50);
        gearbox.setTargetPosition(gearboxTargetPosition);
        gearbox.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        slide.setPower(10);
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
        telemetry.put("Gearbox Position", gearbox.getCurrentPosition());
        telemetry.put("Gearbox Target Position", gearboxTargetPosition);
        telemetry.put("Slide Position", slide.getCurrentPosition());
        telemetry.put("Slide Target Position", slideTargetPosition);

        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "Core Robot";
    }
}