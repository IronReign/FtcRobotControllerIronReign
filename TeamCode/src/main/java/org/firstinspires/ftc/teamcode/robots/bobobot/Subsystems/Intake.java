package org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;


public class Intake {
    private DcMotorEx clawArm = null;
    private Servo clawSpan = null;
    private Servo clawWrist = null;
    public static double OPENCLAW = 0.35;
    public static double CLOSECLAW = 0.55;
    public static double PER_DEGREE = 5.8;
    public static double PER_TICK = 0.1724;
    public static double WRIST_MIN = Utils.servoNormalize(1650);

    public static double WRIST_INIT_POSITION = Utils.servoNormalize(2105);
    public static double WRIST_MAX = Utils.servoNormalize(2200);
    public static double WRIST_SCORE_1 = Utils.servoNormalize(1800);
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    public Intake(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }
    public void telemetryOutput() {
        telemetry.addData("Claw Position \t", Utils.servoDenormalize(clawSpan.getPosition()));
        telemetry.addData("Claw Values \t", clawSpan.getPosition());
        telemetry.addData("Claw Arm Position \t", clawArm.getCurrentPosition());
        telemetry.addData("Arm Speed \t", clawArm.getVelocity());
        telemetry.addData("Claw Wrist Position \t", Utils.servoDenormalize(clawWrist.getPosition()));
        telemetry.addData("Arm Target \t", clawArm.getTargetPosition());
    }
    public void intakeClawInit() {
        clawArm = this.hardwareMap.get(DcMotorEx.class, "clawArm");
        clawSpan = this.hardwareMap.get(Servo.class, "clawSpan");
        clawWrist = this.hardwareMap.get(Servo.class, "clawWrist");
        clawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawArm.setPower(1);
        clawArm.setTargetPosition(20);
        clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void openClaw (boolean press) {
        if(press == true)
            clawSpan.setPosition(OPENCLAW);
    }
    public void closeClaw (boolean press) {
        if(press == true)
            clawSpan.setPosition(CLOSECLAW);

    }
    public void clawArmLift (boolean press) {
        if(press == true && clawArm.getCurrentPosition() < 750) {
            clawArm.setTargetPosition(clawArm.getCurrentPosition() + 75);
        }
    }
    public void clawArmLower (boolean press) {
        if(press == true && clawArm.getCurrentPosition() > 160) {
            clawArm.setTargetPosition(clawArm.getCurrentPosition() - 35);
        }
    }
    public void armWristIn(boolean press) {
        if (press == true && clawArm.getCurrentPosition() < 750) {
            clawWrist.setPosition(WRIST_MAX);
        }
        else if(press == true && clawArm.getCurrentPosition() > 750) {
            clawWrist.setPosition(WRIST_SCORE_1);
        }

    }
    public void armWristOut(boolean press) {
        if (press == true && clawArm.getCurrentPosition() < 400) {
            clawWrist.setPosition(WRIST_MIN);
        }
        if (press == true && clawArm.getCurrentPosition() > 400){
            clawWrist.setPosition(WRIST_SCORE_1);
        }


    }
    public void armPositionTest() {
        clawArm.setTargetPosition(90);
    }



    public void inTake (boolean press) {
        if(press == true){
            clawArm.setTargetPosition(145);
        }
    }
}

