package org.firstinspires.ftc.teamcode.robots.bobobot.TeleOpSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;


public class IntakeClaw {
    private DcMotorEx clawArm = null;
    private Servo clawSpan = null;
    private Servo clawWrist = null;
    public Joint wrist;
    public static double OPENCLAW = 0.35;
    public static double CLOSECLAW = 0.55;
    public static double PER_DEGREE = 5.8;
    public static double PER_TICK = 0.1724;
    public double WRIST_MIN = Utils.servoNormalize(1640);

    public static double WRIST_INIT_POSITION = Utils.servoNormalize(2105);
    public static double WRIST_MAX = Utils.servoNormalize(2200);
    public static double WRIST_SCORE_1 = Utils.servoNormalize(2184);
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    public IntakeClaw(Telemetry telemetry, HardwareMap hardwareMap) {
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
        //wrist = new Joint(hardwareMap, "Claw Wrist", false, WRIST_INIT_POSITION, PER_DEGREE, 1300*PER_TICK, 2300*PER_TICK, 2105*PER_TICK, 1 );
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
        if(press == true && clawArm.getCurrentPosition() < 1000) {
            clawArm.setTargetPosition(clawArm.getCurrentPosition() + 95);
        }
    }
    public void clawArmLower (boolean press) {
        if(press == true && clawArm.getCurrentPosition() > 20) {
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
        if (press == true) {
            clawWrist.setPosition(WRIST_MIN);
        }
//        if(press == true && clawWrist.getPosition() > WRIST_MIN){
//            clawWrist.setPosition(clawWrist.getPosition() - 60);
//        }
    }
    public void armPositionTest() {
        clawArm.setTargetPosition(90);
    }

    public void setWristScore1(){

    }

    public void inTake (boolean press) {
        if(press == true){
            closeClaw(true);
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            clawArm.setTargetPosition(200);
        }
    }
}

