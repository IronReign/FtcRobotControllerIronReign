package org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Joint;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;

import java.util.HashMap;
import java.util.Map;


public class Intake implements Subsystem {
    private DcMotorEx clawArm = null;
    private Servo clawSpan = null;
    private Servo clawWrist = null;
    private Joint wrist = null;
    public static double OPENCLAW = 0.65;
    public static double CLOSECLAW = 0.95;
    public static double PER_DEGREE = 5.8;
    public static double PER_TICK = 0.1724;
    public static double WRIST_MIN = Utils.servoNormalize(1390);
    public static double WRIST_INIT_POSITION = Utils.servoNormalize(2105);
    public static double WRIST_MAX = Utils.servoNormalize(2105);
    public static double WRIST_SCORE_1 = Utils.servoNormalize(1800);

    public enum WristArticulation{
        WRIST_IN, WRIST_OUT;
    }
    public WristArticulation wristArticulation;
    private HardwareMap hardwareMap;
    public RunnerBot runnerBot;
    public Intake(HardwareMap hardwareMap, RunnerBot runnerBot) {
        this.hardwareMap = hardwareMap;
        this.runnerBot = runnerBot;
        clawArm = this.hardwareMap.get(DcMotorEx.class, "clawArm");
        clawSpan = this.hardwareMap.get(Servo.class, "clawSpan");
        clawWrist = this.hardwareMap.get(Servo.class, "clawWrist");

        clawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawArm.setPower(1);
        clawArm.setTargetPosition(0);
        clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    @Override
    public Map<String, Object> getTelemetry(boolean debug){
        Map<String, Object> telemetryMap = new HashMap<>();
        telemetryMap.put("Claw Position \t", Utils.servoDenormalize(clawSpan.getPosition()));
        telemetryMap.put("Claw Values \t", clawSpan.getPosition());
        telemetryMap.put("Claw Arm Position \t", clawArm.getCurrentPosition());
        telemetryMap.put("Arm Speed \t", clawArm.getVelocity());
        telemetryMap.put("Claw Wrist Position \t", Utils.servoDenormalize(clawWrist.getPosition()));
        telemetryMap.put("Arm Target \t", clawArm.getTargetPosition());
        telemetryMap.put("Wrist Articulation \t", getWristArticulation());
        return telemetryMap;
    }

    @Override
    public String getTelemetryName(){return "INTAKE: Arm, Claw and Wrist";}

    @Override
    public void stop(){
        clawArm.setPower(0);
    }
    public void openClaw () {
        clawSpan.setPosition(OPENCLAW);
    }
    public void closeClaw () {
            clawSpan.setPosition(CLOSECLAW);

    }
    public void clawArmLift () {
        if(clawArm.getCurrentPosition() < 750) {
            clawArm.setTargetPosition(clawArm.getCurrentPosition() + 75);
        }
    }
    public void clawArmLower () {
        if( clawArm.getCurrentPosition() > 0) {
            clawArm.setTargetPosition(clawArm.getCurrentPosition() - 35);
        }
    }
    public void armWristIn() {
        if (clawArm.getCurrentPosition() < 750) {
            clawWrist.setPosition(WRIST_MAX);
        }
        else if(clawArm.getCurrentPosition() > 750) {
            clawWrist.setPosition(WRIST_SCORE_1);
        }
        wristArticulation = WristArticulation.WRIST_IN;

    }
    public void armWristOut() {
        if (clawArm.getCurrentPosition() < 400) {
            clawWrist.setPosition(WRIST_MIN);
        }
        if (clawArm.getCurrentPosition() > 400){
            clawWrist.setPosition(WRIST_SCORE_1);
        }
        wristArticulation = WristArticulation.WRIST_OUT;

    }
    public void armPositionTest() {
        clawArm.setTargetPosition(90);
    }

    @Override
    public void update(Canvas fieldOverlay){clawArm.getVelocity();}


    public void inTake (boolean press) {
        if(press == true){
            clawArm.setTargetPosition(145);
        }
    }

    public WristArticulation getWristArticulation() {
        return wristArticulation;
    }
}

