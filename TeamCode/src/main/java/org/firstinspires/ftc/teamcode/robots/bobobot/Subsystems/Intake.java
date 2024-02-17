package org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Joint;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;

import java.util.HashMap;
import java.util.Map;

@Config("BobotIntake")
public class Intake implements Subsystem {
    private DcMotorEx clawArm = null;
    private Servo clawSpan = null;
    private Servo clawWrist = null;
    private Joint wrist = null;
    public static double OPENCLAW = Utils.servoNormalize(900);
    public static double CLOSECLAW = Utils.servoNormalize(1500);
    public static double WRIST_MIN = Utils.servoNormalize(1597);
    public static double denormMin = Utils.servoDenormalize(WRIST_MIN);
    public static double WRIST_MAX = Utils.servoNormalize(899);
    public static double denormMax = Utils.servoDenormalize(WRIST_MAX);
    public static double WRIST_SCORE_1 = Utils.servoNormalize(1730);
    public static double WRIST_PIXEL = WRIST_SCORE_1;

    public static double MOTOR_INIT_PWR = -.1;
    private boolean initalized;

    public enum WristArticulation{
        WRIST_IN, WRIST_OUT, WRIST_SCORE, PIXEL_GRAB, INIT
    }

    public enum ArmState{
        GROUND, BACKDROP, TRUSS, RESET, INIT, SCORING
    }

    public enum ClawState{
        INIT, OPEN, CLOSED
    }
    public ArmState armState;
    public ClawState clawState;
    public WristArticulation wristArticulation;
    private HardwareMap hardwareMap;
    public RunnerBot runnerBot;
    public Intake(HardwareMap hardwareMap, RunnerBot runnerBot) {
        this.hardwareMap = hardwareMap;
        this.runnerBot = runnerBot;
        clawArm = this.hardwareMap.get(DcMotorEx.class, "clawArm");
        clawSpan = this.hardwareMap.get(Servo.class, "clawSpan");
        clawWrist = this.hardwareMap.get(Servo.class, "clawWrist");
        initalized = false;
        clawArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawArm.setPower(MOTOR_INIT_PWR);
        clawState = ClawState.INIT;
        armState = ArmState.INIT;
        wristArticulation = WristArticulation.INIT;
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
        telemetryMap.put("Arm State \t", getArmState());
        telemetryMap.put("Claw State \t", getClawState());
        telemetryMap.put("Arm Motor Current \t", clawArm.getCurrent(CurrentUnit.AMPS));
        telemetryMap.put("Arm Motor Power \t", clawArm.getPower());
        telemetryMap.put("Arm Motor Mode \t", clawArm.getMode());
        return telemetryMap;
    }

    @Override
    public String getTelemetryName(){return "INTAKE: Arm, Claw and Wrist";}

    @Override
    public void stop(){
        clawArm.setPower(0);
    }
    public void openClaw () {clawSpan.setPosition(OPENCLAW);}
    long testTime = 0;
    public void closeClaw () {
        clawSpan.setPosition(CLOSECLAW);
            wristArticulation = WristArticulation.PIXEL_GRAB;
            clawState = ClawState.CLOSED;
    }
    public void clawArmLift () {
    if (clawArm.getCurrentPosition() < 500) {
        clawArm.setPower(1);
        clawArm.setVelocity(350);
        clawArm.setTargetPosition(475);
        armState = ArmState.BACKDROP;
    }
    }
    public void clawArmLower (){
    if (clawArm.getCurrentPosition() > 0) {
        clawArm.setVelocity(300);
        clawArm.setTargetPosition(10);
        clawArm.setPower(0);
        armState = ArmState.GROUND;
    }
    }
    public void armWristIn() {
        if (clawArm.getCurrentPosition() < 750) {
            clawWrist.setPosition(WRIST_MAX);
            wristArticulation = WristArticulation.WRIST_IN;
        }
    }
    public void armWristOut() {
        if (clawArm.getCurrentPosition() < 400) {
            clawWrist.setPosition(WRIST_MIN);
            wristArticulation = WristArticulation.WRIST_OUT;
        }
        if (clawArm.getCurrentPosition() > 350){
            clawWrist.setPosition(WRIST_SCORE_1);
            wristArticulation = WristArticulation.WRIST_SCORE;
        }


    }

    @Override
    public void update(Canvas fieldOverlay){clawArm.getVelocity();}

    public void armScoreLift(){
        if(clawArm.getCurrentPosition() < 400) {
            clawArm.setTargetPosition(clawArm.getCurrentPosition() + 35);
            armState = ArmState.SCORING;
        }
    }

    public void armScoreLower(){
        if(clawArm.getCurrentPosition() > 0) {
            clawArm.setTargetPosition(clawArm.getCurrentPosition() - 45);
            armState = ArmState.SCORING;
        }
    }

    public void reset(){
        clawArm.setTargetPosition(0);
        armWristIn();
        armState = ArmState.RESET;
    }
    public WristArticulation getWristArticulation() {
        return wristArticulation;
    }
    public ClawState getClawState(){return clawState; }
    public ArmState getArmState(){ return armState; }

    public void init_loop(){
        if(!initalized && clawArm.getCurrent(CurrentUnit.AMPS) > 1) {
            initalized = true;
            clawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            clawArm.setTargetPosition(0);
            clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            clawArm.setPower(1);
        }
    }
    public boolean isPressActive = false;

}

