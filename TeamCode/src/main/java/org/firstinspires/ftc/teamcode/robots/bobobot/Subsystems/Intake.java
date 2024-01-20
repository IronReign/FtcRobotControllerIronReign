package org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
    public static double OPENCLAW = 0.55;
    public static double CLOSECLAW = 0.85;
    public static double WRIST_MIN = Utils.servoNormalize(1415);
    public static double WRIST_INIT_POSITION = Utils.servoNormalize(2105);
    public static double WRIST_MAX = Utils.servoNormalize(1720);
    public static double WRIST_SCORE_1 = Utils.servoNormalize(1700);
    private boolean initalized;

    public enum WristArticulation{
        WRIST_IN, WRIST_OUT, WRIST_SCORE;
    }

    public enum ArmState{
        GROUND, BACKDROP, TRUSS, RESET;
    }
    public ArmState armState;
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
      //clawArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      //clawArm.setPower(-.1);
      clawArm.setPower(1);
      clawArm.setTargetPosition(40);
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
        telemetryMap.put("Arm State \t", getArmState());
        telemetryMap.put("Arm Motor Current \t", clawArm.getCurrent(CurrentUnit.AMPS));
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
//        if(clawArm.getCurrentPosition() < 750) {
//            clawArm.setTargetPosition(clawArm.getCurrentPosition() + 75);
//        }
        clawArm.setPower(1);
        clawArm.setVelocity(350);
        clawArm.setTargetPosition(475);
        armState = ArmState.BACKDROP;
    }
    public void clawArmLower (){
//        if( clawArm.getCurrentPosition() > 0) {
//            clawArm.setTargetPosition(clawArm.getCurrentPosition() - 35);
//        }
        //armWristOut();
        clawArm.setVelocity(150);
        clawArm.setTargetPosition(0);
        clawArm.setPower(0);
        armState = ArmState.GROUND;
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
        if (clawArm.getCurrentPosition() > 400){
            clawWrist.setPosition(WRIST_SCORE_1);
            wristArticulation = WristArticulation.WRIST_SCORE;
        }


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
    public void armTrussLift(){
        clawArm.setPower(1);
        clawArm.setTargetPosition(300);
        armState = ArmState.TRUSS;
    }

    public void reset(){
        clawArm.setTargetPosition(0);
        armWristIn();
        armState = ArmState.RESET;
    }
    public WristArticulation getWristArticulation() {
        return wristArticulation;
    }
    public ArmState getArmState(){ return armState; }

    public void init_loop(){
//        if(!initalized && clawArm.getCurrent(CurrentUnit.AMPS) > 3) {
            initalized = true;
            clawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            clawArm.setTargetPosition(0);
            clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            clawArm.setPower(1);
//        }
    }
}

