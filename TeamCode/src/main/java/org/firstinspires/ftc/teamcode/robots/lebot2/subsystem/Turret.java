package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem;


import static org.firstinspires.ftc.teamcode.robots.lebot2.Lebot2_6832.robot;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.robots.lebot2.Missions;
import org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.TankDrivePinpoint;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.drivetrain.DriveTrainBase;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "Lebot2_Turret")
public class Turret implements Subsystem {

    public static double NEW_P = 400;
    public static double NEW_I = 0;
    public static double NEW_D = 0;
    public static double NEW_F = 20;

    public static double CENTERING_MAX_SPEED = 1;

    //values for assuming red side
    public static double MAX_HEADING = 180;
    public static double MIN_HEADING = 90;

    public Vision vision = null;
    //public DriveTrainBase driveTrain = null;

    // ==================== VISION PID PARAMS ====================
    public static double TOLERANCE = 0;
    public static PIDCoefficients VISION_PID = new PIDCoefficients(0.04, 0.001, 2.0);
    public static double VISION_OFFSET = 0; // offset from center of target in LLResult x units
    public static double VISION_TOLERANCE = 1; // degrees of tx
    public static double VISION_INTEGRAL_CUTIN = 3.0; // degrees
    public static double VISION_ALPHA = .5; // EMA alpha for vision PID

    private final PIDController visionPID;

    private double turnMaxSpeed = 1.0;


    private final DcMotorEx turret;

    public enum Behavior {
        IDLE,        //manual
        TARGETING    //turret alignment

    }
    private Behavior behavior = Behavior.IDLE;

    public enum TurnState {
        IDLE,
        TURNING_TO_HEADING,
        TARGETING        // Combined heading centering + distance control
    }

    private TurnState turnState = TurnState.IDLE;
    private double turnTarget = 0;
    private double cachedHeading = 0;

    public static boolean reverseTurret = true;

    private TankDrivePinpoint driveTrain;

    public static double idlePower = 0;


    public Turret(HardwareMap hardwareMap) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        if(reverseTurret){
            turret.setDirection(DcMotorEx.Direction.REVERSE);
        }

        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        //driveTrain = robot.driveTrain;

        visionPID = new PIDController(VISION_PID);
        visionPID.setInputRange(-20, 20);
        visionPID.setOutputRange(-1, 1);
        visionPID.setIntegralCutIn(VISION_INTEGRAL_CUTIN);
        visionPID.setContinuous(false);
        visionPID.setTolerance(VISION_TOLERANCE / 360 * 40); // degrees to percentage of input range
        visionPID.setEmaAlpha(VISION_ALPHA);
        visionPID.enable();

    }

    @Override
    public void readSensors() {
        // PHASE 1: Motor velocity is SDK bulk-cached, no I2C read needed
        // We read currentSpeed in calc() from the bulk cache
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        switch (behavior){
            case IDLE:
                handleIdle();
                break;
            case TARGETING:
                handleTargeting();
                break;
        }
    }

    public void handleTargeting(){
        switch (turnState) {
            case IDLE:
                executeIdle();
                break;
            case TURNING_TO_HEADING:
                executeHeadingTurn();
                break;
            case TARGETING:
                executeTrackingTarget();
                break;
        }
    }

    public void handleIdle(){
        turret.setPower(idlePower);
    }
    public void setPower(double power){
        idlePower = power;
    }


    public void setVision(Vision vision) {
        this.vision = vision;
    }
    public void setDriveTrain(TankDrivePinpoint driveTrain) {
        this.driveTrain = driveTrain;
    }


    public void setIdle(){
        turnState = TurnState.IDLE;
        behavior = Behavior.IDLE;
    }
    public void setTracking(){
        turnState = TurnState.TARGETING;
        behavior = Behavior.TARGETING;
    }

    private void executeIdle(){
        turret.setPower(0);
    }

    private void executeHeadingTurn() {

        if (driveTrain.getPose().heading.toDouble() >= Math.toRadians(MAX_HEADING) || driveTrain.getPose().heading.toDouble() <= Math.toRadians(MIN_HEADING)) {
            turret.setPower(0);
            return;

        } else {
            turnState = TurnState.TARGETING;
        }

    }

    private void executeTrackingTarget() {
        if (vision == null || !vision.hasTarget()) {
            // Lost target - stop motors but stay in tracking mode (will resume when target reappears)
            turret.setPower(0);
            return;
        }
//        if (driveTrain.getPose().heading.toDouble() >= Math.toRadians(MAX_HEADING) || driveTrain.getPose().heading.toDouble() <= Math.toRadians(MIN_HEADING)) {
//            turret.setPower(0);
//            turnState = TurnState.TURNING_TO_HEADING;
//            return;
//
//        }
        turnMaxSpeed = CENTERING_MAX_SPEED;
        turnState = TurnState.TARGETING;
        visionPID.enable();


        double tx = vision.getTx();

        // Negate tx so positive tx (target right) produces positive correction (turn right)
        visionPID.setInput(tx);
        visionPID.setSetpoint(VISION_OFFSET);
        visionPID.setOutputRange(-turnMaxSpeed, turnMaxSpeed);
        visionPID.setPID(VISION_PID);

        double correction = visionPID.performPID();

        if (visionPID.lockedOnTarget() || Math.abs(vision.getTx())<TOLERANCE) {
            turret.setPower(0);
        }else{
            turret.setPower(correction);
        }
    }

    @Override
    public void stop(){

    }
    @Override
    public void resetStates(){

    }

    @Override
    public void act() {
        // PHASE 3: No-op for drive motors
        // RoadRunner Actions write motors directly during trajectory following
        // Turn PID writes motors in calc() for immediate response
    }
    @Override
    public String getTelemetryName() {
        return "Turret";
    }
    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();
        if(vision.hasTarget()){
            telemetry.put("tx: ", vision.getTx());
        }else{
            telemetry.put("No vision: ", null);
        }
        telemetry.put("turn state: ", turnState);
        telemetry.put("behavior: ", behavior);
        telemetry.put("turret power: ", idlePower);


        return telemetry;
    }
}




