package org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems;


import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.alliance;
import static org.firstinspires.ftc.teamcode.robots.csbot.util.Utils.wrapAngle;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robots.bobobot.DebugBot;
import org.firstinspires.ftc.teamcode.robots.bobobot.RoadRunning.MecanumDrive;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.bobobot.Utilities.Constants;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.HashMap;
import java.util.Map;

@Config(value = "BoadBunner")
public class DriveTrain extends MecanumDrive implements Subsystem {
    public RunnerBot runnerBot;
    public DebugBot debugBot;
    public double imuAngle;
    public double imuAngleP;
    public double imuAngleR;
    public double targetHeading;
    public static PIDController headingPID;
    public static PIDCoefficients HEADING_PID_PWR = new PIDCoefficients(0, .1854, 0);
    public static double PID_I = .4;
    public static double PID_P = 0;
    public static double PID_D = .75;
    public static int PurpleTurn = 90;
    public static double HEADING_PID_TOLERANCE = .10; //this is a percentage of the input range .063 of 2PI is 1 degree
    private double PIDCorrection, PIDError;
    public static int turnToTest = 0;
    public static double turnToSpeed= .8;
    public static Constants.Position gamePosition = Constants.Position.START_RIGHT_RED;
    public static Constants.Alliance alliance = Constants.Alliance.RED;
    public int spikeIndex = 1;
    public int turn = 0;
    public DriveTrain(HardwareMap hardwareMap, RunnerBot runnerBot) {
        super(hardwareMap, new Pose2d(0, 0, 0));
        this.runnerBot = runnerBot;
        headingPID = new PIDController(HEADING_PID_PWR);
        headingPID.setInputRange(0, 360);
        headingPID.setOutputRange(-1, 1);
        headingPID.setIntegralCutIn(4);
        headingPID.setContinuous(true);
        headingPID.setTolerance(HEADING_PID_TOLERANCE);
        headingPID.enable();
    }

    public DriveTrain(HardwareMap hardwareMap, DebugBot debugBot) {
        super(hardwareMap, new Pose2d(0, 0, 0));
        this.debugBot = debugBot;
        headingPID = new PIDController(HEADING_PID_PWR);
        headingPID.setInputRange(0, 360);
        headingPID.setOutputRange(-1, 1);
        headingPID.setIntegralCutIn(4);
        headingPID.setContinuous(true);
        headingPID.setTolerance(HEADING_PID_TOLERANCE);
        headingPID.enable();
    }



    public void drive(double x, double y, double theta) {
        setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -y,
                        -x
                ),
                -theta
        ));
        updatePoseEstimate();
    }

    public void changeIndex(){
        switch(spikeIndex){
            case 1:
                spikeIndex++;
                turn = 0;
                break;
            case 2:
                spikeIndex++;
                turn = -PurpleTurn;
                break;
            case 3:
                spikeIndex = 1;
                turn = PurpleTurn;
                break;
        }
    }

    public int getTurn(){
        return turn;
    }
    public int getSpikeIndex(){
        return spikeIndex;
    }
    public void strafe() {
        Pose2d startPosition = pose;
        Actions.runBlocking(
                new SequentialAction(
                        actionBuilder(pose)
                                .strafeTo(new Vector2d(startPosition.position.x, startPosition.position.y + Constants.FIELD_INCHES_PER_GRID))
                                .build()
                )
        );



    }

    public void setPose(Constants.Position start) {
        pose = start.getPose();
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    @Override
    public void update(Canvas fieldOverlay) {
        updatePoseEstimate();
        imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        imuAngleP = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
        imuAngleR = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
        if (turnToTest!=0) turnUntilDegreesIMU(turnToTest,turnToSpeed);
    }

    @Override
    public void stop() {
        drive(0, 0, 0);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {

        Map<String, Object> telemetryMap = new HashMap<>();
        telemetryMap.put("IMU Angle", imuAngle);
        telemetryMap.put("IMU Angle Pitch", imuAngleP);
        telemetryMap.put("IMU Angle Roll", imuAngleR);
        telemetryMap.put("X in Field Coordinates \t", pose.position.x / Constants.FIELD_INCHES_PER_GRID);
        telemetryMap.put("Y in Field Coordinates \t", pose.position.y / Constants.FIELD_INCHES_PER_GRID);
        telemetryMap.put("Game Init Position \t",getGamePosition());
        telemetryMap.put("X in Inches \t", pose.position.x);
        telemetryMap.put("Y in Inches \t", pose.position.y);
        telemetryMap.put("Heading \t", Math.toDegrees(pose.heading.log()));
        telemetryMap.put("Left Odometry Pod \t", leftFront.getCurrentPosition());
        telemetryMap.put("Right Odometry Pod \t", rightFront.getCurrentPosition());
        telemetryMap.put("Cross Odometry Pod \t", rightBack.getCurrentPosition());
        telemetryMap.put("Speed Mode On? \t", isSlowed());
        telemetryMap.put("Speed \t", robotSpeed);
        telemetryMap.put("Average Motor Position \t", getMotorAvgPosition());
        telemetryMap.put("Left Front Motor Power", leftFront.getPower());
        telemetryMap.put("Left Back Motor Power", leftBack.getPower());
        telemetryMap.put("Right Front Motor Power", rightFront.getPower());
        telemetryMap.put("Right Back Motor Power", rightBack.getPower());
        telemetryMap.put("Total Distance Traveled in Inches \t", Math.hypot(pose.position.x, pose.position.y));
        telemetryMap.put("Total Distance Traveled in Field Units \t", Math.hypot(pose.position.x/ Constants.FIELD_INCHES_PER_GRID, pose.position.y/ Constants.FIELD_INCHES_PER_GRID));
        telemetryMap.put("PID Error:\t", PIDError);
        telemetryMap.put("PID Correction:\t", PIDCorrection);
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "DRIVETRAIN";
    }

    int mode = 0;
    double robotSpeed = 0.45;
    private boolean speedModeIsOn = false;
    public void modeToggle(){
        switch(mode){
            case 0: //fast
                robotSpeed = .8;
                mode++;
                speedModeIsOn = true;
                break;
            case 1: //slow
                robotSpeed = 0.45;
                mode = 0;
                speedModeIsOn = false;
                break;

        }
    }
    public boolean isSlowed(){
        return speedModeIsOn;
    }



    public double getRobotSpeed(){
        return robotSpeed;
    }
    public double getMotorAvgPosition(){return (double)(Math.abs(leftFront.getCurrentPosition())+Math.abs(rightFront.getCurrentPosition())+Math.abs(rightBack.getCurrentPosition()))/3.0;}
    public boolean debug = false;
    int debugMode = 0;
    public void debugSwitch(){
        switch(debugMode){
            case 0:
                debug = !debug;
                debugMode++;
                break;
            case 1:
                debug = !debug;
                debugMode = 0;
                break;
        }
    }
    public boolean turnUntilDegreesIMU(double turnAngle, double maxSpeed) {
        targetHeading = wrapAngle(turnAngle);
        headingPID.setPID(new PIDCoefficients(PID_P,PID_I, PID_D));
        headingPID.setInput(imuAngle);
        headingPID.setSetpoint(targetHeading);
        headingPID.setOutputRange(-maxSpeed, maxSpeed);
        headingPID.setTolerance(HEADING_PID_TOLERANCE);
        headingPID.setContinuous();
        double correction = headingPID.performPID();
        PIDCorrection = correction;
        PIDError = headingPID.getError();
        if(headingPID.onTarget()){
            //turn meets accuracy target
            //todo is this a good time to update pose heading from imu?
            //stop
            setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            return true;
        }else{
            headingPID.enable();
            setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), correction));
            return false;
        }
    }

    public boolean getDebug(){
        return debug;
    }

    public double getTargetHeading(){
        return targetHeading;
    }

    public double getImuAngle() {
        return imuAngle;
    }

    public Constants.Position getGamePosition(){
        return gamePosition;
    }

    public Constants.Alliance getAlliance() {
        return alliance;
    }
}