package org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.bobobot.RoadRunning.MecanumDrive;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;
import java.util.HashMap;
import java.util.Map;

@Config(value = "BoadBunner")
public class DriveTrain extends MecanumDrive implements Subsystem {
    public RunnerBot runnerBot;


    public DriveTrain(HardwareMap hardwareMap, RunnerBot runnerBot) {
        super(hardwareMap, new Pose2d(0, 0, 0));
        this.runnerBot = runnerBot;
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
    }

    @Override
    public void stop() {
        drive(0, 0, 0);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {

        Map<String, Object> telemetryMap = new HashMap<>();
        telemetryMap.put("X in Field Coordinates \t", pose.position.x / Constants.FIELD_INCHES_PER_GRID);
        telemetryMap.put("Y in Field Coordinates \t", pose.position.y / Constants.FIELD_INCHES_PER_GRID);
        telemetryMap.put("X in Inches \t", pose.position.x);
        telemetryMap.put("Y in Inches \t", pose.position.y);
        telemetryMap.put("Heading \t", Math.toDegrees(pose.heading.log()));
        telemetryMap.put("Left Odometry Pod \t", leftFront.getCurrentPosition());
        telemetryMap.put("Right Odometry Pod \t", rightFront.getCurrentPosition());
        telemetryMap.put("Cross Odometry Pod \t", rightBack.getCurrentPosition());
        telemetryMap.put("Speed Mode On? \t", isSlowed());
        telemetryMap.put("Speed \t", robotSpeed);
        telemetryMap.put("Average Motor Position \t", getMotorAvgPosition());
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
            case 0:
                robotSpeed = 1;
                mode++;
                speedModeIsOn = true;
                break;
            case 1:
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

    public boolean getDebug(){
        return debug;
    }
}