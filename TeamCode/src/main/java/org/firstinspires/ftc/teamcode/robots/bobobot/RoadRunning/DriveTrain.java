package org.firstinspires.ftc.teamcode.robots.bobobot.RoadRunning;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robots.bobobot.Bots.RunnerBot;
import org.firstinspires.ftc.teamcode.robots.csbot.rr_stuff.MecanumDrive;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;

import java.util.HashMap;
import java.util.Map;

@Config(value = "BoadBunner")
public class DriveTrain extends MecanumDrive implements Subsystem {
    public RunnerBot robot;


    public DriveTrain(HardwareMap hardwareMap, RunnerBot robot) {
        super(hardwareMap, new Pose2d(0, 0, 0));
        this.robot = robot;
    }

    public void drive(double x, double y, double theta) {
        setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -y*robotSpeed,
                        -x*robotSpeed
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
        telemetryMap.put("Cross Odometry Pod:\t", rightBack.getCurrentPosition());
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
    double robotSpeed = 0.75;
    private boolean speedPress = false;
    public void modeToggle(){
        switch(mode){
            case 0:
                robotSpeed = 1;
                mode++;
                speedPress = true;
                break;
            case 1:
                robotSpeed = 0.75;
                mode = 0;
                speedPress = false;
                break;

        }
    }
    public boolean isSlowed(){
        return speedPress;
    }

    long testTime =0;
    int testStage = 0;
    public static double motorPower = 0;
    public void runTest(){
        switch (testStage){
                case 0:
                    testTime=futureTime(2);
                    testStage++;

                case 1:
                    rightFront.setPower(motorPower);

                    if (isPast(testTime)){
                        rightFront.setPower(0);
                        testTime=futureTime(2);
                        testStage++;
                    }
                    break;
                case 2:
                    rightBack.setPower(motorPower);

                    if (isPast(testTime)){
                        rightBack.setPower(0);
                        testTime=futureTime(2);
                        testStage++;
                    }
                    break;
                case 3:
                    leftBack.setPower(motorPower);

                    if (isPast(testTime)){
                        leftBack.setPower(0);
                        testStage++;
                        testTime=futureTime(2);
                    }
                    break;
                case 4:
                    leftFront.setPower(motorPower);

                    if (isPast(testTime)){
                        leftFront.setPower(0);
                        testStage=0;
                    }
                    break;
        }
    }
    public double getMotorAvgPosition(){return (double)(Math.abs(leftFront.getCurrentPosition())+Math.abs(rightFront.getCurrentPosition())+Math.abs(leftBack.getCurrentPosition())+Math.abs(rightBack.getCurrentPosition()))/4.0;}
}