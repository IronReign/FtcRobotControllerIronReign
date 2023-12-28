package org.firstinspires.ftc.teamcode.robots.bobobot.RoadRunning;

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
        telemetryMap.put("X in Field Coordinates", pose.position.x / Constants.FIELD_INCHES_PER_GRID);
        telemetryMap.put("Y in Field Coordinates", pose.position.y / Constants.FIELD_INCHES_PER_GRID);
        telemetryMap.put("X in Inches", pose.position.x);
        telemetryMap.put("Y in Inches", pose.position.y);
        telemetryMap.put("Heading", Math.toDegrees(pose.heading.log()));
        telemetryMap.put("Left Odometry Pod:\t", leftFront.getCurrentPosition());
        telemetryMap.put("Right Odometry Pod:\t", rightFront.getCurrentPosition());
        telemetryMap.put("Cross Odometry Pod:\t", rightBack.getCurrentPosition());
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "DRIVETRAIN";
    }


}