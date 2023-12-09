package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;


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
import org.firstinspires.ftc.teamcode.robots.csbot.rr_stuff.MecanumDrive;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;

import java.util.HashMap;
import java.util.Map;

@Config(value = "CS_ROADRUNNER")
public class DriveTrain extends MecanumDrive implements Subsystem {
    public Robot robot;
    public boolean trajectoryIsActive;

    public Articulation articulation;

    public DistanceSensor backDistanceSensor;
    public double backDistanceSensorValue = 0;
    public enum Articulation{
        BACKSTAGE_DRIVE,
        WING_DRIVE,
    }


    public DriveTrain(HardwareMap hardwareMap, Robot robot, boolean simulated) {
        super(hardwareMap, new Pose2d(0, 0, 0));
        this.robot = robot;
        trajectoryIsActive = false;
        backDistanceSensor = hardwareMap.get(DistanceSensor.class, "backDist");

    }
    //end constructor



    @Override
    public void update(Canvas c) {
        backDistanceSensorValue = backDistanceSensor.getDistance(DistanceUnit.INCH);
        updatePoseEstimate();
    }

    public void drive(double x, double y, double theta) {
        trajectoryIsActive = false;
        setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -y,
                        -x
                ),
                theta
        ));
        updatePoseEstimate();
    }

    public void fieldOrientedDrive(double x, double y, double theta){
        Vector2d input = new Vector2d(
                -y * Math.cos(-pose.heading.log()) + x * Math.sin(-pose.heading.log()),
                -y * Math.sin(-pose.heading.log()) - x * Math.cos(-pose.heading.log())
        );


        setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(
                        input.x,
                        input.y
                        ),
                        theta
                )
        );
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
    public void stop() {
        drive(0, 0, 0);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {

        Map<String, Object> telemetryMap = new HashMap<>();
        telemetryMap.put("x in fieldCoords", pose.position.x / Constants.FIELD_INCHES_PER_GRID);
        telemetryMap.put("y in fieldCoords", pose.position.y / Constants.FIELD_INCHES_PER_GRID);
        telemetryMap.put("x in inches", pose.position.x);
        telemetryMap.put("y in inches", pose.position.y);
        telemetryMap.put("Back Distance Sensor Value", backDistanceSensorValue);
        telemetryMap.put("heading", Math.toDegrees(pose.heading.log()));
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