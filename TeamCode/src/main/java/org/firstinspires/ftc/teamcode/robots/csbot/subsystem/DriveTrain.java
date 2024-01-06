package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.alliance;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//todo this should not reference the reign version of MecanumDrive
import org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832;
import org.firstinspires.ftc.teamcode.robots.csbot.rr_stuff.MecanumDrive;
//todo this should not reference reign's Constants
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
    public double imuRoadrunnerError;
    public double imuAngle;

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
        imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + (alliance.getMod()? -90 : 90);
        imuRoadrunnerError = imuAngle - Math.toDegrees(pose.heading.log());
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

    public void fieldOrientedDrive(double x, double y, double theta, boolean isRed){
        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                x,
               y);
        //additional rotation needed if blue alliance perspective
        Rotation2d heading =(!isRed) ? pose.heading: pose.heading.plus(Math.PI);
        input = heading.inverse().times(
                new Vector2d(-input.x, input.y));
        setDrivePowers(new PoseVelocity2d(input,-theta ));
        updatePoseEstimate();
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
        telemetryMap.put("imu vs roadrunner:\t", imuRoadrunnerError);
        telemetryMap.put("imu:\t", imuAngle);

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "DRIVETRAIN";
    }
}