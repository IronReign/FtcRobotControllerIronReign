package org.firstinspires.ftc.teamcode.robots.csbot.util;

import com.acmerobotics.roadrunner.Pose2d;

import java.io.Serializable;

public class CSPosition implements Serializable {
    private static final long serialVersionUID = 12345L;
    private long timestamp;
    private double chassisX;
    private double chassisY;
    private double chassisHeading;

    public CSPosition() {
        timestamp = System.currentTimeMillis();
        chassisX = 0;
        chassisY = 0;
        chassisHeading = 0;

    }
    public CSPosition(Pose2d driveTrainPose) {
        chassisX = driveTrainPose.position.x;
        chassisY = driveTrainPose.position.y;
        chassisHeading = driveTrainPose.heading.log();
        timestamp = System.currentTimeMillis();
    }
    public void updateTime() { timestamp = System.currentTimeMillis(); }
    public Pose2d getPose(){
        return new Pose2d(chassisX, chassisY, chassisHeading);
    }
    public void setPose(Pose2d pose){
        this.chassisX = pose.position.x;
        this.chassisY = pose.position.y;
        this.chassisHeading = pose.heading.log();
    }
    public long getTimestamp() { return timestamp; }

    public String toString() {
        return "X: " + chassisX / Constants.FIELD_INCHES_PER_GRID + " Y: " + chassisY / Constants.FIELD_INCHES_PER_GRID + " Heading: " + Math.toDegrees(chassisHeading);
    }
}