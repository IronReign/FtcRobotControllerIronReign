package org.firstinspires.ftc.teamcode.robots.deepthought.util;

import com.acmerobotics.roadrunner.Pose2d;

import java.io.Serializable;

public class DTPosition implements Serializable {
    private static final long serialVersionUID = 12345L;
    private int cranePosition;
    private long timestamp;
    private double chassisX;
    private double chassisY;
    private double chassisHeading;


    public DTPosition() {
        timestamp = System.currentTimeMillis();
        chassisX = 0;
        chassisY = 0;
        chassisHeading = 0;
    }

    public DTPosition(Pose2d driveTrainPose, int cranePosition) {
        chassisX = driveTrainPose.position.x;
        chassisY = driveTrainPose.position.y;
        chassisHeading = driveTrainPose.heading.log();
        this.cranePosition = cranePosition;
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
    public int getCranePosition() { return cranePosition; }
    public long getTimestamp() { return timestamp; }

    public String toString() {
        return "X: " + chassisX / Constants.FIELD_INCHES_PER_GRID + " Y: " + chassisY / Constants.FIELD_INCHES_PER_GRID + " Heading: " + Math.toDegrees(chassisHeading);
    }

}