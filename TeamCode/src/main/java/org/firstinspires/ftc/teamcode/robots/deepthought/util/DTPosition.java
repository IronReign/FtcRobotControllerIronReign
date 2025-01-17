package org.firstinspires.ftc.teamcode.robots.deepthought.util;

import com.acmerobotics.roadrunner.Pose2d;

import java.io.Serializable;

public class DTPosition implements Serializable {
    private static final long serialVersionUID = 12345L;
    private int shoulderPosition;
    private long timestamp;
    private double chassisX;
    private double chassisY;
    private double chassisHeading;
    private int slidePosition;

    public DTPosition() {
        timestamp = System.currentTimeMillis();
        chassisX = 0;
        chassisY = 0;
        chassisHeading = 0;
    }

    public DTPosition(Pose2d driveTrainPose, int shoulderPosition, int slidePosition) {
        chassisX = driveTrainPose.position.x;
        chassisY = driveTrainPose.position.y;
        chassisHeading = driveTrainPose.heading.log();
        this.shoulderPosition = shoulderPosition;
        timestamp = System.currentTimeMillis();
        this.slidePosition = slidePosition;
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
    public int getShoulderPosition() { return shoulderPosition; }
    public int getSlidePosition() { return slidePosition; }
    public long getTimestamp() { return timestamp; }

    public String toString() {
        return "X: " + chassisX / Constants.FIELD_INCHES_PER_GRID + " Y: " + chassisY / Constants.FIELD_INCHES_PER_GRID + " Heading: " + Math.toDegrees(chassisHeading);
    }

}