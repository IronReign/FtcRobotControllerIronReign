package org.firstinspires.ftc.teamcode.robots.csbot.util;

import com.acmerobotics.roadrunner.Pose2d;

import java.io.Serializable;

public class CSPosition implements Serializable {
    private static final long serialVersionUID = 12345L;
    private long timestamp;
    private double chassisX;
    private double chassisY;
    private double chassisHeading;
    private int skyhookRightPos, skyhookLeftPos;

    public CSPosition() {
        timestamp = System.currentTimeMillis();
        chassisX = 0;
        chassisY = 0;
        chassisHeading = 0;
        skyhookLeftPos = 0;
        skyhookRightPos = 0;
    }
    public CSPosition(Pose2d driveTrainPose) {
        chassisX = driveTrainPose.position.x;
        chassisY = driveTrainPose.position.y;
        chassisHeading = driveTrainPose.heading.log();
        timestamp = System.currentTimeMillis();
        skyhookLeftPos = 0;
        skyhookRightPos = 0;
    }
    public CSPosition(Pose2d driveTrainPose, int skyhookLeft, int skyhookRight) {
        chassisX = driveTrainPose.position.x;
        chassisY = driveTrainPose.position.y;
        chassisHeading = driveTrainPose.heading.log();
        timestamp = System.currentTimeMillis();
        skyhookLeftPos = skyhookLeft;
        skyhookRightPos = skyhookRight;
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

    public int getSkyhookLeftTicks() {
        return skyhookLeftPos;
    }

    public int getSkyhookRightTicks() {
        return skyhookRightPos;
    }
}