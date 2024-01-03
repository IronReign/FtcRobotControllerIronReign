package org.firstinspires.ftc.teamcode.robots.bobobot.Utilities;

import java.io.Serializable;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;

public class BobotPosition implements Serializable {
    private static final long serialVersionUID = 12345L;
    private long timestamp;
    private double chassisX;
    private double chassisY;
    private double chassisHeading;

    public BobotPosition(){
        timestamp = System.currentTimeMillis();
        chassisX = 0;
        chassisY = 0;
        chassisHeading = 0;
    }

    public BobotPosition(Pose2d drivepose2d){
        chassisX = drivepose2d.position.x;
        chassisY = drivepose2d.position.y;
        chassisHeading = drivepose2d.heading.log();
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
