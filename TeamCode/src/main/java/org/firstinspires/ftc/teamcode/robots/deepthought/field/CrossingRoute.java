package org.firstinspires.ftc.teamcode.robots.deepthought.field;

import static org.firstinspires.ftc.teamcode.robots.deepthought.util.Constants.FIELD_INCHES_PER_GRID;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

class CrossingRoute {
    Pose2d audienceSide;
    Pose2d backstageSide;
    double ROUTE_HEADING_RAD;
    int index;
    Field field;

    public CrossingRoute(Pose2d audienceSide, Pose2d backstageSide, int index, Field parent) {
        this.audienceSide = audienceSide;
        this.backstageSide = backstageSide;
        //should be the same for both poses because crossing routes are linear
        ROUTE_HEADING_RAD = audienceSide.heading.log();
        this.index = index;
        this.field = parent;
    }

    //return the shortest distance between a given pose and either endpoint in field grids
    public double distanceToRoute(Pose2d pose) {
        double distanceToAudienceSide = Math.hypot(Math.abs(pose.position.x - audienceSide.position.x), Math.abs(pose.position.y - audienceSide.position.x)) / FIELD_INCHES_PER_GRID;
        double distanceToBackstageSide = Math.hypot(Math.abs(pose.position.x - backstageSide.position.x), Math.abs(pose.position.y - backstageSide.position.x)) / FIELD_INCHES_PER_GRID;
        return distanceToBackstageSide > distanceToAudienceSide ? distanceToAudienceSide : distanceToBackstageSide;
    }

    //return the closest endpoint to a given pose
    public Pose2d getEntryPose(Pose2d pose) {
        if (field.getZone(pose).equals(Field.Zone.AUDIENCE))
            return audienceSide;
        return backstageSide;
    }

    //returns the farthest endpoint to a given pose
    public Pose2d getExitPose(Pose2d pose) {
        if (field.getZone(pose).equals(Field.Zone.AUDIENCE))
            return backstageSide;
        return audienceSide;
    }

    public int getIndex() {
        return index;
    }

    //assumes robot is at the start of a crossing route and adds the whole route path to the actionBuilder
    public TrajectoryActionBuilder addToPath(TrajectoryActionBuilder actionBuilder, Pose2d startPose) {
        actionBuilder = actionBuilder.strafeTo(getExitPose(startPose).position);
        return actionBuilder;
    }

    public static boolean withinError(double value, double target, double error) {
        return (Math.abs(target - value) <= error);
    }

}
