package org.firstinspires.ftc.teamcode.robots.csbot;

import static org.firstinspires.ftc.teamcode.robots.csbot.util.Constants.FIELD_INCHES_PER_GRID;
import static org.firstinspires.ftc.teamcode.robots.csbot.util.Utils.P2D;

import com.acmerobotics.roadrunner.Pose2d;

public class POI {
    //TODO - does this need to be diff for individual pois?
    //also needs actual testing
    public static double POI_ERROR_RADIUS = .25;

    public static POI WING_INTAKE = new POI(-2.1, 1.1, 90, "WING_INTAKE", Field.Zone.AUDIENCE);
    public static POI HANG_PREP = new POI(.6, -1.5, 180, "HANG_PREP", Field.Zone.BACKSTAGE);
    public static POI HANG = new POI(-0.5, -1.5, 180, "HANG", Field.Zone.RIGGING);
    public static POI SCORE = new POI(1.65, -1.5, 180, "SCORE", Field.Zone.BACKSTAGE);
    public static POI APRILTAG1 = new POI(61.728, -40.344, 180, "APRILTAG1", Field.Zone.BACKSTAGE);
    public static POI APRILTAG2 = new POI(61.728, -35.344, 180, "APRILTAG2", Field.Zone.BACKSTAGE);
    public static POI APRILTAG3 = new POI(61.728, -29.344, 180, "APRILTAG3", Field.Zone.BACKSTAGE);
    public static POI APRILTAG4 = new POI(61.728, 29.344, 180, "APRILTAG4", Field.Zone.BACKSTAGE);
    public static POI APRILTAG5 = new POI(61.728, 35.344, 180, "APRILTAG5", Field.Zone.BACKSTAGE);
    public static POI APRILTAG6 = new POI(61.728, 40.344, 180, "APRILTAG6", Field.Zone.BACKSTAGE);


    POI(double x, double y, double heading, String name, Field.Zone parent) {
        this.parent = parent;
        this.name = name;
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.pose = P2D(x, y, heading);
    }

    public Pose2d pose;
    public double heading;
    public double x;

    public double y;

    public final String name;

    public final Field.Zone parent;

    public POI flipOnX() {
        return new POI(this.x, -this.y, -this.heading, this.name, this.parent);
    }

    public boolean atPOI(Pose2d pose) {
        return Math.hypot(pose.position.x / FIELD_INCHES_PER_GRID - x, pose.position.y / FIELD_INCHES_PER_GRID - y) < POI_ERROR_RADIUS;
    }


    public Field.Zone getZone() {
        return parent;
    }
}
