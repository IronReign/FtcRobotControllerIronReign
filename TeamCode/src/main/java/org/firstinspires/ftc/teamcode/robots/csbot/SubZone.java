package org.firstinspires.ftc.teamcode.robots.csbot;

import static org.firstinspires.ftc.teamcode.robots.csbot.util.Constants.FIELD_INCHES_PER_GRID;

import com.acmerobotics.roadrunner.Pose2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class SubZone {
    public static SubZone PICKUP = new SubZone(Field.MIN_X_VALUE, -1.5, 1.5, Field.MAX_Y_VALUE, "PICKUP", Field.Zone.AUDIENCE);
    public static SubZone WING = new SubZone(Field.MIN_X_VALUE, -1.5, 1.5, Field.MAX_Y_VALUE, "WING", Field.Zone.AUDIENCE);
    public static SubZone BACKDROP = new SubZone(1, Field.MAX_X_VALUE, Field.MIN_Y_VALUE, 0, "BACKDROP", Field.Zone.BACKSTAGE);

    SubZone(double xmin, double xmax, double ymin, double ymax, String name, Field.Zone parent) {
        this.parent = parent;
        this.name = name;
        this.x1 = xmin;
        this.x2 = xmax;
        this.y1 = ymin;
        this.y2 = ymax;
    }

    public double x1;
    public double x2;
    public double y1;
    public double y2;
    public final String name;

    public final Field.Zone parent;

    public SubZone flipOnX() {
        return new SubZone(this.x1, this.x2, -this.y1, -this.y2, this.name, this.parent);
    }

    public boolean withinSubZone(Pose2d pose) {
        List<Double> xVals = Arrays.asList(pose.position.x / FIELD_INCHES_PER_GRID, x1, x2);
        List<Double> yVals = Arrays.asList(pose.position.y / FIELD_INCHES_PER_GRID, y1, y2);
        if (
            //if the pose is an extrema of either list, it's outside the bounds
                !(
                        Collections.min(xVals) == pose.position.x / FIELD_INCHES_PER_GRID || Collections.max(xVals) == (pose.position.x / FIELD_INCHES_PER_GRID) ||
                                Collections.min(yVals) == (pose.position.y / FIELD_INCHES_PER_GRID) || Collections.max(yVals) == (pose.position.y / FIELD_INCHES_PER_GRID)
                )
        ) {
            return true;
        }
        return false;
    }

    public static ArrayList<SubZone> getNamedSubZones(boolean isRed) {
        ArrayList<SubZone> temp = new ArrayList<>();
        if (isRed) {
            temp.addAll(Arrays.asList(PICKUP, WING, BACKDROP));
        } else {
            temp.addAll(Arrays.asList(PICKUP.flipOnX(), WING.flipOnX(), BACKDROP.flipOnX()));
        }
        return temp;
    }

}
