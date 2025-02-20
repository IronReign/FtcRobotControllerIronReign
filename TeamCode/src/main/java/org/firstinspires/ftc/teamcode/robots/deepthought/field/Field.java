package org.firstinspires.ftc.teamcode.robots.deepthought.field;

import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.alliance;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;

import org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Constants;
import org.firstinspires.ftc.teamcode.util.Vector2;

import java.lang.Math;
import java.util.*;

@Config(value = "ITD_FIELD")
public class Field {

    public static final double FIELD_INCHES_PER_GRID = 23.5;
    public boolean finalized = false;
    List<Zone> zones;
    public List<POI> scoreLocations = new ArrayList<POI>();

    public boolean isRed = true;

    public static int allianceMultiplier = 1;
    List<Flippable> elements = new ArrayList<>();
    //all values are in field grids
    public POI basket = new POI(-2.0, -2.0, 35, "BASKET");
    public POI basket2 = new POI(-2.0, -2.0, 35, "BASKET");
    public POI basketPrep = new POI(-2, -2, 50, "BASKET");

    public POI subAccess = new POI(-1.5, -.5, 180, "SUBACCESS");

    //neutral samples
    public POI ground1;
    public POI ground2;
    public POI ground3;

    // alliance samples - approx settings for using back plate to push samples to ozone
    public POI ground4 = new POI(2, -.5, 90, "GROUND4");
    public POI ground5 = new POI(2.25, -.5, 90, "GROUND5");
    public POI ground6 = new POI(2.6, -.5, 90, "GROUND6"); // grazing back wall

    public POI sweep1 = new POI(1, -2, 45.5, "SWEEP1");
    public POI sweep1Oz = new POI(1, -2, -40, "SWEEP1Ozone");
    public POI sweep2 = new POI(1.3, -1.7, 35, "SWEEP2");
    public POI sweep2Oz = new POI(1.3, -1.7, -65, "SWEEP2Ozone");
    public POI sweep3 = new POI(1.55, -1.45, 25, "SWEEP3"); // grazing back wall
    public POI sweep3Oz = new POI(1.55, -1.45, -80, "SWEEP3Ozone");
    public POI ozone = new POI(2.4, -1.75, -90, "OZone");
    public POI oZoneWalltake = new POI(2.0, -1.5, -90, "OZoneWalltake");
    public POI hibarPrep = new POI(0, -1.5, -90, "HiBarPrep");
    public POI hibar = new POI(0, -1.45, -90, "HiBar");
    public POI zig = new POI(1.5, -1.7, -90, "Zig"); // zig from speciminer scoring
    public POI zag = new POI(2, -.5, -90, "Zag"); // zag from zig to safely get back of robot on  far side of alliance samples


    public static final double MAX_Y_VALUE = 3;
    public static final double MAX_X_VALUE = 3;
    public static final double MIN_X_VALUE = -3;
    public static final double MIN_Y_VALUE = -3;

    public static final int STANDARD_HEADING = 180;

    public enum Zone {
        AUDIENCE(Field.MIN_X_VALUE, -1.5, Field.MIN_Y_VALUE, Field.MAX_Y_VALUE, "AUDIENCE"),
        BACKSTAGE(.5, Field.MAX_X_VALUE, Field.MIN_Y_VALUE, Field.MAX_Y_VALUE, "BACKSTAGE"),
        RIGGING(-1.5, .5, Field.MIN_Y_VALUE, Field.MAX_Y_VALUE, "RIGGING");
        public double x1;
        public double x2;
        public double y1;
        public double y2;
        public final String name;

        Zone(double x1, double x2, double y1, double y2, String name) {
            this.name = name;
            this.x1 = x1;
            this.x2 = x2;
            this.y1 = y1;
            this.y2 = y2;
        }

        public static ArrayList<Zone> getNamedZones() {
            ArrayList<Zone> temp = new ArrayList<>();
            temp.addAll(Arrays.asList(RIGGING, BACKSTAGE, AUDIENCE));
            return temp;
        }

        public boolean withinZone(Pose2d pose) {
            List<Double> xVals = Arrays.asList(pose.position.x / FIELD_INCHES_PER_GRID, x1, x2);
            List<Double> yVals = Arrays.asList(pose.position.y / FIELD_INCHES_PER_GRID, y1, y2);

            if (
                //if the pose is not an extrema of the list, it's within the bounds of the list
                    !(
                            Collections.min(xVals) == pose.position.x / FIELD_INCHES_PER_GRID || Collections.max(xVals) == (pose.position.x / FIELD_INCHES_PER_GRID) ||
                                    Collections.min(yVals) == (pose.position.y / FIELD_INCHES_PER_GRID) || Collections.max(yVals) == (pose.position.y / FIELD_INCHES_PER_GRID)
                    )
            ) {
                return true;
            }
            return false;
        }
    }

    public double bearingToPoseRad(Pose2d robotPose, Pose2d targetPose) {
        Vector2 start = new Vector2(robotPose.position.x, robotPose.position.y);
        Vector2 end = new Vector2(targetPose.position.x, targetPose.position.y);
        return start.angleBetween(end);
    }

    public double bearingToPOIRad(Pose2d robotPose, POI endPOI) {
        return bearingToPoseRad(robotPose, endPOI.getPose());
    }

    public double bearingToPOIDeg(Pose2d robotPose, POI endPOI) {
        return Math.toDegrees(bearingToPOIRad(robotPose, endPOI));
    }

    public double bearingToPoseDeg(Pose2d robotPose, Pose2d endPose) {
        return Math.toDegrees(bearingToPoseRad(robotPose, endPose));
    }

    public Field() {
        elements.add(basket);
        elements.add(subAccess);
    }

    public void init_loop() {
        isRed = alliance.isRed();
    }

    public void finalizeField(Constants.Alliance alliance) {
        flipField(alliance.isRed());
        finalized = true;
        zones = Zone.getNamedZones();
        isRed = alliance.isRed();
        allianceMultiplier = isRed ? 1 : -1;
        basket = new POI(-2.4 * allianceMultiplier, -2.3 * allianceMultiplier, isRed ? 53 : 53 + 180, "BASKET");
        basket2 = new POI(-2.375 * allianceMultiplier, -2.375 * allianceMultiplier, 55, "BASKET");
        basketPrep = new POI(-2.4 * allianceMultiplier, -2 * allianceMultiplier, isRed ? 50 : 50 + 180, "BASKET");
        ground1 = new POI(-2.4 * allianceMultiplier, -1.8 * allianceMultiplier, isRed ? 85 : 85  + 180, "GROUND1");
        ground2 = new POI(-2 * allianceMultiplier, -1.8 * allianceMultiplier, isRed ? 89 : 89 + 180, "GROUND2");
        ground3 = new POI(-2.7 * allianceMultiplier, -1.7 * allianceMultiplier, isRed ? 95 : 95 + 180, "GROUND3");
    }

    public void flipField(boolean alliance) {

    }

    public void update(TelemetryPacket packet, Robot robot) {
        //handling dashboard fieldOverlay
//        Zone zone = getZone(robot.driveTrain.pose);
//        Canvas c = packet.fieldOverlay();
//        if (zone != null) {
//            double zoneX = Math.min(zone.x1, zone.x2) * FIELD_INCHES_PER_GRID;
//            double zoneY = Math.min(zone.y1, zone.y2) * FIELD_INCHES_PER_GRID;
//            double zoneHeight = Math.max(zone.x1, zone.x2) * FIELD_INCHES_PER_GRID - zoneX;
//            double zoneWidth = Math.max(zone.y1, zone.y2) * FIELD_INCHES_PER_GRID - zoneY;
//            if (System.currentTimeMillis() / 1000 % 2 == 0) {
//                c.setAlpha(.5);
//                c.setFill("green");
//                c.fillRect(zoneX, zoneY, zoneHeight, zoneWidth);
//
//            }
//            c.setAlpha(1);
//        }
    }

    public Zone getZone(Pose2d robotPosition) {
        for (Zone k : zones) {
            if (k.withinZone(robotPosition))
                return k;
        }
        //SHOULD NEVER HAPPEN BC ZONES BLANKET THE WHOLE FIELD
        return null;
    }

    public static double wrapAngle(double angle) {
        return ((angle % 360) + 360) % 360;
    }

    //get SubZones at robotPosition
    public ArrayList<SubZone> getSubZones(Pose2d robotPosition) {
        ArrayList<SubZone> temp = new ArrayList<>();
        SubZone[] subZones = null;
        for (SubZone k : subZones) {
            if (k.withinSubZone(robotPosition))
                temp.add(k);
        }
        return temp;
    }


    //get POIs at robotPosition
    public POI getPOI(Pose2d robotPosition) {
        List<POI> temp = Arrays.asList();
        for (POI poi : temp) {
            if (poi.atPOI(robotPosition))
                return poi;
        }
        return null;
    }

    public static Pose2d P2D(double x, double y, double deg) {
        return new Pose2d(x * FIELD_INCHES_PER_GRID, y * FIELD_INCHES_PER_GRID, Math.toRadians(deg));
    }
}







