package org.firstinspires.ftc.teamcode.robots.csbot;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.alliance;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.robot;
import static org.firstinspires.ftc.teamcode.robots.csbot.util.Constants.FIELD_INCHES_PER_GRID;
import static org.firstinspires.ftc.teamcode.robots.csbot.util.Utils.P2D;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;

import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot;

import java.lang.Math;
import java.util.*;

public class Field {
    public boolean finalized = false;
    List<Zone> zones;
    List<SubZone> subZones;

    public List<CrossingRoute> crossingRoutes = new ArrayList<>();

    //shouldn't iterate through POIS, they're selectable destinations
    //todo - maybe implement w/ a map?
    POI HANG;
    POI HANG_PREP;
    POI WING_INTAKE;
    POI SCORE;

    POI APRILTAG1, APRILTAG2, APRILTAG3, APRILTAG4, APRILTAG5, APRILTAG6;


    //all values are in field grids
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
            List<Double> xVals = Arrays.asList(pose.position.x/FIELD_INCHES_PER_GRID, x1, x2);
            List<Double> yVals = Arrays.asList(pose.position.y/FIELD_INCHES_PER_GRID, y1, y2);
            if(
                //if the pose is not an extrema of the list, it's within the bounds of the list
                    !(
                            Collections.min(xVals) == pose.position.x/FIELD_INCHES_PER_GRID || Collections.max(xVals) == (pose.position.x/FIELD_INCHES_PER_GRID) ||
                                    Collections.min(yVals) == (pose.position.y/FIELD_INCHES_PER_GRID) || Collections.max(yVals) == (pose.position.y/FIELD_INCHES_PER_GRID)
                    )
            )
            {
                return true;
            }
            return false;
        }
    }

    public boolean isRed = true;

    public void init_loop() {
        isRed = alliance.getMod();
    }
    public void finalizeField() {
        finalized = true;
        zones = Zone.getNamedZones();
        subZones = SubZone.getNamedSubZones(isRed);
        if(isRed) {
            crossingRoutes.add(new CrossingRoute(P2D(1.5, -2.5, STANDARD_HEADING), P2D(3.5, -2.5, STANDARD_HEADING), 0));
            crossingRoutes.add(new CrossingRoute(P2D(1.5, -1.5, STANDARD_HEADING), P2D(3.5, -1.5, STANDARD_HEADING), 1));
            crossingRoutes.add(new CrossingRoute(P2D(1.5, -.5, STANDARD_HEADING), P2D(3.5, -.5, STANDARD_HEADING), 2));
            // DIAGONAL ROUTE
            crossingRoutes.add(new CrossingRoute(P2D(1.5, .5, -56.309932474), P2D(3.5, -.5, -56.309932474), 3));
            //
            crossingRoutes.add(new CrossingRoute(P2D(1.5, .5, STANDARD_HEADING), P2D(3.5, .5, STANDARD_HEADING), 4));
            crossingRoutes.add(new CrossingRoute(P2D(1.5, 1.5, STANDARD_HEADING), P2D(3.5, 1.5, STANDARD_HEADING), 5));
            crossingRoutes.add(new CrossingRoute(P2D(1.5, 2.5, STANDARD_HEADING), P2D(3.5, 2.5, STANDARD_HEADING), 6));
        }
        else {

            crossingRoutes.add(new CrossingRoute(P2D(1.5, 2.5, STANDARD_HEADING), P2D(3.5, 2.5, STANDARD_HEADING), 0));
            crossingRoutes.add(new CrossingRoute(P2D(1.5, 1.5, STANDARD_HEADING), P2D(3.5, 1.5, STANDARD_HEADING), 1));
            crossingRoutes.add(new CrossingRoute(P2D(1.5, .5, STANDARD_HEADING), P2D(3.5, .5, STANDARD_HEADING), 2));
            crossingRoutes.add(new CrossingRoute(P2D(1.5, -.5, STANDARD_HEADING), P2D(3.5, -.5, STANDARD_HEADING), 3));
//             DIAGONAL ROUTE
            crossingRoutes.add(new CrossingRoute(P2D(1.5, -.5, 56.309932474), P2D(3.5, .5, 56.309932474), 4));
            //
            crossingRoutes.add(new CrossingRoute(P2D(1.5, -1.5, STANDARD_HEADING), P2D(3.5, -1.5, STANDARD_HEADING), 5));
            crossingRoutes.add(new CrossingRoute(P2D(1.5, -2.5, STANDARD_HEADING), P2D(3.5, -2.5, STANDARD_HEADING), 6));
        }

        HANG = isRed? POI.HANG : POI.HANG.flipOnX();
        HANG_PREP = isRed? POI.HANG_PREP : POI.HANG_PREP.flipOnX();
        WING_INTAKE = isRed? POI.WING_INTAKE : POI.WING_INTAKE.flipOnX();
        SCORE = isRed? POI.SCORE : POI.SCORE.flipOnX();
        APRILTAG1 = POI.APRILTAG1;
        APRILTAG2 = POI.APRILTAG2;
        APRILTAG3 = POI.APRILTAG3;
        APRILTAG4 = POI.APRILTAG4;
        APRILTAG5 = POI.APRILTAG5;
        APRILTAG6 = POI.APRILTAG6;
    }

    public void update(TelemetryPacket packet, Robot robot) {
        //handling dashboard fieldOverlay
        Zone zone = getZone(robot.driveTrain.pose);
        Canvas c = packet.fieldOverlay();
        if (zone != null) {
            double zoneX = Math.min(zone.x1, zone.x2) * FIELD_INCHES_PER_GRID;
            double zoneY = Math.min(zone.y1, zone.y2) * FIELD_INCHES_PER_GRID;
            double zoneHeight = Math.max(zone.x1, zone.x2) * FIELD_INCHES_PER_GRID - zoneX;
            double zoneWidth = Math.max(zone.y1, zone.y2) * FIELD_INCHES_PER_GRID - zoneY;
            if (System.currentTimeMillis()/1000 % 2 == 0) {
                c.setAlpha(.5);
                c.setFill("green");
                c.fillRect(zoneX, zoneY, zoneHeight, zoneWidth);

            }
            c.setAlpha(100);
        }
    }

    //swap to pathtoPOI(pose, destinationpoi)

    public SequentialAction pathToPOI(Pose2d robotPosition, POI poi){
        Zone startZone = getZone(robotPosition);
        Zone endZone = poi.getZone();
        //ROBOT DOES NOTHING IF THE STARTZONE IS IN THE RIGGING
        if(startZone == Zone.RIGGING)
            return new SequentialAction();

        TrajectoryActionBuilder actionBuilder = robot.driveTrain.actionBuilder(robotPosition);



        return null;

    }

    public Pose2d getAprilTagPose(int id) {
        switch(id){
            case 1:
                return APRILTAG1.pose;
            case 2:
                return APRILTAG2.pose;
            case 3:
                return APRILTAG3.pose;
            case 4:
                return APRILTAG4.pose;
            case 5:
                return APRILTAG5.pose;
            case 6:
                return APRILTAG6.pose;
        }
        return null;
    }

    public Zone getZone(Pose2d robotPosition) {
        for(Zone k : zones) {
            if(k.withinZone(robotPosition))
                return k;
        }

        //SHOULD NEVER HAPPEN BC ZONES BLANKET THE WHOLE FIELD
        return null;
    }

    public ArrayList<SubZone> getSubZones(Pose2d robotPosition) {
        ArrayList<SubZone> temp = new ArrayList<>();
        for(SubZone k : subZones) {
            if(k.withinSubZone(robotPosition))
                temp.add(k);
        }
        return temp;
    }

    public POI getPOI(Pose2d robotPosition) {
        List<POI> temp = Arrays.asList(HANG, HANG_PREP, SCORE, WING_INTAKE);
        for(POI poi : temp) {
            if(poi.atPOI(robotPosition))
                return poi;
        }
        return null;
    }


}

class CrossingRoute {
    Pose2d audienceSide;
    Pose2d backstageSide;
    double ROUTE_HEADING;
    int index;

    public CrossingRoute(Pose2d audienceSide, Pose2d backstageSide, int index){
        this.audienceSide = audienceSide;
        this.backstageSide = backstageSide;
        ROUTE_HEADING = Math.toDegrees(audienceSide.heading.log());
        this.index = index;
    }

    //return the shortest distance between a given pose and either endpoint in field grids
    public double distanceToRoute(Pose2d pose) {
        double distanceToAudienceSide = Math.hypot(Math.abs(pose.position.x - audienceSide.position.x), Math.abs(pose.position.y - audienceSide.position.x)) / FIELD_INCHES_PER_GRID;
        double distanceToBackstageSide = Math.hypot(Math.abs(pose.position.x - backstageSide.position.x), Math.abs(pose.position.y - backstageSide.position.x)) / FIELD_INCHES_PER_GRID;
        return distanceToBackstageSide > distanceToAudienceSide?  distanceToAudienceSide : distanceToBackstageSide;
    }

    //return the closest entry point to a given pose
    public Pose2d getClosestEntryPose(Pose2d pose) {
        double distanceToAudienceSide = Math.hypot(Math.abs(pose.position.x - audienceSide.position.x), Math.abs(pose.position.y - audienceSide.position.x)) / FIELD_INCHES_PER_GRID;
        double distanceToBackstageSide = Math.hypot(Math.abs(pose.position.x - backstageSide.position.x), Math.abs(pose.position.y - backstageSide.position.x)) / FIELD_INCHES_PER_GRID;
        return distanceToBackstageSide > distanceToAudienceSide?  audienceSide : backstageSide;
    }

    //assumes robot is at the start of a crossing route and adds the whole route path to the actionbuilder
    public void addCrossingRoute(TrajectoryActionBuilder actionBuilder) {
//        actionBuilder.turnTo(Math.toRadians(ROUTE_HEADING)).lineto

    }

}







