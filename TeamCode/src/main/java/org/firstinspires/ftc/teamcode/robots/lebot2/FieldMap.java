package org.firstinspires.ftc.teamcode.robots.lebot2;

import com.acmerobotics.roadrunner.Pose2d;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

/**
 * Field waypoints for autonomous navigation.
 *
 * Waypoints are defined for the RED alliance and automatically reflected
 * across the X axis for BLUE alliance. This uses DECODE field coordinates:
 * - X+ toward audience
 * - Y+ toward blue alliance wall
 * - Red side is negative Y, Blue side is positive Y
 *
 * Usage:
 *   Pose2d pose = FieldMap.getPose("FIRE_1", Robot.isRedAlliance);
 *   Waypoint wp = FieldMap.get("BALL_ROW_1_START", Robot.isRedAlliance);
 */
public class FieldMap {

    //=====================INTAKE ASYMMETRY OFFSET================
    //ball row waypoints need X offset when reflected for blue alliance
    //due to asymmetric intake design )shifts 6" in positive x direction)
    public static final double BALL_ROW_BLUE_X_OFFSET = 6.0; //inches

    // ==================== WAYPOINT CLASS ====================

    public static class Waypoint {
        public final double x;        // inches
        public final double y;        // inches
        public final double heading;  // degrees

        public Waypoint(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        /**
         * Convert to RoadRunner Pose2d (heading in radians).
         */
        public Pose2d toPose2d() {
            return new Pose2d(x, y, Math.toRadians(heading));
        }

        /**
         * Reflect across X axis for opposite alliance.
         * Y negates, heading negates (mirror symmetry).
         */
        public Waypoint reflected() {
            return new Waypoint(x, -y, -heading);
        }

        //Reflect across X axis with an additional X offset
        //used for asymmetric mechanisms like intake
        public Waypoint reflectedWithXOffset(double xOffset){
            return new Waypoint(x + xOffset, -y,-heading);
        }

        @Override
        public String toString() {
            return String.format("(%.1f, %.1f, %.1f°)", x, y, heading);
        }
    }

    // ==================== RED ALLIANCE WAYPOINTS ====================
    // All waypoints defined for RED alliance (negative Y side)
    // Blue alliance waypoints are auto-generated via reflection

    private static final Map<String, Waypoint> RED_WAYPOINTS = new LinkedHashMap<>();

    static {
        // ----- Starting Positions -----
        // AUDIENCE: Near audience, facing goal
        RED_WAYPOINTS.put("START_AUDIENCE", new Waypoint(0, 0, 0));  // TODO: measure
        // GOAL: Near goal, facing goal
        RED_WAYPOINTS.put("START_GOAL", new Waypoint(-46.4566, 47.244, 135));      // TODO: measure

        // ----- Firing Positions -----
        // Positions where robot stops to launch balls at goal
        RED_WAYPOINTS.put("FIRE_1", new Waypoint(-17.12595, 15.748, 135));  // TODO: measure
        RED_WAYPOINTS.put("FIRE_2", new Waypoint(0, 0, 0));  // TODO: measure
        RED_WAYPOINTS.put("FIRE_3", new Waypoint(0, 0, 0));  // TODO: measure
        RED_WAYPOINTS.put("FIRE_4", new Waypoint(0, 0, 0));  // TODO: measure

        // ----- Ball Pickup Waypoints -----
        // Starting points for each of the 3 ball rows
        RED_WAYPOINTS.put("BALL_ROW_1_START", new Waypoint(-14.1732, 25.9842, 90));  // TODO: measure
        RED_WAYPOINTS.put("BALL_ROW_2_START", new Waypoint(10.2362, 25.9842, 90));  // TODO: measure
        RED_WAYPOINTS.put("BALL_ROW_3_START", new Waypoint(34.6456, 25.9842, 90));  // TODO: measure

        // ----- Ball Row Endpoints -----
        // Ending points after driving through ball rows
        RED_WAYPOINTS.put("BALL_ROW_1_END", new Waypoint(-14.1732, 46.8503, 90));  // TODO: measure
        RED_WAYPOINTS.put("BALL_ROW_2_END", new Waypoint(10.2362, 46.8503, 90));  // TODO: measure
        RED_WAYPOINTS.put("BALL_ROW_3_END", new Waypoint(34.6456, 46.8503, 90));  // TODO: measure

        // ----- Gate -----
        // Position to release previously scored balls
        RED_WAYPOINTS.put("GATE", new Waypoint(0, 0, 0));  // TODO: measure

        // ----- Homebase -----
        RED_WAYPOINTS.put("HOMEBASE", new Waypoint(0, 0, 0));  // TODO: measure
    }

    // ==================== WAYPOINT ACCESS ====================

    /**
     * Get a waypoint by name, automatically reflected for blue alliance.
     *Ball row waypoints get an additional X offset for blue alliance
     * to compensate for asymmetric intake design
     * @param name Waypoint name (e.g., "FIRE_1", "BALL_ROW_2_START")
     * @param isRedAlliance true for red, false for blue
     * @return The waypoint, reflected if blue alliance
     * @throws IllegalArgumentException if waypoint name not found
     */
    public static Waypoint get(String name, boolean isRedAlliance) {
        Waypoint redWaypoint = RED_WAYPOINTS.get(name);
        if (redWaypoint == null) {
            throw new IllegalArgumentException("Unknown waypoint: " + name);
        }
        if(isRedAlliance){
            return redWaypoint;
        }

        //Ball row waypoints need X offset for blue due to intake asymmetry
        if(name.startsWith("BALL_ROW")){
            return redWaypoint.reflectedWithXOffset(BALL_ROW_BLUE_X_OFFSET);
        }

        return redWaypoint.reflected();
    }

    /**
     * Get a waypoint as a Pose2d, automatically reflected for blue alliance.
     *
     * @param name Waypoint name
     * @param isRedAlliance true for red, false for blue
     * @return Pose2d with heading in radians
     */
    public static Pose2d getPose(String name, boolean isRedAlliance) {
        return get(name, isRedAlliance).toPose2d();
    }

    /**
     * Get all available waypoint names.
     */
    public static Set<String> getWaypointNames() {
        return RED_WAYPOINTS.keySet();
    }

    /**
     * Check if a waypoint exists.
     */
    public static boolean hasWaypoint(String name) {
        return RED_WAYPOINTS.containsKey(name);
    }

    /**
     * Add a waypoint at runtime (for testing/tuning via dashboard).
     * Waypoint is defined for red alliance, auto-reflected for blue.
     */
    public static void addWaypoint(String name, double x, double y, double heading) {
        RED_WAYPOINTS.put(name, new Waypoint(x, y, heading));
    }

    // ==================== WAYPOINT NAME CONSTANTS ====================
    // Use these to avoid typos

    public static final String START_AUDIENCE = "START_AUDIENCE";
    public static final String START_GOAL = "START_GOAL";

    public static final String FIRE_1 = "FIRE_1";
    public static final String FIRE_2 = "FIRE_2";
    public static final String FIRE_3 = "FIRE_3";
    public static final String FIRE_4 = "FIRE_4";

    public static final String BALL_ROW_1_START = "BALL_ROW_1_START";
    public static final String BALL_ROW_2_START = "BALL_ROW_2_START";
    public static final String BALL_ROW_3_START = "BALL_ROW_3_START";

    public static final String BALL_ROW_1_END = "BALL_ROW_1_END";
    public static final String BALL_ROW_2_END = "BALL_ROW_2_END";
    public static final String BALL_ROW_3_END = "BALL_ROW_3_END";

    public static final String GATE = "GATE";
    public static final String HOMEBASE = "HOMEBASE";
}
