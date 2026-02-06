package org.firstinspires.ftc.teamcode.robots.lebot2;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.TankDriveActions;

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
@Config(value = "Lebot2_FieldMap")
public class FieldMap {

    // ==================== VISUALIZATION CONFIG ====================
    public static boolean DRAW_WAYPOINTS = true;  // Toggle waypoint visualization
    public static double WAYPOINT_RADIUS = 4.0;   // 8" diameter = 4" radius

    // ==================== INTAKE ASYMMETRY OFFSET ====================
    // Ball row waypoints need X offset when reflected for blue alliance
    // due to asymmetric intake design (shifts 6" in positive X direction)
    public static double BALL_ROW_BLUE_X_OFFSET = -1;  // inches
    public static double ROW_X_OFFSET = 0;
    public static double ROW_Y_START_OFFSET = 4;

    // ==================== SPLINE-SPECIFIC OFFSETS ====================
    // Additional X offsets for row starts when using splines — helps create
    // a more gradual curve that the intake can handle. Only applied when USE_SPLINES is true.
    public static double ROW_1_SPLINE_X_OFFSET = -2;
    public static double ROW_2_SPLINE_X_OFFSET = -4;  // inches, negative = shift left
    public static double ROW_3_SPLINE_X_OFFSET = 0;   // inches

    public static double FIRE_1_ANGLE_OFFSET = -2;

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

        /**
         * Reflect across X axis with an additional X offset.
         * Used for asymmetric mechanisms like intake.
         */
        public Waypoint reflectedWithXOffset(double xOffset) {
            return new Waypoint(x + xOffset, -y, -heading);
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
        //speed for audience start is 1100 and star feed .8
        //65, 16.8
        RED_WAYPOINTS.put("START_AUDIENCE", new Waypoint(64.5,16.8, 160));  // TODO: measure heading
        // GOAL: Near goal, facing goal
        RED_WAYPOINTS.put("START_GOAL", new Waypoint(-46.4566, 47.244, 135));

        // ----- Firing Positions -----
        // Positions where robot stops to launch balls at goal
        // FIRE_1 X uses FIRE_1_BASE_X which can be offset via Dashboard
        RED_WAYPOINTS.put("FIRE_1", new Waypoint(-14.1732+2, 15.748, 135));  // Base position, offset applied in get()
        RED_WAYPOINTS.put("FIRE_2", new Waypoint(-31.7,16, 124.4));  // TODO: measure
        RED_WAYPOINTS.put("FIRE_3", new Waypoint(0, 0, 0));  // TODO: measure
        RED_WAYPOINTS.put("FIRE_4", new Waypoint(64.5,16.8, 160));  // TODO: measure    //fire from back triangle

        // ----- Ball Pickup Waypoints -----
        // Starting points for each of the 3 ball rows
        // Base coordinates only — ROW_X_OFFSET and ROW_Y_START_OFFSET applied dynamically in get()
        RED_WAYPOINTS.put("BALL_ROW_1_START", new Waypoint(-14.1732, 25.9842, 90));
        RED_WAYPOINTS.put("BALL_ROW_2_START", new Waypoint(10.2362, 25.9842, 90));
        RED_WAYPOINTS.put("BALL_ROW_3_START", new Waypoint(34.6456, 25.9842, 90));

        // ----- Ball Row Endpoints -----
        // Ending points after driving through ball rows
        // Base coordinates only — ROW_X_OFFSET applied dynamically in get()
        RED_WAYPOINTS.put("BALL_ROW_1_END", new Waypoint(-14.1732, 46.8503+4.5-3, 90));
        RED_WAYPOINTS.put("BALL_ROW_2_END", new Waypoint(10.2362, 46.8503+4.5-5, 90));
        RED_WAYPOINTS.put("BALL_ROW_3_END", new Waypoint(34.6456, 46.8503+4.5, 90));

        // ----- Gate -----
        // Position to release previously scored balls
        RED_WAYPOINTS.put("GATE", new Waypoint(0, 0, 0));  // TODO: measure

        // ----- Homebase -----
        RED_WAYPOINTS.put("HOMEBASE", new Waypoint(0, 0, 0));  // TODO: measure

        // ----- Goal Target -----
        // Center of goal opening - used for aiming calculations
        // From official DECODE field coordinates
        RED_WAYPOINTS.put("GOAL", new Waypoint(-58.3727, 55.6425, 135));
    }

    // ==================== WAYPOINT ACCESS ====================

    /**
     * Get a waypoint by name, automatically reflected for blue alliance.
     * Ball row waypoints have dynamic offsets applied (ROW_X_OFFSET, ROW_Y_START_OFFSET)
     * so Dashboard changes take effect immediately.
     *
     * @param name Waypoint name (e.g., "FIRE_1", "BALL_ROW_2_START")
     * @param isRedAlliance true for red, false for blue
     * @return The waypoint with offsets applied, reflected if blue alliance
     * @throws IllegalArgumentException if waypoint name not found
     */
    public static Waypoint get(String name, boolean isRedAlliance) {
        Waypoint baseWaypoint = RED_WAYPOINTS.get(name);
        if (baseWaypoint == null) {
            throw new IllegalArgumentException("Unknown waypoint: " + name);
        }
        Waypoint redWaypoint = baseWaypoint;
        if(name.equals("FIRE_1")){
            double angleOffset = FIRE_1_ANGLE_OFFSET;
            //code for angle to be right just for blue (angleoffset = -2)
            redWaypoint = new Waypoint(
                    baseWaypoint.x,
                    baseWaypoint.y,
                    baseWaypoint.heading + angleOffset
            );
            //code for angle to be right for red and blue
//            redWaypoint = new Waypoint(
//                    baseWaypoint.x,
//                    baseWaypoint.y,
//                    baseWaypoint.heading - (isRedAlliance?angleOffset: -angleOffset)
//            );
        }

        // Apply dynamic offsets to ball row waypoints
        //Waypoint redWaypoint = baseWaypoint;
        if (name.startsWith("BALL_ROW")) {
            double xOffset = -ROW_X_OFFSET;  // Negative because original had subtraction
            double yOffset = 0;
            if (name.contains("_START")) {
                yOffset = ROW_Y_START_OFFSET;

                // Apply spline-specific X offsets when USE_SPLINES is enabled
                // These help create more gradual curves that the intake can handle
                if (TankDriveActions.USE_SPLINES) {
                    if(name.equals("BALL_ROW_1_START")){
                        xOffset +=ROW_1_SPLINE_X_OFFSET;
                    }
                    else if (name.equals("BALL_ROW_2_START")) {
                        xOffset += ROW_2_SPLINE_X_OFFSET;
                    } else if (name.equals("BALL_ROW_3_START")) {
                        xOffset += ROW_3_SPLINE_X_OFFSET;
                    }
                }
            }
            redWaypoint = new Waypoint(
                    baseWaypoint.x + xOffset,
                    baseWaypoint.y + yOffset,
                    baseWaypoint.heading
            );
        }

        if (isRedAlliance) {
            return redWaypoint;
        }

        // Ball row waypoints need additional X offset for blue due to intake asymmetry
        if (name.startsWith("BALL_ROW")) {
            if(name.startsWith("BALL_ROW_1")){
                return redWaypoint.reflectedWithXOffset(0);
            }
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
     * Calculate bearing from current position to target position.
     * Use this to turn toward a target before driving with lineToX/lineToY.
     *
     * @param from Current pose
     * @param to Target pose
     * @return Bearing in radians (ready for turnTo())
     */
    public static double bearingTo(Pose2d from, Pose2d to) {
        double dx = to.position.x - from.position.x;
        double dy = to.position.y - from.position.y;
        return Math.atan2(dy, dx);
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
    public static final String GOAL = "GOAL";

    // ==================== VISUALIZATION ====================

    /**
     * Draw all waypoints on the field overlay.
     * Red alliance waypoints drawn in red, blue alliance in blue.
     * Each waypoint is drawn as an 8" diameter circle.
     * Waypoints are drawn with all dynamic offsets applied (ROW_X_OFFSET,
     * ROW_Y_START_OFFSET, spline-specific offsets when USE_SPLINES is true).
     *
     * @param canvas The FTC Dashboard field overlay canvas
     */
    public static void drawWaypoints(Canvas canvas) {
        if (!DRAW_WAYPOINTS || canvas == null) {
            return;
        }

        canvas.setStrokeWidth(1);

        for (String name : getWaypointNames()) {
            // Check base waypoint to skip unmeasured ones (at origin)
            Waypoint baseWp = RED_WAYPOINTS.get(name);
            if (baseWp.x == 0 && baseWp.y == 0 && baseWp.heading == 0) {
                continue;
            }

            // Get waypoints with all dynamic offsets applied
            Waypoint redWp = get(name, true);   // Red alliance with offsets
            Waypoint blueWp = get(name, false); // Blue alliance with offsets + reflection

            // Draw red alliance waypoint
            canvas.setStroke("#FF0000");  // Red
            canvas.strokeCircle(redWp.x, redWp.y, WAYPOINT_RADIUS);

            // Draw blue alliance waypoint
            canvas.setStroke("#0000FF");  // Blue
            canvas.strokeCircle(blueWp.x, blueWp.y, WAYPOINT_RADIUS);
        }
    }
}
