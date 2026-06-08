package org.firstinspires.ftc.teamcode.robots.lebot2;

import android.os.Environment;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.Locale;
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

    public static double BALL_ROW_5_START_OFFSET = 9;

    public static double AVOID_COLLISION_AUDIENCE = 10;

    public static double AUDIENCE_HUMAN_PLAYER_BALLROW_OFFSET =5 ;
    public static double OFFSET = 2;          //4.5

    // ==================== VISUALIZATION CONFIG ====================
    public static boolean DRAW_WAYPOINTS = true;  // Toggle waypoint visualization
    public static double WAYPOINT_RADIUS = 4.0;   // 8" diameter = 4" radius

    // ==================== START POSITION MODE ====================
    // Selects which ball-row set is used AND drawn: goal-approach vs audience-approach.
    // Set by Autonomous/Robot at init from the chosen start; also a manual Dashboard
    // toggle for tuning/viewing one row set at a time.
    public static boolean IS_AUDIENCE_START = false;

    // Ball rows are now decoupled goal vs audience sets (WP_GOAL_ROW_* / WP_AUD_ROW_*) —
    // the waypoints are the spline anchors directly, no offsets. Blue still mirrors red.

    // ==================== FIRING ANGLE OFFSETS ====================
    // Offset from goal center for firing positions (degrees)
    // Applied before reflection so red/blue aim at mirrored backboards
    public static double FIRE_1_ANGLE_OFFSET = -6;  // Goal start firing position
    public static double FIRE_2_ANGLE_OFFSET = -9;
    public static double FIRE_4_ANGLE_OFFSET = 0;   // Audience start firing position

    // ==================== DEFAULT FLYWHEEL SPEEDS ====================
    // Fallback speeds (deg/sec) when vision distance can't be solved
    public static double FIRE_1_DEFAULT_DPS = 1050;   // Goal start distance
    public static double FIRE_4_DEFAULT_DPS = 1050;  // Audience start distance

    // ==================== WAYPOINT CLASS ====================

    public static class Waypoint {
        // Non-final so FTC Dashboard can edit x/y/heading live when a Waypoint is
        // exposed as a public static @Config field (see WP_* fields below).
        public double x;        // inches
        public double y;        // inches
        public double heading;  // degrees

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

    // ==================== TUNABLE WAYPOINTS (Dashboard-editable) ====================
    // RED-alliance base coords; blue is mirrored automatically in get().
    // Edit x/y/heading live on Dashboard (Lebot2_FieldMap → WP_*), watch the field overlay,
    // then set DUMP_WAYPOINTS = true to print paste-ready coords to logcat and copy them
    // back into the static block below. (Dashboard edits are NOT saved to code.)
    // NOTE: fixed field references (GOAL, BASE, GATE, HUMAN_PLAYER, HOMEBASE) are intentionally
    // NOT exposed — they're official field constants and shouldn't be casually edited.
    public static Waypoint WP_START_AUDIENCE  = new Waypoint(58.7, 20.8, 139);
    public static Waypoint WP_START_GOAL      = new Waypoint(-50.4566, 51.244, 135);
    public static Waypoint WP_FIRE_1 = new Waypoint(-24.9, 28.0, 135);
    public static Waypoint WP_FIRE_2 = new Waypoint(-22, 22, 90);
    public static Waypoint WP_FIRE_3 = new Waypoint(-16.6, 17.1, 90);
    public static Waypoint WP_FIRE_4 = new Waypoint(56.7, 20.8, 139);   // was 58.7 - OFFSET
    public static Waypoint WP_FIRE_5 = new Waypoint(58.7, 21.5, 139);
    public static Waypoint WP_FIRE_6 = new Waypoint(58.7, 21.5, 139);
    // Ball rows are decoupled by approach direction — splining a row from the goal end vs the
    // audience end is a different trajectory, so each has its own anchors. RED base; blue mirrors.
    // get() selects the set by IS_AUDIENCE_START (which also drives the field overlay).
    // Seeded from the resolved offset values that were in use at decouple time.
    // ----- Goal-approach set -----
    public static Waypoint WP_GOAL_ROW_1_START = new Waypoint(-17, 38, 45);
    public static Waypoint WP_GOAL_ROW_2_START = new Waypoint(5, 39, 45);
    public static Waypoint WP_GOAL_ROW_3_START = new Waypoint(29, 39, 30);
    public static Waypoint WP_GOAL_ROW_1_END   = new Waypoint(-12, 54, 90);
    public static Waypoint WP_GOAL_ROW_2_END   = new Waypoint(12, 54, 90);
    public static Waypoint WP_GOAL_ROW_3_END   = new Waypoint(34, 54, 90);
    // ----- Audience-approach set -----
    public static Waypoint WP_AUD_ROW_1_START = new Waypoint(-7.1732, 36.4842, 90);
    public static Waypoint WP_AUD_ROW_2_START = new Waypoint(8.2362, 29.9842, 90);
    public static Waypoint WP_AUD_ROW_3_START = new Waypoint(33.6456, 29.9842, 90);
    public static Waypoint WP_AUD_ROW_1_END   = new Waypoint(-13.1732, 56.3503, 90);
    public static Waypoint WP_AUD_ROW_2_END   = new Waypoint(10.2362, 52.3503, 90);
    public static Waypoint WP_AUD_ROW_3_END   = new Waypoint(33.6456, 53.3503, 90);

    // Set true on Dashboard to print all current waypoints (paste-ready) to logcat, then resets.
    public static boolean DUMP_WAYPOINTS = false;

    // ==================== RED ALLIANCE WAYPOINTS ====================
    // All waypoints defined for RED alliance (negative Y side)
    // Blue alliance waypoints are auto-generated via reflection

    private static final Map<String, Waypoint> RED_WAYPOINTS = new LinkedHashMap<>();

    static {
        // ----- Starting Positions -----
        // AUDIENCE: Near audience, facing goal
        //speed for audience start is 1100 and star feed .8
        //65, 16.8
        //OLD WAYPOINT
        //RED_WAYPOINTS.put("START_AUDIENCE", new Waypoint(64.7, 17.1, 168.2));
        //RED_WAYPOINTS.put("START_AUDIENCE", new Waypoint(58.7, 20.8, 139));
        //old reliable
        RED_WAYPOINTS.put("START_AUDIENCE", WP_START_AUDIENCE);

        // GOAL: Near goal, facing goal
        RED_WAYPOINTS.put("START_GOAL", WP_START_GOAL);

        // ----- Firing Positions -----
        // Positions where robot stops to launch balls at goal
        // FIRE_1 X uses FIRE_1_BASE_X which can be offset via Dashboard
        //-------OLD WAYPOINTS-------
//        RED_WAYPOINTS.put("FIRE_1", new Waypoint(-14.1732-2, 15.748+2, 135));  // Base position, offset applied in get()
//        RED_WAYPOINTS.put("FIRE_2", new Waypoint(-31.7-OFFSET,16+OFFSET, 124.4));  // Fire from inside big triangle
//        RED_WAYPOINTS.put("FIRE_3", new Waypoint(56.1, 18.9+AVOID_COLLISION_AUDIENCE, 168.8));  // TODO: measure
//        RED_WAYPOINTS.put("FIRE_4", new Waypoint(64.7, 17.1, 168.2));  //fire from back triangle

        //----firing points for goal auton-----
        RED_WAYPOINTS.put("FIRE_1", WP_FIRE_1);  // heading matches START_GOAL
        RED_WAYPOINTS.put("FIRE_2", WP_FIRE_2);  // Fire from inside big triangle
        RED_WAYPOINTS.put("FIRE_3", WP_FIRE_3);

        //------firing points for audience auton--------
        RED_WAYPOINTS.put("FIRE_4", WP_FIRE_4);  //fire from back triangle
        RED_WAYPOINTS.put("FIRE_5", WP_FIRE_5);
        RED_WAYPOINTS.put("FIRE_6", WP_FIRE_6);



        // ----- Ball Pickup Waypoints -----
        // Ball rows are decoupled goal/audience sets; get() selects the active set by
        // IS_AUDIENCE_START. These map entries reference the GOAL set only for enumeration
        // and overlay (get() overrides the selection for any BALL_ROW name).
        RED_WAYPOINTS.put("BALL_ROW_1_START", WP_GOAL_ROW_1_START);
        RED_WAYPOINTS.put("BALL_ROW_2_START", WP_GOAL_ROW_2_START);
        RED_WAYPOINTS.put("BALL_ROW_3_START", WP_GOAL_ROW_3_START);

        //new waypoint for picking up ball in opposing human player area during auton
//        RED_WAYPOINTS.put("BALL_ROW_4_START", new Waypoint(62.4-BALL_ROW_5_START_OFFSET, 53.2, 90));
//        RED_WAYPOINTS.put("BALL_ROW_4_END", new Waypoint(62.4, 60+AUDIENCE_HUMAN_PLAYER_BALLROW_OFFSET, 90));
//        RED_WAYPOINTS.put("BALL_ROW_5_START", new Waypoint(10.2362, 25.9842, 90));
//        RED_WAYPOINTS.put("BALL_ROW_5_END", new Waypoint(10.2362, 46.8503+4.5-5, 90));

        // ----- Ball Row Endpoints -----
        // (See note above — GOAL set referenced for enumeration; get() selects active set.)
        RED_WAYPOINTS.put("BALL_ROW_1_END", WP_GOAL_ROW_1_END);
        RED_WAYPOINTS.put("BALL_ROW_2_END", WP_GOAL_ROW_2_END);
        RED_WAYPOINTS.put("BALL_ROW_3_END", WP_GOAL_ROW_3_END);

        // ----- Gate -----
        // Position to release previously scored balls
        RED_WAYPOINTS.put("GATE", new Waypoint(0, 0, 0));  // TODO: measure

        // ----- Homebase -----
        RED_WAYPOINTS.put("HOMEBASE", new Waypoint(0, 0, 0));  // TODO: measure

        // ----- Goal Target -----
        // Center of goal opening - used for aiming calculations
        // From official DECODE field coordinates
        RED_WAYPOINTS.put("GOAL", new Waypoint(-58.3727, 55.6425, 135));

        // ----- Team Bases -----
        // Safe parking spots for each alliance
        RED_WAYPOINTS.put("BASE", new Waypoint(36.5, -33.25, 0));

        // ----- Human Player Zones -----
        // Audience wall corners where robots can be hand-fed balls
        RED_WAYPOINTS.put("HUMAN_PLAYER", new Waypoint(41, -46, 45));
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
        // Ball rows: decoupled goal vs audience approach sets, selected by start mode.
        // The waypoints ARE the spline anchors — no offsets applied. Blue mirrors red.
        if (name.startsWith("BALL_ROW")) {
            Waypoint redRow = selectRowWaypoint(name);
            return isRedAlliance ? redRow : redRow.reflected();
        }

        Waypoint baseWaypoint = RED_WAYPOINTS.get(name);
        if (baseWaypoint == null) {
            throw new IllegalArgumentException("Unknown waypoint: " + name);
        }

        // Start with base waypoint, apply any position-specific offsets
        Waypoint redWaypoint = baseWaypoint;

        // Apply firing angle offset (before reflection, so red/blue aim at mirrored backboards)
        if (name.equals("FIRE_1")) {
            redWaypoint = new Waypoint(
                    baseWaypoint.x,
                    baseWaypoint.y,
                    baseWaypoint.heading + FIRE_1_ANGLE_OFFSET
            );
        } else if (name.equals("FIRE_2")) {
            redWaypoint = new Waypoint(
                    baseWaypoint.x,
                    baseWaypoint.y,
                    baseWaypoint.heading + FIRE_2_ANGLE_OFFSET
            );
        }

        if (isRedAlliance) {
            return redWaypoint;
        }

        return redWaypoint.reflected();
    }

    /**
     * Select the active RED ball-row waypoint based on start mode (goal vs audience approach).
     * Splining a row from opposite ends of the field is a different trajectory, so each
     * approach has its own anchors — no shared base + offsets.
     */
    private static Waypoint selectRowWaypoint(String name) {
        boolean aud = IS_AUDIENCE_START;
        switch (name) {
            case "BALL_ROW_1_START": return aud ? WP_AUD_ROW_1_START : WP_GOAL_ROW_1_START;
            case "BALL_ROW_2_START": return aud ? WP_AUD_ROW_2_START : WP_GOAL_ROW_2_START;
            case "BALL_ROW_3_START": return aud ? WP_AUD_ROW_3_START : WP_GOAL_ROW_3_START;
            case "BALL_ROW_1_END":   return aud ? WP_AUD_ROW_1_END   : WP_GOAL_ROW_1_END;
            case "BALL_ROW_2_END":   return aud ? WP_AUD_ROW_2_END   : WP_GOAL_ROW_2_END;
            case "BALL_ROW_3_END":   return aud ? WP_AUD_ROW_3_END   : WP_GOAL_ROW_3_END;
            default:
                throw new IllegalArgumentException("Unknown ball row: " + name);
        }
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

    /**
     * Build a paste-ready dump of all current RED waypoints (base coords, before offsets).
     * After tuning the WP_* waypoints on Dashboard, set DUMP_WAYPOINTS=true to print this to
     * logcat, then copy the lines back into the static block / WP_* fields so the tuned values
     * survive in code (Dashboard edits are not persisted to source).
     */
    public static String dumpWaypoints() {
        StringBuilder sb = new StringBuilder("==== FieldMap waypoint dump (RED base coords) ====\n");
        // Non-row waypoints from the map (rows are decoupled — handled explicitly below)
        for (Map.Entry<String, Waypoint> e : RED_WAYPOINTS.entrySet()) {
            if (e.getKey().startsWith("BALL_ROW")) continue;
            Waypoint w = e.getValue();
            sb.append(dumpLine(e.getKey(), w));
        }
        // Both decoupled ball-row sets
        sb.append("-- goal-approach rows --\n");
        sb.append(dumpLine("WP_GOAL_ROW_1_START", WP_GOAL_ROW_1_START));
        sb.append(dumpLine("WP_GOAL_ROW_2_START", WP_GOAL_ROW_2_START));
        sb.append(dumpLine("WP_GOAL_ROW_3_START", WP_GOAL_ROW_3_START));
        sb.append(dumpLine("WP_GOAL_ROW_1_END", WP_GOAL_ROW_1_END));
        sb.append(dumpLine("WP_GOAL_ROW_2_END", WP_GOAL_ROW_2_END));
        sb.append(dumpLine("WP_GOAL_ROW_3_END", WP_GOAL_ROW_3_END));
        sb.append("-- audience-approach rows --\n");
        sb.append(dumpLine("WP_AUD_ROW_1_START", WP_AUD_ROW_1_START));
        sb.append(dumpLine("WP_AUD_ROW_2_START", WP_AUD_ROW_2_START));
        sb.append(dumpLine("WP_AUD_ROW_3_START", WP_AUD_ROW_3_START));
        sb.append(dumpLine("WP_AUD_ROW_1_END", WP_AUD_ROW_1_END));
        sb.append(dumpLine("WP_AUD_ROW_2_END", WP_AUD_ROW_2_END));
        sb.append(dumpLine("WP_AUD_ROW_3_END", WP_AUD_ROW_3_END));
        return sb.toString();
    }

    private static String dumpLine(String label, Waypoint w) {
        return String.format(Locale.US,
                "new Waypoint(%.4f, %.4f, %.4f);  // %s%n", w.x, w.y, w.heading, label);
    }

    /**
     * Write the current waypoint dump to /FIRST/waypoint_dump.txt (overwrites each time, so the
     * file always holds the latest values). Pull it off the robot like the CSV logs.
     */
    public static void writeDumpToFile() {
        String path = Environment.getExternalStorageDirectory() + "/FIRST/waypoint_dump.txt";
        try (FileWriter fw = new FileWriter(new File(path), false)) {
            fw.write(dumpWaypoints());
            System.out.println("Waypoint dump written to " + path);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // ==================== WAYPOINT NAME CONSTANTS ====================
    // Use these to avoid typos

    public static final String START_AUDIENCE = "START_AUDIENCE";
    public static final String START_GOAL = "START_GOAL";

    public static final String FIRE_1 = "FIRE_1";
    public static final String FIRE_2 = "FIRE_2";
    public static final String FIRE_3 = "FIRE_3";
    public static final String FIRE_4 = "FIRE_4";
    public static final String FIRE_5 = "FIRE_5";
    public static final String FIRE_6 = "FIRE_6";


    public static final String BALL_ROW_1_START = "BALL_ROW_1_START";
    public static final String BALL_ROW_2_START = "BALL_ROW_2_START";
    public static final String BALL_ROW_3_START = "BALL_ROW_3_START";
    public static final String BALL_ROW_4_START = "BALL_ROW_4_START";
    public static final String BALL_ROW_5_START = "BALL_ROW_5_START";

    public static final String BALL_ROW_1_END = "BALL_ROW_1_END";
    public static final String BALL_ROW_2_END = "BALL_ROW_2_END";
    public static final String BALL_ROW_3_END = "BALL_ROW_3_END";
    public static final String BALL_ROW_4_END = "BALL_ROW_4_END";
    public static final String BALL_ROW_5_END = "BALL_ROW_5_END";

    public static final String GATE = "GATE";
    public static final String HOMEBASE = "HOMEBASE";
    public static final String GOAL = "GOAL";
    public static final String BASE = "BASE";
    public static final String HUMAN_PLAYER = "HUMAN_PLAYER";

    // ==================== VISUALIZATION ====================

    /**
     * Draw all waypoints on the field overlay.
     * Red alliance waypoints drawn in red, blue alliance in blue.
     * Each waypoint is drawn as an 8" diameter circle.
     * Ball rows show only the ACTIVE set (goal vs audience) per IS_AUDIENCE_START,
     * since get() returns the selected set.
     *
     * @param canvas The FTC Dashboard field overlay canvas
     */
    public static void drawWaypoints(Canvas canvas) {
        // One-shot: dump current waypoints to /FIRST/waypoint_dump.txt when toggled on Dashboard
        if (DUMP_WAYPOINTS) {
            writeDumpToFile();
            DUMP_WAYPOINTS = false;
        }

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
