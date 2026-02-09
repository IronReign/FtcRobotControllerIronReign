package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.LazyServo;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.LimelightStream;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Vision subsystem - wraps the Limelight for target detection and field localization.
 *
 * The Limelight is used for:
 * - Detecting AprilTags on goals (Blue=Tag20, Red=Tag24)
 * - Providing tx (horizontal offset) for fine targeting/aiming
 * - Providing botpose for field localization and distance calculation
 *
 * Distance Calculation Strategy:
 * Uses botpose (robot's field position from AprilTag localization) combined with
 * known goal positions to calculate distance. This is more robust than ty-based
 * trigonometry because it:
 * - Works regardless of camera pitch angle
 * - Handles oblique viewing angles automatically
 * - Allows camera servo to point anywhere (ball detection, goal tracking)
 *
 * THREE-PHASE UPDATE:
 * - readSensors(): Nothing needed (Limelight is USB, not I2C)
 * - calc(): Read latest Limelight result, compute distance from botpose
 * - act(): Nothing needed (read-only subsystem)
 */
@Config(value = "Lebot2_Vision")
public class Vision implements Subsystem {

    // Hardware
    private final Limelight3A limelight;
    private final LazyServo tilt;

    public enum Angle {
        DOWN,       //looking at balls (1615)
        STRAIGHT,   //straight forward (1365)
        UPMIN,      //default for long distance shooting (1240)
        UPMAX       //default for short distance shooting (1200)
    }
    private int tiltTicks=1460;

    private Angle angle = Angle.UPMAX;

    // Pipeline configuration
    public enum Pipeline {
        BLUE(0),      // Only recognizes blue goal (Tag 20)
        RED(1),       // Only recognizes red goal (Tag 24)
        GOALS(2),     // Either goal recognized
        OBELISK(3),   // Detects Tags 21-23 on the randomization obelisk
        DECODE(4);    // All DECODE season AprilTags - for vision lock tuning

        public final int id;
        Pipeline(int id) { this.id = id; }
    }

    // DECODE Field Goal Positions (meters, from field center)
    // Source: https://downloads.limelightvision.io/models/ftc2025DECODE.fmap
    public static final double BLUE_GOAL_X = -1.4827;  // Tag 20
    public static final double BLUE_GOAL_Y = -1.4133;
    public static final double RED_GOAL_X = -1.4827;   // Tag 24
    public static final double RED_GOAL_Y = 1.4133;

    // State
    private boolean isRedAlliance = true;
    private Pipeline currentPipeline = Pipeline.RED;
    private boolean hasValidTarget = false;
    private boolean hasBotPose = false;
    private double tx = 0;  // Horizontal offset (degrees) - for aiming
    private double ty = 0;  // Vertical offset (degrees)
    private double ta = 0;  // Target area (0-100% of image)
    private Pose3D botPose = null;       // Currently selected pose (MT2 or MT1)
    private double distanceToGoal = 0;  // Calculated from botpose + known goal position
    private double robotX = 0;  // Robot field position (meters) - from selected pose
    private double robotY = 0;
    private double robotHeading = 0;  // Robot heading from botpose (radians)
    private boolean usingMT2 = false;  // True if MT2 localization is active
    private boolean forceMT1 = false;  // Force MT1 mode (when initial position unknown)

    // Both MT1 and MT2 poses for comparison (calibration/debugging)
    private Pose3D mt1Pose = null;
    private Pose3D mt2Pose = null;
    private boolean hasMT1Pose = false;
    private boolean hasMT2Pose = false;
    private double mt1X = 0, mt1Y = 0, mt1Heading = 0;  // MT1 extracted values
    private double mt2X = 0, mt2Y = 0, mt2Heading = 0;  // MT2 extracted values

    // Dashboard streaming - fetches frames from Limelight's MJPEG stream
    private LimelightStream limelightStream = null;
    public static boolean ENABLE_DASHBOARD_STREAM = false;  // Toggle via dashboard config
    public static int STREAM_FPS = 5;  // Target FPS for dashboard streaming (keep low to reduce lag)
    public static double STREAM_SCALE = 0.5;  // Scale factor for dashboard image (0.25-1.0)

    public static double FLYWHEEL_SPEED_MULTIPLIER =1;

    public Vision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(currentPipeline.id);
        limelight.start();

        tilt = new LazyServo(hardwareMap, "tilt");
        tilt.setPosition(servoNormalize(tiltTicks));

    }

    // ==================== THREE-PHASE METHODS ====================

    @Override
    public void readSensors() {
        // PHASE 1: Nothing needed - Limelight is USB, not I2C
        // It processes frames asynchronously and we just read the latest result
    }

    /**
     * Update the Limelight with current robot orientation for MegaTag2 localization.
     * Call this before calc() with the current Pinpoint heading.
     *
     * @param yawDegrees Robot heading in degrees (from Pinpoint IMU)
     */
    public void updateRobotOrientation(double yawDegrees) {
        limelight.updateRobotOrientation(yawDegrees);
    }

    public double getFlywheelSpeed(){
        double speed = 31.58941*Math.pow(distanceToGoal,4) - 193.20612*Math.pow(distanceToGoal,3) + 422.84196*Math.pow(distanceToGoal,2) - 251.80393*distanceToGoal + 701.34893;
        return speed*FLYWHEEL_SPEED_MULTIPLIER;
    }


    @Override
    public void calc(Canvas fieldOverlay) {
        // PHASE 2: Get latest Limelight result and extract data
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            hasValidTarget = true;
            tx = result.getTx();
            ty = result.getTy();
            ta = result.getTa();

            // Always capture both MT1 and MT2 for comparison
            mt1Pose = result.getBotpose();
            mt2Pose = result.getBotpose_MT2();

            // Extract MT1 pose data
            if (mt1Pose != null) {
                hasMT1Pose = true;
                mt1X = mt1Pose.getPosition().x;
                mt1Y = mt1Pose.getPosition().y;
                mt1Heading = Math.toRadians(mt1Pose.getOrientation().getYaw());
            } else {
                hasMT1Pose = false;
                mt1X = mt1Y = mt1Heading = 0;
            }

            // Extract MT2 pose data
            if (mt2Pose != null) {
                hasMT2Pose = true;
                mt2X = mt2Pose.getPosition().x;
                mt2Y = mt2Pose.getPosition().y;
                mt2Heading = Math.toRadians(mt2Pose.getOrientation().getYaw());
            } else {
                hasMT2Pose = false;
                mt2X = mt2Y = mt2Heading = 0;
            }

            // Select which pose to use based on forceMT1 flag
            if (!forceMT1 && mt2Pose != null) {
                usingMT2 = true;
                botPose = mt2Pose;
            } else {
                // Force MT1 mode or MT2 unavailable
                usingMT2 = false;
                botPose = mt1Pose;
            }

            // Extract selected pose data for main API
            if (botPose != null) {
                hasBotPose = true;
                robotX = botPose.getPosition().x;
                robotY = botPose.getPosition().y;
                robotHeading = Math.toRadians(botPose.getOrientation().getYaw());

                // Get the appropriate goal position based on alliance
                double goalX = isRedAlliance ? RED_GOAL_X : BLUE_GOAL_X;
                double goalY = isRedAlliance ? RED_GOAL_Y : BLUE_GOAL_Y;

                // Euclidean distance from robot to goal
                distanceToGoal = Math.hypot(goalX - robotX, goalY - robotY);
            } else {
                hasBotPose = false;
                distanceToGoal = 0;
                robotHeading = 0;
            }
        } else {
            hasValidTarget = false;
            hasBotPose = false;
            hasMT1Pose = false;
            hasMT2Pose = false;
            usingMT2 = false;
            tx = 0;
            ty = 0;
            ta = 0;
            botPose = null;
            mt1Pose = null;
            mt2Pose = null;
            mt1X = mt1Y = mt1Heading = 0;
            mt2X = mt2Y = mt2Heading = 0;
            distanceToGoal = 0;
            robotHeading = 0;
        }
    }

    @Override
    public void act() {
        // PHASE 3: Nothing needed - Vision is read-only
    }

    /**
     * Check if Limelight has a valid target in view.
     */
    public boolean hasTarget() {
        return hasValidTarget;
    }

    /**
     * Check if we have a valid botpose for field localization.
     * Distance calculations require botpose.
     */
    public boolean hasBotPose() {
        return hasBotPose;
    }

    /**
     * Get horizontal offset to target.
     *
     * @return tx in degrees. Negative = target is left, Positive = right
     */
    public double getTx() {
        return tx;
    }

    /**
     * Get vertical offset to target.
     *
     * @return ty in degrees. Used for distance calculation.
     */
    public double getTy() {
        return ty;
    }

    /**
     * Get target area.
     *
     * @return ta as percentage (0-100)
     */
    public double getTa() {
        return ta;
    }

    // ==================== TARGET DISTANCE ESTIMATION ====================
    // For tracking a handheld AprilTag (not field-fixed goal)
    // Distance estimated from target area using inverse square law: distance = k / sqrt(ta)
    public static double TARGET_DISTANCE_K = 30.0;  // Calibration constant (tune with known distance)

    /**
     * Get estimated distance to the current target based on apparent size.
     * Uses the inverse square law: distance ∝ 1/sqrt(area).
     * Calibrate TARGET_DISTANCE_K by measuring actual distance when ta is known.
     *
     * @return Estimated distance in inches (0 if no valid target)
     */
    public double getTargetDistanceInches() {
        if (!hasValidTarget || ta <= 0) {
            return 0;
        }
        // Inverse square law: as distance doubles, area quarters
        // So distance = k / sqrt(ta)
        return TARGET_DISTANCE_K / Math.sqrt(ta);
    }

    /**
     * Get calculated distance to goal.
     * Uses botpose field localization + known goal positions.
     *
     * @return Distance in meters (0 if no valid botpose)
     */
    public double getDistanceToGoal() {
        return distanceToGoal;
    }

    /**
     * Get robot's X position on field (from botpose).
     * @return X in meters from field center
     */
    public double getRobotX() {
        return robotX;
    }

    /**
     * Get robot's Y position on field (from botpose).
     * @return Y in meters from field center
     */
    public double getRobotY() {
        return robotY;
    }

    /**
     * Get robot's heading from botpose.
     * @return Heading in radians
     */
    public double getRobotHeading() {
        return robotHeading;
    }

    /**
     * Check if currently using MegaTag2 localization.
     * MT2 uses IMU heading for better single-tag accuracy.
     * Falls back to MT1 when heading is unavailable.
     */
    public boolean isUsingMT2() {
        return usingMT2;
    }

    /**
     * Force MT1 mode (skip MT2 even if heading is available).
     * Use when initial robot position is unknown and Pinpoint heading may be wrong.
     *
     * @param force true to force MT1 mode, false to allow MT2
     */
    public void setForceMT1(boolean force) {
        this.forceMT1 = force;
    }

    /**
     * Check if MT1 mode is being forced.
     */
    public boolean isForcingMT1() {
        return forceMT1;
    }

    /**
     * Get bot pose from AprilTag localization.
     *
     * @return Pose3D or null if not available
     */
    public Pose3D getBotPose() {
        return botPose;
    }

    // ==================== DASHBOARD STREAMING ====================

    /**
     * Start streaming frames to FTC Dashboard.
     * Fetches frames from Limelight's MJPEG stream on a background thread.
     */
    public void startDashboardStream() {
        if (limelightStream == null) {
            limelightStream = new LimelightStream();
        }
        limelightStream.setTargetFps(STREAM_FPS);
        limelightStream.start();
    }

    /**
     * Stop streaming frames to FTC Dashboard.
     */
    public void stopDashboardStream() {
        if (limelightStream != null) {
            limelightStream.stop();
        }
    }

    /**
     * Get a new frame for dashboard display, scaled down to reduce bandwidth.
     * Returns null if streaming is not active, no frame available, or no new frame since last call.
     * Only returns each frame once to avoid redundant dashboard updates.
     *
     * @return Scaled Bitmap frame or null
     */
    public Bitmap getDashboardFrame() {
        if (limelightStream == null) return null;

        Bitmap frame = limelightStream.getNewFrame();
        if (frame == null) return null;

        // Scale down to reduce bandwidth and CPU load
        double scale = Math.max(0.1, Math.min(1.0, STREAM_SCALE));
        if (scale < 1.0) {
            int newWidth = (int) (frame.getWidth() * scale);
            int newHeight = (int) (frame.getHeight() * scale);
            if (newWidth > 0 && newHeight > 0) {
                try {
                    Bitmap scaled = Bitmap.createScaledBitmap(frame, newWidth, newHeight, true);
                    return scaled;
                } catch (Exception e) {
                    // If scaling fails, return original
                    return frame;
                }
            }
        }
        return frame;
    }

    /**
     * Check if there's a new frame available (not yet sent to dashboard).
     */
    public boolean hasNewDashboardFrame() {
        return limelightStream != null && limelightStream.hasNewFrame();
    }

    /**
     * Check if dashboard streaming is currently active.
     */
    public boolean isDashboardStreamActive() {
        return limelightStream != null && limelightStream.isRunning();
    }

    /**
     * Get dashboard stream status for telemetry.
     */
    public String getDashboardStreamStatus() {
        if (limelightStream == null) return "Not initialized";
        return limelightStream.getStatus();
    }

    /**
     * Set alliance color and switch to appropriate pipeline.
     *
     * @param isRed true for red alliance, false for blue
     */
    public void setAlliance(boolean isRed) {
        isRedAlliance = isRed;
        currentPipeline = isRed ? Pipeline.RED : Pipeline.BLUE;
        limelight.pipelineSwitch(currentPipeline.id);
    }

    /**
     * Check if configured for red alliance.
     */
    public boolean isRedAlliance() {
        return isRedAlliance;
    }

    /**
     * Switch to a specific pipeline.
     *
     * @param pipeline Pipeline enum value
     */
    public void setPipeline(Pipeline pipeline) {
        currentPipeline = pipeline;
        limelight.pipelineSwitch(pipeline.id);
    }

    /**
     * Get current pipeline.
     */
    public Pipeline getCurrentPipeline() {
        return currentPipeline;
    }

    public void setTiltPosition(){
        if(angle == Angle.DOWN){
            tiltTicks=1615;
        }else if(angle == Angle.STRAIGHT){
            tiltTicks=1365;
        }else if(angle == Angle.UPMAX){
            tiltTicks=1200;
        }else{
            tiltTicks=1240;
        }
    }

    public void setTiltDown(){
        tilt.setPosition(servoNormalize(1615));
        angle = Angle.DOWN;
    }
    public void setTiltStraight(){
        tilt.setPosition(servoNormalize(1365));
        angle = Angle.STRAIGHT;
    }
    public void setTiltUpMin(){
        tilt.setPosition(servoNormalize(1240));
        angle = Angle.UPMIN;
    }
    public void setTiltUpMax(){
        tilt.setPosition(servoNormalize(1200));
        angle = Angle.UPMAX;
    }


    public static double servoNormalize(int pulse) {
        double normalized = (double) pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

    @Override
    public void stop() {
        // Stop dashboard stream if running
        stopDashboardStream();
    }

    @Override
    public void resetStates() {
        // No states to reset
    }

    @Override
    public String getTelemetryName() {
        return "Vision";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Tilt State: ", angle);

        telemetry.put("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.put("Has Target", hasValidTarget ? "YES" : "no");
        telemetry.put("Has BotPose", hasBotPose ? "YES" : "no");

        if (hasBotPose) {
            telemetry.put("MT Mode", usingMT2 ? "MT2 (active)" : "MT1 (active)");
            telemetry.put("Distance to Goal", String.format("%.2f m (%.1f in)", distanceToGoal, distanceToGoal * 39.37));
        }

        // Always show both MT1 and MT2 poses for comparison when available
        if (hasMT1Pose) {
            telemetry.put("MT1 Pos", String.format("(%.2f, %.2f) m", mt1X, mt1Y));
            telemetry.put("MT1 Heading", String.format("%.1f°", Math.toDegrees(mt1Heading)));
        }
        if (hasMT2Pose) {
            telemetry.put("MT2 Pos", String.format("(%.2f, %.2f) m", mt2X, mt2Y));
            telemetry.put("MT2 Heading", String.format("%.1f°", Math.toDegrees(mt2Heading)));
        }

        // Show difference between MT1 and MT2 when both are available
        if (hasMT1Pose && hasMT2Pose) {
            double posDiff = Math.hypot(mt2X - mt1X, mt2Y - mt1Y);
            double headingDiff = Math.toDegrees(mt2Heading - mt1Heading);
            // Normalize heading difference to -180 to 180
            while (headingDiff > 180) headingDiff -= 360;
            while (headingDiff < -180) headingDiff += 360;
            telemetry.put("MT Diff", String.format("%.3f m, %.1f°", posDiff, headingDiff));
        }

        if (hasValidTarget) {
            telemetry.put("tx (aim)", String.format("%.1f°", tx));
        }

        // Dashboard stream status
        if (limelightStream != null) {
            telemetry.put("Stream", getDashboardStreamStatus());
        }

        if (debug) {
            telemetry.put("Pipeline", currentPipeline);
            telemetry.put("ty", String.format("%.1f°", ty));
            telemetry.put("ta (area)", String.format("%.1f%%", ta));
            if (botPose != null) {
                telemetry.put("BotPose X (m)", String.format("%.3f", botPose.getPosition().x));
                telemetry.put("BotPose Y (m)", String.format("%.3f", botPose.getPosition().y));
                telemetry.put("BotPose Z (m)", String.format("%.3f", botPose.getPosition().z));
                telemetry.put("BotPose Yaw (deg)", String.format("%.1f", botPose.getOrientation().getYaw()));
                telemetry.put("Goal Pos Used", String.format("(%.2f, %.2f)",
                    isRedAlliance ? RED_GOAL_X : BLUE_GOAL_X,
                    isRedAlliance ? RED_GOAL_Y : BLUE_GOAL_Y));
            }
        }

        return telemetry;
    }
}
