package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.LazyServo;

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
    private int tiltTicks=1200;

    private Angle angle = Angle.UPMAX;

    // Configuration
    public static int PIPELINE_BLUE = 0;
    public static int PIPELINE_RED = 1;

    // DECODE Field Goal Positions (meters, from field center)
    // Source: https://downloads.limelightvision.io/models/ftc2025DECODE.fmap
    public static final double BLUE_GOAL_X = -1.4827;  // Tag 20
    public static final double BLUE_GOAL_Y = -1.4133;
    public static final double RED_GOAL_X = -1.4827;   // Tag 24
    public static final double RED_GOAL_Y = 1.4133;

    // State
    private boolean isRedAlliance = true;
    private int currentPipeline = PIPELINE_RED;
    private boolean hasValidTarget = false;
    private boolean hasBotPose = false;
    private double tx = 0;  // Horizontal offset (degrees) - for aiming
    private double ty = 0;  // Vertical offset (degrees)
    private double ta = 0;  // Target area (0-100% of image)
    private Pose3D botPose = null;
    private double distanceToGoal = 0;  // Calculated from botpose + known goal position
    private double robotX = 0;  // Robot field position (meters)
    private double robotY = 0;

    public Vision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(currentPipeline);
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

    @Override
    public void calc(Canvas fieldOverlay) {
        // PHASE 2: Get latest Limelight result and extract data
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            hasValidTarget = true;
            tx = result.getTx();
            ty = result.getTy();
            ta = result.getTa();
            botPose = result.getBotpose_MT2();

            // Calculate distance using botpose (field localization)
            if (botPose != null) {
                hasBotPose = true;
                robotX = botPose.getPosition().x;
                robotY = botPose.getPosition().y;

                // Get the appropriate goal position based on alliance
                double goalX = isRedAlliance ? RED_GOAL_X : BLUE_GOAL_X;
                double goalY = isRedAlliance ? RED_GOAL_Y : BLUE_GOAL_Y;

                // Euclidean distance from robot to goal
                distanceToGoal = Math.hypot(goalX - robotX, goalY - robotY);
            } else {
                hasBotPose = false;
                distanceToGoal = 0;
            }
        } else {
            hasValidTarget = false;
            hasBotPose = false;
            tx = 0;
            ty = 0;
            ta = 0;
            botPose = null;
            distanceToGoal = 0;
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
     * Get bot pose from AprilTag localization.
     *
     * @return Pose3D or null if not available
     */
    public Pose3D getBotPose() {
        return botPose;
    }

    /**
     * Set alliance color and switch to appropriate pipeline.
     *
     * @param isRed true for red alliance, false for blue
     */
    public void setAlliance(boolean isRed) {
        isRedAlliance = isRed;
        currentPipeline = isRed ? PIPELINE_RED : PIPELINE_BLUE;
        limelight.pipelineSwitch(currentPipeline);
    }

    /**
     * Check if configured for red alliance.
     */
    public boolean isRedAlliance() {
        return isRedAlliance;
    }

    /**
     * Manually switch to a specific pipeline.
     *
     * @param pipeline Pipeline index (0-9)
     */
    public void setPipeline(int pipeline) {
        currentPipeline = pipeline;
        limelight.pipelineSwitch(pipeline);
    }

    /**
     * Get current pipeline.
     */
    public int getCurrentPipeline() {
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
        // Limelight doesn't need explicit stop
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
            telemetry.put("Distance to Goal", String.format("%.2f m (%.1f in)", distanceToGoal, distanceToGoal * 39.37));
            telemetry.put("Robot Pos", String.format("(%.2f, %.2f) m", robotX, robotY));
        }

        if (hasValidTarget) {
            telemetry.put("tx (aim)", String.format("%.1f°", tx));
        }

        if (debug) {
            telemetry.put("Pipeline", currentPipeline);
            telemetry.put("ty", String.format("%.1f°", ty));
            telemetry.put("ta (area)", String.format("%.1f%%", ta));
            if (botPose != null) {
                telemetry.put("Full BotPose", botPose.toString());
            }
        }

        return telemetry;
    }
}
