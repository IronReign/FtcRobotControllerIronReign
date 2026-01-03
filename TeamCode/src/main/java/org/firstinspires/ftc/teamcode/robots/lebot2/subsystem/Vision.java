package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Vision subsystem - wraps the Limelight for target detection.
 *
 * The Limelight is used for:
 * - Detecting AprilTags (alliance-specific targets)
 * - Providing tx (horizontal offset) for fine targeting
 * - Providing ty (vertical offset) for distance calculation
 * - Optional: Bot pose via AprilTag localization
 *
 * Pipeline switching allows detecting different alliance colors.
 */
@Config(value = "Lebot2_Vision")
public class Vision implements Subsystem {

    // Hardware
    private final Limelight3A limelight;

    // Configuration
    public static int PIPELINE_BLUE = 0;
    public static int PIPELINE_RED = 1;

    // Limelight mounting parameters (for distance calculation)
    public static double MOUNT_ANGLE_DEGREES = 29.5;  // Angle from horizontal
    public static double MOUNT_HEIGHT_METERS = 0.3048; // 12 inches
    public static double TARGET_HEIGHT_METERS = 0.757; // ~29.8 inches

    // State
    private boolean isRedAlliance = true;
    private int currentPipeline = PIPELINE_RED;
    private boolean hasValidTarget = false;
    private double tx = 0;  // Horizontal offset (degrees)
    private double ty = 0;  // Vertical offset (degrees)
    private double ta = 0;  // Target area (0-100% of image)
    private Pose3D botPose = null;
    private double distanceToTarget = 0; // Calculated from ty

    public Vision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(currentPipeline);
        limelight.start();
    }

    @Override
    public void update(Canvas fieldOverlay) {
        // READ: Get latest Limelight result
        LLResult result = limelight.getLatestResult();

        // PROCESS: Extract data from result
        if (result != null && result.isValid()) {
            hasValidTarget = true;
            tx = result.getTx();
            ty = result.getTy();
            ta = result.getTa();
            botPose = result.getBotpose_MT2();

            // Calculate distance from ty using trigonometry
            distanceToTarget = calculateDistanceFromTy(ty);
        } else {
            hasValidTarget = false;
            tx = 0;
            ty = 0;
            ta = 0;
            botPose = null;
            distanceToTarget = 0;
        }
    }

    /**
     * Calculate horizontal distance to target using vertical angle.
     *
     * Uses the formula:
     * distance = (targetHeight - lensHeight) / tan(mountAngle + ty)
     *
     * @param ty Vertical offset from Limelight (degrees)
     * @return Distance in meters
     */
    public double calculateDistanceFromTy(double ty) {
        double angleToTarget = Math.toRadians(MOUNT_ANGLE_DEGREES + ty);
        double heightDiff = TARGET_HEIGHT_METERS - MOUNT_HEIGHT_METERS;

        if (Math.tan(angleToTarget) <= 0) {
            return 0; // Invalid angle
        }

        return heightDiff / Math.tan(angleToTarget);
    }

    /**
     * Check if Limelight has a valid target in view.
     */
    public boolean hasTarget() {
        return hasValidTarget;
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
     * Get calculated distance to target.
     *
     * @return Distance in meters
     */
    public double getDistanceToTarget() {
        return distanceToTarget;
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

        telemetry.put("Has Target", hasValidTarget ? "YES" : "no");
        telemetry.put("Alliance", isRedAlliance ? "RED" : "BLUE");

        if (hasValidTarget) {
            telemetry.put("tx (horiz)", String.format("%.1f°", tx));
            telemetry.put("Distance", String.format("%.2f m", distanceToTarget));
        }

        if (debug) {
            telemetry.put("Pipeline", currentPipeline);
            telemetry.put("ty (vert)", String.format("%.1f°", ty));
            telemetry.put("ta (area)", String.format("%.1f%%", ta));
            if (botPose != null) {
                telemetry.put("Bot Pose", botPose.toString());
            }
        }

        return telemetry;
    }
}
