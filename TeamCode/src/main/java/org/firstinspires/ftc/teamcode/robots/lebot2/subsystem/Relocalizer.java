package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

import org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.TankDrivePinpoint;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Background relocalization collector.
 *
 * Runs every cycle. Collects vision botpose samples when the robot is stationary,
 * the turret is still, and vision has a valid pose. Averages samples with outlier
 * rejection and exposes a ready/apply API.
 *
 * External code only needs:
 *   - isReady()  — is a validated correction available?
 *   - apply()    — write it to the drivetrain and reset
 *   - update()   — called every cycle from Robot.update()
 */
@Config(value = "Lebot2_Relocalizer")
public class Relocalizer {

    // ==================== CONSTANTS (Dashboard tunable) ====================

    public static int MIN_SAMPLES = 3;
    public static int MAX_SAMPLES = 15;
    public static double WINDOW_MS = 500;                   // max age of oldest sample
    public static double MAX_TRANSLATION_VEL = 2.0;         // in/s
    public static double MAX_ROTATION_VEL = 5.0;            // deg/s
    public static double MAX_TURRET_VEL = 3.0;              // deg/s
    public static double MAX_SPREAD_INCHES = 6.0;           // ~15cm position spread
    public static double MAX_HEADING_SPREAD_DEG = 5.0;
    public static double AUDIENCE_RANGE_THRESHOLD_M = 2.3;  // relax spread beyond this
    public static double AUDIENCE_SPREAD_MULTIPLIER = 1.5;  // fixed, not cumulative
    public static double OUTLIER_REJECTION_FACTOR = 2.0;    // reject > 2*MAD from median
    public static double MAX_OUTLIER_FRACTION = 0.3;        // flush if >30% rejected

    // Camera offset from center of rotation (meters)
    public static double CAMERA_FORWARD_OFFSET_M = 0.33;
    public static double CAMERA_SIDE_OFFSET_M = 0.0;

    // ==================== REFERENCES ====================

    private final Vision vision;
    private final TankDrivePinpoint driveTrain;
    private final Turret turret;

    // ==================== STATE ====================

    private final List<TimestampedPose> samples = new ArrayList<>();
    private double previousTurretAngle = 0;
    private long previousTurretTime = 0;
    private boolean relocReady = false;
    private Pose2d averagedPose = null;
    private int lastSurvivorCount = 0;
    private double lastSpread = 0;

    private static class TimestampedPose {
        final Pose2d pose;
        final long timestampMs;

        TimestampedPose(Pose2d pose, long timestampMs) {
            this.pose = pose;
            this.timestampMs = timestampMs;
        }
    }

    // ==================== CONSTRUCTOR ====================

    public Relocalizer(Vision vision, TankDrivePinpoint driveTrain, Turret turret) {
        this.vision = vision;
        this.driveTrain = driveTrain;
        this.turret = turret;
    }

    // ==================== PUBLIC API ====================

    /**
     * Run every cycle from Robot.update(). Handles collection, evaluation, and flushing.
     */
    public void update() {
        long now = System.currentTimeMillis();

        // --- Gate checks ---
        if (!vision.hasBotPose()) {
            flush();
            return;
        }

        PoseVelocity2d vel = driveTrain.getVelocity();
        double translationSpeed = Math.hypot(vel.linearVel.x, vel.linearVel.y);
        if (translationSpeed > MAX_TRANSLATION_VEL) {
            flush();
            return;
        }

        double rotationSpeed = Math.abs(Math.toDegrees(vel.angVel));
        if (rotationSpeed > MAX_ROTATION_VEL) {
            flush();
            return;
        }

        double turretAngle = turret.getTurretAngleDeg();
        double turretRate = computeTurretRate(turretAngle, now);
        if (turretRate > MAX_TURRET_VEL) {
            flush();
            return;
        }

        // --- Collect ---
        Pose2d sample = transformBotposeToRobotCenter();
        samples.add(new TimestampedPose(sample, now));

        // Drop stale samples
        long cutoff = now - (long) WINDOW_MS;
        while (!samples.isEmpty() && samples.get(0).timestampMs < cutoff) {
            samples.remove(0);
        }

        // --- Evaluate ---
        if (samples.size() < MIN_SAMPLES) {
            relocReady = false;
            return;
        }

        // Extract pose list for processing
        List<Pose2d> poses = new ArrayList<>();
        for (TimestampedPose ts : samples) {
            poses.add(ts.pose);
        }

        // Median-based outlier rejection
        double medianX = median(poses, true);
        double medianY = median(poses, false);
        double madX = mad(poses, medianX, true);
        double madY = mad(poses, medianY, false);
        double threshold = OUTLIER_REJECTION_FACTOR * Math.max(madX, madY);
        if (threshold < 0.5) threshold = 0.5; // Floor to avoid rejecting everything on tight data

        List<Pose2d> survivors = new ArrayList<>();
        for (Pose2d p : poses) {
            double dist = Math.hypot(p.position.x - medianX, p.position.y - medianY);
            if (dist <= threshold) {
                survivors.add(p);
            }
        }

        // Check outlier fraction
        double rejectedFraction = 1.0 - ((double) survivors.size() / poses.size());
        if (rejectedFraction > MAX_OUTLIER_FRACTION) {
            if (samples.size() >= MAX_SAMPLES) {
                flush(); // Too noisy, give up
            }
            relocReady = false;
            return;
        }

        if (survivors.size() < MIN_SAMPLES) {
            if (samples.size() >= MAX_SAMPLES) {
                flush();
            }
            relocReady = false;
            return;
        }

        // Check position spread
        double spread = maxDistFromMedian(survivors, medianX, medianY);
        double effectiveSpread = MAX_SPREAD_INCHES;
        if (vision.getDistanceToGoal() > AUDIENCE_RANGE_THRESHOLD_M) {
            effectiveSpread *= AUDIENCE_SPREAD_MULTIPLIER;
        }

        // Check heading spread
        double headingSpread = maxHeadingDeviation(survivors);

        lastSurvivorCount = survivors.size();
        lastSpread = spread;

        if (spread < effectiveSpread && headingSpread < MAX_HEADING_SPREAD_DEG) {
            relocReady = true;
            averagedPose = averagePoses(survivors);
        } else if (samples.size() >= MAX_SAMPLES) {
            flush(); // 15 samples and still can't converge
            relocReady = false;
        } else {
            relocReady = false;
        }
    }

    /**
     * Is a validated correction ready to apply?
     */
    public boolean isReady() {
        return relocReady;
    }

    /**
     * Apply the averaged pose to the drivetrain and reset.
     * @return true if applied, false if not ready
     */
    public boolean apply() {
        if (!relocReady || averagedPose == null) return false;
        driveTrain.setPose(averagedPose);
        flush();
        return true;
    }

    /**
     * Get the averaged pose without applying it (for telemetry/preview).
     */
    public Pose2d getAveragedPose() {
        return averagedPose;
    }

    /**
     * Clear all samples and reset state.
     */
    public void flush() {
        samples.clear();
        relocReady = false;
        averagedPose = null;
        lastSurvivorCount = 0;
        lastSpread = 0;
    }

    // ==================== TELEMETRY ====================

    public Map<String, Object> getTelemetry() {
        Map<String, Object> t = new LinkedHashMap<>();
        t.put("Samples", samples.size());
        t.put("Survivors", lastSurvivorCount);
        t.put("Spread (in)", String.format("%.1f", lastSpread));
        t.put("Ready", relocReady ? "YES" : "no");
        if (averagedPose != null) {
            t.put("Avg Pose", String.format("(%.1f, %.1f) %.1f°",
                    averagedPose.position.x, averagedPose.position.y,
                    Math.toDegrees(averagedPose.heading.toDouble())));
        }
        return t;
    }

    // ==================== INTERNALS ====================

    private void flush(String reason) {
        flush();
    }

    private double computeTurretRate(double currentAngle, long now) {
        if (previousTurretTime == 0) {
            previousTurretAngle = currentAngle;
            previousTurretTime = now;
            return 0;
        }
        double dt = (now - previousTurretTime) / 1000.0;
        double rate = (dt > 0) ? Math.abs(currentAngle - previousTurretAngle) / dt : 0;
        previousTurretAngle = currentAngle;
        previousTurretTime = now;
        return rate;
    }

    private Pose2d transformBotposeToRobotCenter() {
        double camX = vision.getRobotX();
        double camY = vision.getRobotY();
        double headingRad = vision.getRobotHeading();

        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);

        double robotX = camX - CAMERA_FORWARD_OFFSET_M * cosH + CAMERA_SIDE_OFFSET_M * sinH;
        double robotY = camY - CAMERA_FORWARD_OFFSET_M * sinH - CAMERA_SIDE_OFFSET_M * cosH;

        double xInches = robotX * 39.3701;
        double yInches = robotY * 39.3701;

        return new Pose2d(xInches, yInches, headingRad);
    }

    private static Pose2d averagePoses(List<Pose2d> poses) {
        double sumX = 0, sumY = 0, sinSum = 0, cosSum = 0;
        for (Pose2d p : poses) {
            sumX += p.position.x;
            sumY += p.position.y;
            double h = p.heading.toDouble();
            sinSum += Math.sin(h);
            cosSum += Math.cos(h);
        }
        int n = poses.size();
        return new Pose2d(sumX / n, sumY / n, Math.atan2(sinSum / n, cosSum / n));
    }

    private static double median(List<Pose2d> poses, boolean useX) {
        List<Double> vals = new ArrayList<>();
        for (Pose2d p : poses) {
            vals.add(useX ? p.position.x : p.position.y);
        }
        Collections.sort(vals);
        int n = vals.size();
        if (n % 2 == 0) {
            return (vals.get(n / 2 - 1) + vals.get(n / 2)) / 2.0;
        }
        return vals.get(n / 2);
    }

    private static double mad(List<Pose2d> poses, double median, boolean useX) {
        List<Double> deviations = new ArrayList<>();
        for (Pose2d p : poses) {
            double val = useX ? p.position.x : p.position.y;
            deviations.add(Math.abs(val - median));
        }
        Collections.sort(deviations);
        int n = deviations.size();
        if (n % 2 == 0) {
            return (deviations.get(n / 2 - 1) + deviations.get(n / 2)) / 2.0;
        }
        return deviations.get(n / 2);
    }

    private static double maxDistFromMedian(List<Pose2d> poses, double medX, double medY) {
        double max = 0;
        for (Pose2d p : poses) {
            double d = Math.hypot(p.position.x - medX, p.position.y - medY);
            if (d > max) max = d;
        }
        return max;
    }

    private static double maxHeadingDeviation(List<Pose2d> poses) {
        // Circular median for heading
        List<Double> headings = new ArrayList<>();
        for (Pose2d p : poses) {
            headings.add(p.heading.toDouble());
        }
        // Use circular mean as reference (close enough for small spreads)
        double sinSum = 0, cosSum = 0;
        for (double h : headings) {
            sinSum += Math.sin(h);
            cosSum += Math.cos(h);
        }
        double refHeading = Math.atan2(sinSum / headings.size(), cosSum / headings.size());

        double maxDev = 0;
        for (double h : headings) {
            double dev = Math.abs(normalizeAngle(h - refHeading));
            if (dev > maxDev) maxDev = dev;
        }
        return Math.toDegrees(maxDev);
    }

    private static double normalizeAngle(double rad) {
        while (rad > Math.PI) rad -= 2 * Math.PI;
        while (rad < -Math.PI) rad += 2 * Math.PI;
        return rad;
    }
}
