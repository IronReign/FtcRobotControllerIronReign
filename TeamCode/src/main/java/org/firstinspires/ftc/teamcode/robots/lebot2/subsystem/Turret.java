package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.lebot2.FieldMap;
import org.firstinspires.ftc.teamcode.robots.lebot2.Robot;
import org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.TankDrivePinpoint;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;

import org.firstinspires.ftc.teamcode.util.CsvLogKeeper;

@Config(value = "Lebot2_Turret")
public class Turret implements Subsystem {

    public static double BUFFER = 2.5;

    private ElapsedTime lostVisionTimer = new ElapsedTime();
    public static double LOST_VISION_TIME = .2;  // Legacy — replaced by tiered response below

    // ==================== THREE-TIER VISION LOSS RESPONSE ====================
    // Tier 1: Hold current turret angle (covers glitches + short robot occlusions)
    // Tier 2: Fall back to odo-based bearing (only if odo is trusted)
    // Tier 3: Fire from held position (better than not firing at all)
    public static double HOLD_POSITION_S = 0.3;     // Tier 1 duration (seconds)
    public static double ODO_FALLBACK_S  = 2.0;     // Tier 2 cutoff (seconds)
    // Beyond ODO_FALLBACK_S: Tier 3 — hold position + signal degraded readiness

    // ==================== ODO TRUST TRACKING ====================
    public static double ODO_TRUST_THRESHOLD_DEG = 10.0;  // Max vision/odo bearing divergence to trust odo
    public static double ODO_TRUST_STALE_S = 5.0;         // If divergence hasn't been checked in this long, mark untrusted
    private boolean odoTrusted = false;
    private double lastDivergenceDeg = 0;
    private long lastDivergenceTimeMs = 0;

    private boolean readyToLaunchDegraded = false;

    // ==================== OSCILLATION-AWARE LOCK ====================
    // Graduated readiness: perfect lock fires immediately, good-enough fires after timeout
    public static double LOCK_WINDOW_MS = 500;       // Rolling window for oscillation measurement
    public static double PERFECT_LOCK_DEG = 1.0;     // Amplitude < this = perfect lock
    public static double GOOD_ENOUGH_DEG = 3.0;      // Amplitude < this = acceptable after timeout
    public static double LOCK_TIMEOUT_MS = 1000;      // Wait this long before accepting good-enough
    public static double LOCK_HARD_CEILING_MS = 2000; // Fire anyway if amplitude < 2*GOOD_ENOUGH_DEG
    public static double LAUNCH_TOLERANCE = 1;        // Legacy — kept for compatibility

    // Rolling window of (timestamp, tx) samples
    private static final int TX_HISTORY_SIZE = 50;    // Ring buffer capacity
    private final long[] txTimestamps = new long[TX_HISTORY_SIZE];
    private final double[] txValues = new double[TX_HISTORY_SIZE];
    private int txHistoryIndex = 0;
    private int txHistoryCount = 0;

    // Lock state
    private ElapsedTime aimingTimer = new ElapsedTime();  // Time since tracking started on current target
    private ElapsedTime perfectLockTimer = new ElapsedTime();  // Time since amplitude dropped below PERFECT
    private boolean inPerfectLock = false;
    public boolean readyToLaunch = false;
    private double currentOscillationDeg = 0;  // Exposed for telemetry

    public enum LockQuality { NO_VISION, SEEKING, GOOD_ENOUGH, PERFECT }
    private LockQuality lockQuality = LockQuality.NO_VISION;

    // ==================== CSV LOGGING FOR PID TUNING ====================
    public static boolean TURRET_LOGGING_ENABLED = false;  // Dashboard tunable
    private CsvLogKeeper turretLog = null;
    private long logStartTime = 0;

    // ==================== ENCODER / MECHANICAL LIMITS ====================
    // Calibrate these on the physical robot:
    //   Center the turret manually, read encoder -> TURRET_CENTER_TICKS
    //   Rotate CW to stop, read encoder -> CW_LIMIT_TICKS
    //   Rotate CCW to stop, read encoder -> CCW_LIMIT_TICKS
    //   TICKS_PER_DEGREE = (CW_LIMIT_TICKS - CCW_LIMIT_TICKS) / 210.0
    public static int TURRET_CENTER_TICKS = 0;
    public static int FULL_ROTATION_TICKS = 2820;
    public static double DEGREES_FOR_ROTATION = 360;
    public static double FALL_BACK_TURN = 35;       //CCW RED
    public static int CW_LIMIT_TICKS = -705;     // placeholder - calibrate on robot
    public static int CCW_LIMIT_TICKS = 705;    // placeholder - calibrate on robot
    public static double TICKS_PER_DEGREE = FULL_ROTATION_TICKS / DEGREES_FOR_ROTATION;

    // Derived degree limits (computed from ticks, but also dashboard-tunable for quick adjustment)
    public static double CW_LIMIT_DEG = -90;
    public static double CCW_LIMIT_DEG = 90;

    // ==================== PID PARAMS ====================
    public static PIDCoefficients TURRET_PID = new PIDCoefficients(0.04, 0.00005, 0.0075);
    public static double MAX_SPEED = 1.0;
    public static double TOLERANCE = 1.0;          // degrees - PID convergence threshold
    public static double INTEGRAL_CUTIN = 3.0;     // degrees
    public static double EMA_ALPHA = 0.5;
    public static double VISION_OFFSET = 0;        // camera center offset in degrees

    // ==================== HARDWARE ====================
    public static boolean REVERSE_MOTOR = true;
    private final DcMotorEx turretMotor;

    // ==================== DEPENDENCIES ====================
    private Vision vision;
    private TankDrivePinpoint driveTrain;

    // ==================== PID ====================
    private final PIDController turretPID;

    // ==================== STATE ====================
    public enum Behavior {
        TRACKING,   // Autonomous target seeking: vision when available, pose-based bearing fallback
        LOCKED      // PID holds turret at 0° (forward). Safety fallback.
    }

    public enum TargetingPhase {
        VISION_TRACKING,    // Vision has target, PID on tx
        POSE_SEEKING,       // No vision, PID on pose-computed bearing
        HOLDING,            // Vision lost briefly, holding current robot-frame angle
        AT_LIMIT,           // Desired angle clamped to mechanical limit
        LOCKED_FORWARD      // Behavior.LOCKED - holding at 0°
    }

    private Behavior behavior = Behavior.LOCKED;
    private TargetingPhase phase = TargetingPhase.LOCKED_FORWARD;

    // Sensor reads (Phase 1: readSensors)
    private int encoderTicks = 0;
    private double turretAngleDeg = 0;

    // Staged output (Phase 2: calc -> Phase 3: act)
    private double stagedPower = 0;

    // ==================== CONSTRUCTOR ====================

    public Turret(HardwareMap hardwareMap) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        if (REVERSE_MOTOR) {
            turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        }
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turretPID = new PIDController(TURRET_PID);
        turretPID.setInputRange(-180, 180);
        turretPID.setOutputRange(-MAX_SPEED, MAX_SPEED);
        turretPID.setIntegralCutIn(INTEGRAL_CUTIN);
        turretPID.setContinuous(false);
        turretPID.setTolerance(TOLERANCE / 360 * 100);
        turretPID.setEmaAlpha(EMA_ALPHA);
        turretPID.enable();
    }


    public void resetTurret(){
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ==================== THREE-PHASE SUBSYSTEM ====================

    @Override
    public void readSensors() {
        encoderTicks = turretMotor.getCurrentPosition();
        if (TICKS_PER_DEGREE != 0) {
            turretAngleDeg = (encoderTicks - TURRET_CENTER_TICKS) / TICKS_PER_DEGREE;
        }
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        // Update PID tuning from dashboard
        turretPID.setPID(TURRET_PID);
        turretPID.setOutputRange(-MAX_SPEED, MAX_SPEED);

        // Oscillation-aware lock: track tx samples and compute readiness
        updateLockQuality();

        // CSV logging for PID tuning
        if (TURRET_LOGGING_ENABLED) {
            logTurretData();
        }

        if (behavior == Behavior.LOCKED) {
            // Hold turret at 0° (forward)
            phase = TargetingPhase.LOCKED_FORWARD;
            double targetAngle = clampToLimits(0);
            turretPID.setSetpoint(targetAngle);
            turretPID.setInput(turretAngleDeg);
            stagedPower = turretPID.performPID();
            lostVisionTimer.reset();

        } else { // TRACKING
            readyToLaunchDegraded = false;

            if (vision != null && vision.hasTarget()) {
                // ===== VISION ACTIVE =====
                lostVisionTimer.reset();

                // Update odo trust: compare vision bearing vs odo bearing while we have both
                updateOdoTrust();

                if (turretAngleDeg <= CCW_LIMIT_DEG && turretAngleDeg >= CW_LIMIT_DEG) {
                    // Within mechanical limits — PID on vision tx
                    phase = TargetingPhase.VISION_TRACKING;
                    turretPID.setSetpoint(VISION_OFFSET);
                    turretPID.setInput(vision.getTx());
                    stagedPower = turretPID.performPID();
                } else {
                    // Past mechanical limits — PID back toward nearest limit
                    phase = TargetingPhase.AT_LIMIT;
                    double nearestLimit = (turretAngleDeg < 0) ? CW_LIMIT_DEG + BUFFER : CCW_LIMIT_DEG - BUFFER;
                    turretPID.setSetpoint(nearestLimit);
                    turretPID.setInput(turretAngleDeg);
                    stagedPower = turretPID.performPID();
                }

            } else {
                // ===== VISION LOST — three-tier response =====
                double lostDuration = lostVisionTimer.seconds();

                // Check if odo trust has gone stale
                if (System.currentTimeMillis() - lastDivergenceTimeMs > ODO_TRUST_STALE_S * 1000) {
                    odoTrusted = false;
                }

                if (lostDuration < HOLD_POSITION_S) {
                    // Tier 1: HOLD — keep turret at its current angle
                    // Covers momentary dropouts, short robot occlusions (0-300ms)
                    phase = TargetingPhase.HOLDING;
                    turretPID.setSetpoint(turretAngleDeg);
                    turretPID.setInput(turretAngleDeg);
                    stagedPower = turretPID.performPID();

                } else if (lostDuration < ODO_FALLBACK_S && odoTrusted && driveTrain != null) {
                    // Tier 2: ODO BEARING — use pose-based bearing (only if odo is trusted)
                    Pose2d currentPose = driveTrain.getPose();
                    Pose2d goalPose = FieldMap.getPose(FieldMap.GOAL, Robot.isRedAlliance);
                    double bearingRad = FieldMap.bearingTo(currentPose, goalPose);
                    double chassisRad = currentPose.heading.toDouble();
                    double desiredTurretDeg = normalizeDeg(Math.toDegrees(bearingRad - chassisRad));

                    double clampedTarget = clampToLimits(desiredTurretDeg);
                    phase = (clampedTarget != desiredTurretDeg) ? TargetingPhase.AT_LIMIT : TargetingPhase.POSE_SEEKING;

                    turretPID.setSetpoint(clampedTarget);
                    turretPID.setInput(turretAngleDeg);
                    stagedPower = turretPID.performPID();

                } else {
                    // Tier 3: HOLD + DEGRADED READY — hold position, signal we can fire from here
                    // Vision has been gone long enough that we should fire and move on
                    phase = TargetingPhase.HOLDING;
                    turretPID.setSetpoint(turretAngleDeg);
                    turretPID.setInput(turretAngleDeg);
                    stagedPower = turretPID.performPID();
                    readyToLaunchDegraded = true;
                }
            }
        }
    }

    @Override
    public void act() {
        turretMotor.setPower(stagedPower);
    }

    // ==================== PUBLIC API ====================

    public void setVision(Vision vision) {
        this.vision = vision;
    }

    public void setDriveTrain(TankDrivePinpoint driveTrain) {
        this.driveTrain = driveTrain;
    }

    public void setTracking() {
        behavior = Behavior.TRACKING;
    }

    public void setLocked() {
        behavior = Behavior.LOCKED;
    }

    public Behavior getBehavior() {
        return behavior;
    }

    public TargetingPhase getPhase() {
        return phase;
    }

    public boolean isLockedOnTarget() {
        return phase == TargetingPhase.VISION_TRACKING && turretPID.lockedOnTarget();
    }

    public double getTurretAngleDeg() {
        return turretAngleDeg;
    }

    public boolean isReadyToLaunch(){return readyToLaunch;}

    public boolean isReadyToLaunchDegraded() { return readyToLaunchDegraded; }

    public LockQuality getLockQuality() { return lockQuality; }

    public double getOscillationDeg() { return currentOscillationDeg; }

    // ==================== HELPERS ====================

    /**
     * Record a tx sample and evaluate lock quality using oscillation amplitude
     * over a time-based rolling window.
     */
    private void updateLockQuality() {
        if (vision != null && vision.hasTarget() && phase == TargetingPhase.VISION_TRACKING) {
            // Record sample
            long now = System.currentTimeMillis();
            txTimestamps[txHistoryIndex] = now;
            txValues[txHistoryIndex] = vision.getTx();
            txHistoryIndex = (txHistoryIndex + 1) % TX_HISTORY_SIZE;
            if (txHistoryCount < TX_HISTORY_SIZE) txHistoryCount++;

            // Compute oscillation amplitude over the rolling window
            currentOscillationDeg = computeOscillation(now);
            double aimingTimeMs = aimingTimer.milliseconds();

            // Evaluate lock quality (priority order)
            if (currentOscillationDeg < PERFECT_LOCK_DEG) {
                if (!inPerfectLock) {
                    inPerfectLock = true;
                    perfectLockTimer.reset();
                }
                if (perfectLockTimer.milliseconds() > 100) {
                    lockQuality = LockQuality.PERFECT;
                    readyToLaunch = true;
                } else {
                    lockQuality = LockQuality.SEEKING;
                    readyToLaunch = false;
                }
            } else {
                inPerfectLock = false;

                if (currentOscillationDeg < GOOD_ENOUGH_DEG && aimingTimeMs > LOCK_TIMEOUT_MS) {
                    lockQuality = LockQuality.GOOD_ENOUGH;
                    readyToLaunch = true;
                } else if (currentOscillationDeg < 2 * GOOD_ENOUGH_DEG && aimingTimeMs > LOCK_HARD_CEILING_MS) {
                    lockQuality = LockQuality.GOOD_ENOUGH;
                    readyToLaunch = true;
                } else {
                    lockQuality = LockQuality.SEEKING;
                    readyToLaunch = false;
                }
            }
        } else {
            // No vision or not in vision tracking phase
            lockQuality = LockQuality.NO_VISION;
            readyToLaunch = false;
            inPerfectLock = false;
            // Reset aiming timer when we lose vision so it restarts on reacquisition
            aimingTimer.reset();
            txHistoryCount = 0;
        }
    }

    /**
     * Compute oscillation amplitude (max - min of tx) over the rolling time window.
     */
    private double computeOscillation(long now) {
        if (txHistoryCount == 0) return 999;  // No data = max oscillation

        double min = Double.MAX_VALUE;
        double max = -Double.MAX_VALUE;
        long windowStart = now - (long) LOCK_WINDOW_MS;
        int samplesInWindow = 0;

        for (int i = 0; i < txHistoryCount; i++) {
            if (txTimestamps[i] >= windowStart) {
                double val = txValues[i];
                if (val < min) min = val;
                if (val > max) max = val;
                samplesInWindow++;
            }
        }

        if (samplesInWindow < 3) return 999;  // Not enough data yet
        return max - min;
    }

    /**
     * CSV logging for PID tuning. Toggle via TURRET_LOGGING_ENABLED on Dashboard.
     * Logs: elapsedMs, phase, turretAngleDeg, tx, stagedPower, oscillation, lockQuality, readyToLaunch
     */
    private void logTurretData() {
        if (behavior == Behavior.TRACKING && logStartTime == 0) {
            turretLog = new CsvLogKeeper("turret_pid_log", 8,
                    "elapsedMs,phase,turretAngleDeg,tx,stagedPower,oscillation,lockQuality,readyToLaunch");
            logStartTime = System.currentTimeMillis();
        }
        if (logStartTime > 0 && behavior == Behavior.TRACKING) {
            ArrayList<Object> row = new ArrayList<>();
            row.add(System.currentTimeMillis() - logStartTime);
            row.add(phase.name());
            row.add(String.format("%.2f", turretAngleDeg));
            row.add(vision != null && vision.hasTarget() ? String.format("%.2f", vision.getTx()) : "N/A");
            row.add(String.format("%.4f", stagedPower));
            row.add(String.format("%.2f", currentOscillationDeg));
            row.add(lockQuality.name());
            row.add(readyToLaunch);
            turretLog.UpdateLog(row);
        }
        if (behavior != Behavior.TRACKING && logStartTime > 0) {
            turretLog.CloseLog();
            logStartTime = 0;
            turretLog = null;
        }
    }

    /**
     * Compare vision-derived bearing vs odo-derived bearing to assess odo reliability.
     * Called every loop cycle when vision is active. Updates odoTrusted flag.
     */
    private void updateOdoTrust() {
        if (driveTrain == null || vision == null || !vision.hasTarget()) return;

        // Vision says the goal is at: current turret angle minus the tx offset
        double visionBearingDeg = turretAngleDeg - vision.getTx();

        // Odo says the goal is at: computed bearing from pose
        Pose2d currentPose = driveTrain.getPose();
        Pose2d goalPose = FieldMap.getPose(FieldMap.GOAL, Robot.isRedAlliance);
        double bearingRad = FieldMap.bearingTo(currentPose, goalPose);
        double chassisRad = currentPose.heading.toDouble();
        double odoBearingDeg = normalizeDeg(Math.toDegrees(bearingRad - chassisRad));

        lastDivergenceDeg = Math.abs(normalizeDeg(visionBearingDeg - odoBearingDeg));
        lastDivergenceTimeMs = System.currentTimeMillis();
        odoTrusted = (lastDivergenceDeg < ODO_TRUST_THRESHOLD_DEG);
    }

    private double clampToLimits(double angleDeg) {
        return Math.min(Math.max(CW_LIMIT_DEG+BUFFER, angleDeg), CCW_LIMIT_DEG-BUFFER);
    }

    private static double normalizeDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg <= -180) deg += 360;
        return deg;
    }

    // ==================== LIFECYCLE ====================

    @Override
    public void stop() {
        turretMotor.setPower(0);
        behavior = Behavior.LOCKED;
        stagedPower = 0;
    }

    @Override
    public void resetStates() {
        behavior = Behavior.LOCKED;
        phase = TargetingPhase.LOCKED_FORWARD;
        stagedPower = 0;
        turretPID.enable();
    }

    // ==================== TELEMETRY ====================

    @Override
    public String getTelemetryName() {
        return "Turret";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();
        telemetry.put("Behavior", behavior);
        telemetry.put("Phase", phase);
        telemetry.put("Angle (deg)", String.format("%.1f", turretAngleDeg));
        telemetry.put("Power", String.format("%.3f", stagedPower));
        telemetry.put("Locked On", isLockedOnTarget());
        telemetry.put("Encoder", encoderTicks);
        telemetry.put("Lock Quality", lockQuality);
        telemetry.put("Oscillation (deg)", String.format("%.1f", currentOscillationDeg));
        telemetry.put("Aiming Time (ms)", String.format("%.0f", aimingTimer.milliseconds()));
        telemetry.put("Ready to Launch? ", readyToLaunch);
        telemetry.put("Degraded Ready", readyToLaunchDegraded);
        telemetry.put("Odo Trusted", odoTrusted);
        telemetry.put("Divergence (deg)", String.format("%.1f", lastDivergenceDeg));
        if (vision != null && vision.hasTarget()) {
            telemetry.put("tx", String.format("%.1f", vision.getTx()));
        }
        if (debug) {
            telemetry.put("Encoder", encoderTicks);
            telemetry.put("Lost Vision (s)", String.format("%.2f", lostVisionTimer.seconds()));
            telemetry.put("Logging", TURRET_LOGGING_ENABLED);
        }
        return telemetry;
    }
}
