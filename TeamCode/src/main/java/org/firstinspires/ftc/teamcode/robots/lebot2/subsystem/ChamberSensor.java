package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.CachedDistanceSensor;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Virtual ball chamber sensor — fuses three distance sensors into a ball count.
 *
 * SENSOR LAYOUT (from rear to front of channel):
 *   Rear sensor: looks through channel toward the gate
 *     > REAR_EMPTY_CM:          no balls in positions 1 or 2 (sees through gate)
 *     REAR_BALL1_CM..REAR_EMPTY_CM: 1st ball detected (nearest launch position)
 *     < REAR_BALL1_CM:          2nd ball detected (1st ball + gate block the view)
 *
 *   Mid sensor: detects ball in 3rd (fully loaded) position
 *     < MID_BALL3_CM:           3rd ball present
 *
 *   Front sensor: detects trapped 4th ball (overfull)
 *     < FRONT_OVERFULL_CM:      4th ball trapped — needs brief eject
 *
 * USAGE:
 *   Loader owns this object and calls:
 *     refresh()  — in readSensors() phase (triggers I2C reads)
 *     update()   — in calc() phase (processes distances into ball count)
 *   Then queries:
 *     getBallCount(), isFull(), isEmpty(), isOverfull()
 *
 *   Launcher can also hold a reference to read getBallCount() for
 *   dynamic BALL_EXIT_EXPECTED_COUNT.
 */
@Config(value = "Lebot2_ChamberSensor")
public class ChamberSensor {

    // ==================== THRESHOLDS (Dashboard tunable) ====================

    // Rear sensor: mounted on side of channel (cm)
    // Discrete ranges with dead zones between them
    public static double REAR_EMPTY_CM = 20;       // > this = no balls
    public static double REAR_BALL1_MAX_CM = 20;   // 16–20 = 1st ball
    public static double REAR_BALL1_MIN_CM = 16;
    public static double REAR_BALL2_MAX_CM = 13.5;  // 2–13.5 = 2nd ball
    public static double REAR_BALL2_MIN_CM = 2;

    // Mid sensor: 3rd ball position (cm)
    public static double MID_BALL3_CM = 15;         // < this = 3rd ball present

    // Front sensor: 4th ball trap detection (cm)
    public static double FRONT_OVERFULL_CM = 13;    // < this = 4th ball trapped

    // Overfull debounce: a ball in transit passes the front sensor briefly (~100ms).
    // A genuine 4th ball stays. Require front sensor held for this long AND 3 balls
    // already loaded before reporting overfull.
    public static double OVERFULL_DEBOUNCE_MS = 150;

    // ==================== HARDWARE ====================

    private final CachedDistanceSensor rearSensor;
    private final CachedDistanceSensor midSensor;
    private final CachedDistanceSensor frontSensor;

    // ==================== STATE ====================

    private double rearDistance = 100;
    private double midDistance = 100;
    private double frontDistance = 100;

    private int ballCount = 0;
    private int previousRearBalls = 0;  // Retained across dead zones
    private boolean overFull = false;
    private long overfullStartMs = 0;   // when raw overfull condition first detected

    // ==================== CONSTRUCTOR ====================

    public ChamberSensor(HardwareMap hardwareMap) {
        rearSensor = new CachedDistanceSensor(hardwareMap, "backDist", DistanceUnit.CM);
        midSensor = new CachedDistanceSensor(hardwareMap, "midDist", DistanceUnit.CM);
        frontSensor = new CachedDistanceSensor(hardwareMap, "frontDist", DistanceUnit.CM);
    }

    // ==================== THREE-PHASE METHODS ====================

    /**
     * Phase 1: Trigger I2C reads on all three sensors.
     * Call from Loader.readSensors().
     */
    public void refresh() {
        rearSensor.refresh();
        midSensor.refresh();
        frontSensor.refresh();
    }

    /**
     * Phase 2: Process cached distances into ball count.
     * Call from Loader.calc().
     */
    public void update() {
        rearDistance = rearSensor.getDistance();
        midDistance = midSensor.getDistance();
        frontDistance = frontSensor.getDistance();

        // Rear sensor: discrete ranges with dead zones
        // Readings in dead zones retain the previous count (no change)
        int rearBalls = previousRearBalls;
        if (rearDistance > REAR_EMPTY_CM) {
            rearBalls = 0;  // empty
        } else if (rearDistance <= REAR_BALL1_MAX_CM && rearDistance >= REAR_BALL1_MIN_CM) {
            rearBalls = 1;  // 1st ball
        } else if (rearDistance <= REAR_BALL2_MAX_CM && rearDistance >= REAR_BALL2_MIN_CM) {
            rearBalls = 2;  // 2nd ball
        }
        previousRearBalls = rearBalls;

        // Mid sensor: position 3
        boolean ball3 = midDistance < MID_BALL3_CM;

        int rawCount = rearBalls + (ball3 ? 1 : 0);

        // Rate-limit: ballCount can only change by ±1 per cycle.
        // Balls enter one at a time through the channel — a jump of 2+ in one cycle
        // is a transit artifact (e.g., a ball temporarily reading as "2 balls" at the
        // rear sensor while another passes through mid).
        if (rawCount > ballCount) {
            ballCount = Math.min(rawCount, ballCount + 1);
        } else if (rawCount < ballCount) {
            ballCount = Math.max(rawCount, ballCount - 1);
        }
        // Note: 4th ball is tracked separately — ballCount reflects launchable balls (0-3)

        // Front sensor: position 4 (overfull)
        // State guard: only a genuine 4th ball if we already have 3 loaded.
        // Without this, every incoming ball triggers overfull as it passes the front sensor.
        // Debounce guard: ball in transit passes in ~100ms, genuine 4th ball stays.
        boolean rawOverfull = frontDistance < FRONT_OVERFULL_CM && ballCount >= 3;
        if (rawOverfull) {
            if (overfullStartMs == 0) {
                overfullStartMs = System.currentTimeMillis();
            }
            overFull = (System.currentTimeMillis() - overfullStartMs) >= OVERFULL_DEBOUNCE_MS;
        } else {
            overfullStartMs = 0;
            overFull = false;
        }
    }

    // ==================== QUERIES ====================

    /** Number of launchable balls in the chamber (0-3). */
    public int getBallCount() {
        return ballCount;
    }

    /** Chamber has 3 balls — ready to fire. */
    public boolean isFull() {
        return ballCount >= 3;
    }

    /** Chamber has no balls. */
    public boolean isEmpty() {
        return ballCount == 0;
    }

    /** 4th ball is trapped at the front — needs brief eject. */
    public boolean isOverfull() {
        return overFull;
    }

    /** At least one ball is in a launch position (rear sensor sees something). */
    public boolean hasBallsAtRear() {
        return rearDistance <= REAR_EMPTY_CM;
    }

    // ==================== RAW DISTANCES (for debugging) ====================

    public double getRearDistance() { return rearDistance; }
    public double getMidDistance() { return midDistance; }
    public double getFrontDistance() { return frontDistance; }

    // ==================== TELEMETRY ====================

    public Map<String, Object> getTelemetry() {
        Map<String, Object> t = new LinkedHashMap<>();
        t.put("Ball Count", ballCount);
        t.put("Overfull", overFull ? "YES — EJECT" : "no");
        t.put("Rear", String.format("%.1f cm", rearDistance));
        t.put("Mid", String.format("%.1f cm", midDistance));
        t.put("Front", String.format("%.1f cm", frontDistance));
        return t;
    }
}
