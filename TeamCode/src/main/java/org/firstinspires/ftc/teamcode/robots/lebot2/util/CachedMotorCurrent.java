package org.firstinspires.ftc.teamcode.robots.lebot2.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Read-only cached motor current wrapper for stall detection.
 *
 * This class is for monitoring motors owned by other systems (like RoadRunner).
 * It provides current monitoring without owning motor control.
 *
 * This class enforces the three-phase update pattern:
 * - Phase 1 (readSensors): Call refresh() to read current and cache value
 * - Phase 2 (calc): Call getCurrent() to get cached value
 * - Phase 3 (act): No action needed (read-only)
 *
 * IMPORTANT: Motor current reading is I2C (~7ms per read).
 * This wrapper ensures current is read once per cycle.
 *
 * Use case: Robot wants to monitor drive motor current for stall detection,
 * but the motors are owned/controlled by RoadRunner.
 */
public class CachedMotorCurrent {
    private final DcMotorEx motor;
    private final String name;

    // Cached value
    private double cachedCurrent = 0;

    // Stall detection
    private double stallThresholdAmps = 5.0;
    private long stallStartTime = 0;
    private boolean isStalled = false;

    /**
     * Create a CachedMotorCurrent wrapper.
     *
     * @param hardwareMap Hardware map from OpMode
     * @param deviceName  Name of the motor in robot configuration
     */
    public CachedMotorCurrent(HardwareMap hardwareMap, String deviceName) {
        this.motor = hardwareMap.get(DcMotorEx.class, deviceName);
        this.name = deviceName;
    }

    /**
     * Create a CachedMotorCurrent wrapper from an existing motor.
     *
     * @param motor Existing DcMotorEx instance
     * @param name  Name for telemetry/debugging
     */
    public CachedMotorCurrent(DcMotorEx motor, String name) {
        this.motor = motor;
        this.name = name;
    }

    // ==================== CONFIGURATION ====================

    /**
     * Set the stall detection threshold.
     *
     * @param amps Current threshold in amps to consider a stall
     */
    public void setStallThreshold(double amps) {
        this.stallThresholdAmps = amps;
    }

    // ==================== PHASE 1: READ SENSORS ====================

    /**
     * Phase 1: Read current and cache value.
     * Call this once per cycle in readSensors().
     */
    public void refresh() {
        cachedCurrent = motor.getCurrent(CurrentUnit.AMPS);

        // Update stall detection
        if (cachedCurrent > stallThresholdAmps) {
            if (stallStartTime == 0) {
                stallStartTime = System.currentTimeMillis();
            }
        } else {
            stallStartTime = 0;
            isStalled = false;
        }
    }

    // ==================== PHASE 2: CALCULATIONS ====================

    /**
     * Phase 2: Get cached motor current.
     * Returns value from last refresh() call.
     *
     * @return Motor current in amps
     */
    public double getCurrent() {
        return cachedCurrent;
    }

    /**
     * Check if current exceeds stall threshold.
     *
     * @return true if current is above stall threshold
     */
    public boolean isOverCurrent() {
        return cachedCurrent > stallThresholdAmps;
    }

    /**
     * Check if motor has been stalled for a duration.
     *
     * @param durationMs Minimum duration in milliseconds to consider stalled
     * @return true if motor has been over current for at least durationMs
     */
    public boolean isStalledFor(long durationMs) {
        if (stallStartTime == 0) {
            return false;
        }
        return (System.currentTimeMillis() - stallStartTime) >= durationMs;
    }

    // ==================== ACCESS ====================

    /**
     * Get the motor name.
     */
    public String getName() {
        return name;
    }

    /**
     * Get the stall threshold.
     */
    public double getStallThreshold() {
        return stallThresholdAmps;
    }
}
