package org.firstinspires.ftc.teamcode.robots.lebot2.util;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Cached distance sensor wrapper for I2C sensors like Rev 2m Distance Sensor.
 *
 * This class enforces the three-phase update pattern:
 * - Phase 1 (readSensors): Call refresh() to read sensor and cache value
 * - Phase 2 (calc): Call getDistance() to get cached value
 * - Phase 3 (act): No action needed (read-only sensor)
 *
 * IMPORTANT: Distance sensor reads are I2C operations (~7ms each).
 * This wrapper ensures the sensor is read once per cycle and cached
 * for multiple accesses within that cycle.
 */
public class CachedDistanceSensor {
    private final DistanceSensor sensor;
    private final String name;
    private final DistanceUnit defaultUnit;

    // Cached value
    private double cachedDistance = 0;

    /**
     * Create a CachedDistanceSensor wrapper.
     *
     * @param hardwareMap Hardware map from OpMode
     * @param deviceName  Name of the sensor in robot configuration
     * @param unit        Default distance unit for reads
     */
    public CachedDistanceSensor(HardwareMap hardwareMap, String deviceName, DistanceUnit unit) {
        this.sensor = hardwareMap.get(DistanceSensor.class, deviceName);
        this.name = deviceName;
        this.defaultUnit = unit;
    }

    /**
     * Create a CachedDistanceSensor wrapper with CM as default unit.
     *
     * @param hardwareMap Hardware map from OpMode
     * @param deviceName  Name of the sensor in robot configuration
     */
    public CachedDistanceSensor(HardwareMap hardwareMap, String deviceName) {
        this(hardwareMap, deviceName, DistanceUnit.CM);
    }

    /**
     * Create a CachedDistanceSensor wrapper from an existing sensor.
     *
     * @param sensor Existing DistanceSensor instance
     * @param name   Name for telemetry/debugging
     * @param unit   Default distance unit for reads
     */
    public CachedDistanceSensor(DistanceSensor sensor, String name, DistanceUnit unit) {
        this.sensor = sensor;
        this.name = name;
        this.defaultUnit = unit;
    }

    // ==================== PHASE 1: READ SENSORS ====================

    /**
     * Phase 1: Read sensor and cache value.
     * Call this once per cycle in readSensors().
     */
    public void refresh() {
        cachedDistance = sensor.getDistance(defaultUnit);
    }

    // ==================== PHASE 2: CALCULATIONS ====================

    /**
     * Phase 2: Get cached distance value in default unit.
     * Returns value from last refresh() call.
     *
     * @return Distance in default unit
     */
    public double getDistance() {
        return cachedDistance;
    }

    /**
     * Phase 2: Get cached distance value converted to specified unit.
     * Returns value from last refresh() call, converted.
     *
     * @param targetUnit Unit to convert to
     * @return Distance in target unit
     */
    public double getDistance(DistanceUnit targetUnit) {
        return targetUnit.fromUnit(defaultUnit, cachedDistance);
    }

    /**
     * Check if a distance reading is valid (not infinity/out of range).
     *
     * @return true if the reading is valid
     */
    public boolean isValid() {
        return !Double.isInfinite(cachedDistance) && !Double.isNaN(cachedDistance);
    }

    // ==================== ACCESS ====================

    /**
     * Get the underlying sensor for advanced operations.
     * Use sparingly - prefer the cached methods.
     */
    public DistanceSensor getSensor() {
        return sensor;
    }

    /**
     * Get the sensor name.
     */
    public String getName() {
        return name;
    }

    /**
     * Get the default distance unit.
     */
    public DistanceUnit getDefaultUnit() {
        return defaultUnit;
    }
}
