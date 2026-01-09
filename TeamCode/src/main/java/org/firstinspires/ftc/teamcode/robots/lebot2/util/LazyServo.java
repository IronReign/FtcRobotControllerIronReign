package org.firstinspires.ftc.teamcode.robots.lebot2.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Lazy servo wrapper that defers position writes.
 *
 * This class enforces the three-phase update pattern:
 * - Phase 1 (readSensors): No action needed (servos have no readable state)
 * - Phase 2 (calc): Call setPosition() to queue position value
 * - Phase 3 (act): Call flush() to write queued position to hardware
 *
 * Unlike motors, servos don't have I2C sensor reads, but we still defer
 * writes to maintain consistent timing and allow position changes to be
 * computed before being applied.
 */
public class LazyServo {
    private final Servo servo;
    private final String name;

    // Deferred output
    private double pendingPosition = 0.5;
    private boolean positionChanged = false;

    /**
     * Create a LazyServo wrapper.
     *
     * @param hardwareMap Hardware map from OpMode
     * @param deviceName  Name of the servo in robot configuration
     */
    public LazyServo(HardwareMap hardwareMap, String deviceName) {
        this.servo = hardwareMap.get(Servo.class, deviceName);
        this.name = deviceName;
    }

    /**
     * Create a LazyServo wrapper from an existing servo.
     *
     * @param servo Existing Servo instance
     * @param name  Name for telemetry/debugging
     */
    public LazyServo(Servo servo, String name) {
        this.servo = servo;
        this.name = name;
    }

    // ==================== CONFIGURATION ====================

    /**
     * Set servo direction.
     */
    public void setDirection(Servo.Direction direction) {
        servo.setDirection(direction);
    }

    /**
     * Scale the servo range.
     *
     * @param min Minimum position (0.0 to 1.0)
     * @param max Maximum position (0.0 to 1.0)
     */
    public void scaleRange(double min, double max) {
        servo.scaleRange(min, max);
    }

    // ==================== PHASE 1: READ SENSORS ====================

    /**
     * Phase 1: No action needed for servos.
     * Included for interface consistency.
     */
    public void refresh() {
        // Servos have no readable state
    }

    // ==================== PHASE 2: CALCULATIONS ====================

    /**
     * Phase 2: Queue a position value to be written in Phase 3.
     * Does NOT write to hardware immediately.
     *
     * @param position Servo position (0.0 to 1.0)
     */
    public void setPosition(double position) {
        pendingPosition = position;
        positionChanged = true;
    }

    /**
     * Get the pending position value (what will be written on flush).
     */
    public double getPendingPosition() {
        return pendingPosition;
    }

    // ==================== PHASE 3: ACT ====================

    /**
     * Phase 3: Write pending position to hardware.
     * Call this once per cycle in act().
     */
    public void flush() {
        if (positionChanged) {
            servo.setPosition(pendingPosition);
            positionChanged = false;
        }
    }

    // ==================== ACCESS ====================

    /**
     * Get the underlying servo for advanced operations.
     * Use sparingly - prefer the lazy methods.
     */
    public Servo getServo() {
        return servo;
    }

    /**
     * Get the servo name.
     */
    public String getName() {
        return name;
    }
}
