package org.firstinspires.ftc.teamcode.robots.lebot2.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Lazy motor wrapper that defers power writes and caches current reads.
 *
 * This class enforces the three-phase update pattern:
 * - Phase 1 (readSensors): Call refresh() to cache motor current
 * - Phase 2 (calc): Call setPower() to queue power value, getCurrent() to read cached
 * - Phase 3 (act): Call flush() to write queued power to hardware
 *
 * IMPORTANT: Motor current reading is I2C (~7ms). This wrapper ensures
 * current is read once per cycle and cached for multiple accesses.
 *
 * Note: Motor encoder position/velocity are NOT cached here because they
 * benefit from SDK bulk caching (LynxModule.BulkCachingMode.MANUAL).
 */
public class LazyMotor {
    private final DcMotorEx motor;
    private final String name;

    // Deferred output
    private double pendingPower = 0;
    private boolean powerChanged = false;

    // Cached input (I2C read)
    private double cachedCurrent = 0;
    private boolean currentReadEnabled = false;

    /**
     * Create a LazyMotor wrapper.
     *
     * @param hardwareMap Hardware map from OpMode
     * @param deviceName  Name of the motor in robot configuration
     */
    public LazyMotor(HardwareMap hardwareMap, String deviceName) {
        this.motor = hardwareMap.get(DcMotorEx.class, deviceName);
        this.name = deviceName;
    }

    /**
     * Create a LazyMotor wrapper from an existing motor.
     *
     * @param motor Existing DcMotorEx instance
     * @param name  Name for telemetry/debugging
     */
    public LazyMotor(DcMotorEx motor, String name) {
        this.motor = motor;
        this.name = name;
    }

    // ==================== CONFIGURATION ====================

    /**
     * Enable current reading. Only enable if you actually need current monitoring,
     * as each read costs ~7ms of I2C time.
     */
    public void enableCurrentRead(boolean enabled) {
        this.currentReadEnabled = enabled;
    }

    /**
     * Set zero power behavior (BRAKE or FLOAT).
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }

    /**
     * Set motor direction.
     */
    public void setDirection(DcMotor.Direction direction) {
        motor.setDirection(direction);
    }

    /**
     * Set motor run mode.
     */
    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    // ==================== PHASE 1: READ SENSORS ====================

    /**
     * Phase 1: Refresh cached sensor values.
     * Call this once per cycle in readSensors().
     */
    public void refresh() {
        if (currentReadEnabled) {
            cachedCurrent = motor.getCurrent(CurrentUnit.AMPS);
        }
    }

    // ==================== PHASE 2: CALCULATIONS ====================

    /**
     * Phase 2: Queue a power value to be written in Phase 3.
     * Does NOT write to hardware immediately.
     *
     * @param power Motor power (-1.0 to 1.0)
     */
    public void setPower(double power) {
        pendingPower = power;
        powerChanged = true;
    }

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
     * Phase 2: Get motor encoder position.
     * This is bulk-cached by SDK, so direct read is fine.
     *
     * @return Encoder position in ticks
     */
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    /**
     * Phase 2: Get motor velocity.
     * This is bulk-cached by SDK, so direct read is fine.
     *
     * @return Velocity in ticks per second
     */
    public double getVelocity() {
        return motor.getVelocity();
    }

    /**
     * Get the pending power value (what will be written on flush).
     */
    public double getPendingPower() {
        return pendingPower;
    }

    // ==================== PHASE 3: ACT ====================

    /**
     * Phase 3: Write pending power to hardware.
     * Call this once per cycle in act().
     */
    public void flush() {
        if (powerChanged) {
            motor.setPower(pendingPower);
            powerChanged = false;
        }
    }

    // ==================== EMERGENCY ====================

    /**
     * Emergency stop - immediately writes zero power.
     * Bypasses the lazy pattern for safety.
     */
    public void stop() {
        pendingPower = 0;
        motor.setPower(0);
        powerChanged = false;
    }

    // ==================== ACCESS ====================

    /**
     * Get the underlying motor for advanced operations.
     * Use sparingly - prefer the lazy methods.
     */
    public DcMotorEx getMotor() {
        return motor;
    }

    /**
     * Get the motor name.
     */
    public String getName() {
        return name;
    }
}
