package org.firstinspires.ftc.teamcode.robots.lebot2.util;

import java.util.Map;

/**
 * Interface for classes that provide telemetry data.
 *
 * Subsystems implement this to expose their internal state for debugging
 * and monitoring. The OpMode collects telemetry from all providers and
 * sends it to the Driver Station and FTC Dashboard.
 *
 * IMPORTANT: getTelemetry() should ONLY collect data, never transmit it.
 * Transmission is the OpMode's responsibility.
 */
public interface TelemetryProvider {
    /**
     * Get telemetry data from this provider.
     *
     * @param debug If true, include verbose debugging information.
     *              If false, include only essential operational data.
     * @return Map of telemetry key-value pairs
     */
    Map<String, Object> getTelemetry(boolean debug);

    /**
     * Get the display name for this telemetry section.
     *
     * @return Human-readable name shown as section header
     */
    String getTelemetryName();
}
