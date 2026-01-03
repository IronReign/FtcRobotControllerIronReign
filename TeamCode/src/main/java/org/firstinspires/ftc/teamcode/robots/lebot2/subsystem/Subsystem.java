package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.TelemetryProvider;

/**
 * Base interface for all robot subsystems.
 *
 * A subsystem is an independent unit that manages a specific part of the robot.
 * Each subsystem:
 * - Owns specific hardware (motors, servos, sensors)
 * - Has its own state machine (if needed)
 * - Provides telemetry for debugging
 * - Updates every loop cycle
 *
 * The Robot class coordinates subsystems and handles cross-subsystem behaviors.
 */
public interface Subsystem extends TelemetryProvider {

    /**
     * Called every loop cycle to update the subsystem's state.
     *
     * This method should:
     * 1. Read sensor values (or use cached values from bulk read)
     * 2. Process state machine logic
     * 3. Set motor/servo outputs
     *
     * IMPORTANT: This method must be NON-BLOCKING. Never use Thread.sleep()
     * or while loops that wait for conditions.
     *
     * @param fieldOverlay Canvas for drawing on FTC Dashboard field view
     */
    void update(Canvas fieldOverlay);

    /**
     * Called when the OpMode stops.
     *
     * Use this to:
     * - Set motors to zero power
     * - Move servos to safe positions
     * - Clean up any resources
     */
    void stop();

    /**
     * Reset all state machines to their initial states.
     *
     * Called when transitioning between autonomous and teleop,
     * or when the driver wants to cancel an in-progress action.
     */
    void resetStates();
}
