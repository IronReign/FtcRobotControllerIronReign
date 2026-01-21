package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.drivetrain;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Vision;

/**
 * Interface for swappable drivetrain implementations.
 *
 * This interface allows the robot to use different chassis types
 * (tank/differential, mecanum, swerve) with the same control code.
 * Each implementation handles the specifics of its drive type.
 *
 * All implementations should use the Pinpoint localizer for consistent
 * odometry across chassis types.
 */
public interface DriveTrainBase extends Subsystem {

    /**
     * Drive the robot with the given inputs.
     *
     * @param throttle Forward/backward power (-1 to 1). Positive = forward.
     * @param strafe   Left/right power (-1 to 1). Positive = right.
     *                 Ignored by differential drives.
     * @param turn     Rotational power (-1 to 1). Positive = clockwise.
     */
    void drive(double throttle, double strafe, double turn);

    //void setMotorPowers(double left, double right);

    /**
     * Turn to a specific heading using the IMU.
     *
     * This is a non-blocking call. Check isTurnComplete() to know when done.
     *
     * @param headingDegrees Target heading in degrees (0-360)
     * @param maxSpeed       Maximum turn speed (0 to 1)
     */
    void turnToHeading(double headingDegrees, double maxSpeed);

    /**
     * Turn to center on a target using vision (tx from Limelight).
     *
     * This is a non-blocking call. Check isTurnComplete() to know when done.
     * NOTE: This uses a single tx value. For continuous tracking, use centerOnTarget().
     *
     * @param tx       Target's horizontal offset from center (degrees)
     * @param maxSpeed Maximum turn speed (0 to 1)
     */
    void turnToTarget(double tx, double maxSpeed);

    /**
     * Center on target using Vision with continuous tx updates.
     *
     * This is a non-blocking call that queries Vision directly each loop
     * for fresh tx values. Runs to completion (tx near 0) unless:
     * - Joystick input interrupts
     * - Target is lost
     * - cancelTurn() is called
     *
     * Requires setVision() to be called first.
     */
    void centerOnTarget();

    /**
     * Set the Vision reference for continuous target tracking.
     *
     * @param vision Vision subsystem reference
     */
    void setVision(Vision vision);

    /**
     * Check if the current turn operation is complete.
     *
     * @return true if no turn is in progress or turn has reached target
     */
    boolean isTurnComplete();

    /**
     * Cancel any in-progress turn operation.
     */
    void cancelTurn();

    /**
     * Get the robot's current pose from the localizer.
     *
     * @return Current pose (x, y in inches, heading in radians)
     */
    Pose2d getPose();

    /**
     * Set the robot's pose (for resetting position).
     *
     * @param pose New pose to set
     */
    void setPose(Pose2d pose);

    /**
     * Set the robot's pose from a starting position constant.
     *
     * @param position Starting position enum
     */
    void setPose(Object position); // Will use Constants.Position

    /**
     * Get the current heading from the IMU.
     *
     * @return Heading in degrees (0-360)
     */
    double getHeadingDegrees();

    /**
     * Reset the drive encoders.
     */
    void resetEncoders();

    int getLeftTicks();
}
