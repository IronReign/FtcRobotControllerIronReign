package org.firstinspires.ftc.teamcode.robots.csbot.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public interface DcMotorExSquared extends DcMotorEx {

/*
This extension to DcMotorEx adds the ability to reset the encoder to any arbitrary value
*/

/**
 * The DcMotorEx interface provides enhanced motor functionality which is available with some
 * hardware devices. The DcMotorEx interface is typically used as a second interface on an object
 * whose primary interface is DcMotor. To access it, cast your DcMotor object
 * to DcMotorEx. However, it is perhaps prudent to first test whether the cast will succeed by
 * testing using 'instanceof'.
 * @see PwmControl
 */
    /**
     * Sets the current ticks of the motor by maintaining an offset
     * between the lynx and the clients of this class
     */
    void resetPosition(int position);
    /**
     * Individually energizes this particular motor
     * @see #setMotorDisable()
     * @see #isMotorEnabled()
     */
    void setMotorEnable();

    /**
     * Individually de-energizes this particular motor
     * @see #setMotorEnable()
     * @see #isMotorEnabled()
     */
    void setMotorDisable();

    /**
     * Returns whether this motor is energized
     * @see #setMotorEnable()
     * @see #setMotorDisable()
     */
    boolean isMotorEnabled();

    /**
     * Sets the velocity of the motor
     * @param angularRate  the desired ticks per second
     */
    void setVelocity(double angularRate);

    /**
     * Sets the velocity of the motor
     * @param angularRate   the desired angular rate, in units per second
     * @param unit          the units in which angularRate is expressed
     *
     * @see #getVelocity(AngleUnit)
     */
    void setVelocity(double angularRate, AngleUnit unit);

    /**
     * Returns the current velocity of the motor, in ticks per second
     * @return the current velocity of the motor
     */
    double getVelocity();

    /**
     * Returns the current velocity of the motor, in angular units per second
     * @param unit          the units in which the angular rate is desired
     * @return              the current velocity of the motor
     *
     * @see #setVelocity(double, AngleUnit)
     */
    double getVelocity(AngleUnit unit);

    /**
     * Sets the PID control coefficients for one of the PID modes of this motor.
     * Note that in some controller implementations, setting the PID coefficients for one
     * mode on a motor might affect other modes on that motor, or might affect the PID
     * coefficients used by other motors on the same controller (this is not true on the
     * REV Expansion Hub).
     *
     * @param mode either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @param pidCoefficients the new coefficients to use when in that mode on this motor
     *
     * @see #getPIDCoefficients(RunMode)
     *
     * @deprecated Use {@link #setPIDFCoefficients(RunMode, PIDFCoefficients)} instead
     */
    @Deprecated
    void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients);

    /**
     * {@link #setPIDFCoefficients} is a superset enhancement to {@link #setPIDCoefficients}. In addition
     * to the proportional, integral, and derivative coefficients previously supported, a feed-forward
     * coefficient may also be specified. Further, a selection of motor control algorithms is offered:
     * the originally-shipped Legacy PID algorithm, and a PIDF algorithm which avails itself of the
     * feed-forward coefficient. Note that the feed-forward coefficient is not used by the Legacy PID
     * algorithm; thus, the feed-forward coefficient must be indicated as zero if the Legacy PID
     * algorithm is used. Also: the internal implementation of these algorithms may be different: it
     * is not the case that the use of PIDF with the F term as zero necessarily exhibits exactly the
     * same behavior as the use of the LegacyPID algorithm, though in practice they will be quite close.
     *
     * Readers are reminded that {@link RunMode#RUN_TO_POSITION} mode makes use of <em>both</em>
     * the coefficients set for RUN_TO_POSITION <em>and</em> the coefficients set for RUN_WITH_ENCODER,
     * due to the fact that internally the RUN_TO_POSITION logic calculates an on-the-fly velocity goal
     * on each control cycle, then (logically) runs the RUN_WITH_ENCODER logic. Because of that double-
     * layering, only the proportional ('p') coefficient makes logical sense for use in the RUN_TO_POSITION
     * coefficients.
     *
     * @see #setVelocityPIDFCoefficients(double, double, double, double)
     * @see #setPositionPIDFCoefficients(double)
     * @see #getPIDFCoefficients(RunMode)
     */
    void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException;

    /**
     * A shorthand for setting the PIDF coefficients for the {@link RunMode#RUN_USING_ENCODER}
     * mode. {@link MotorControlAlgorithm#PIDF} is used.
     *
     * @see #setPIDFCoefficients(RunMode, PIDFCoefficients)
     */
    void setVelocityPIDFCoefficients(double p, double i, double d, double f);

    /**
     * A shorthand for setting the PIDF coefficients for the {@link RunMode#RUN_TO_POSITION}
     * mode. {@link MotorControlAlgorithm#PIDF} is used.
     *
     * Readers are reminded that {@link RunMode#RUN_TO_POSITION} mode makes use of <em>both</em>
     * the coefficients set for RUN_TO_POSITION <em>and</em> the coefficients set for RUN_WITH_ENCODER,
     * due to the fact that internally the RUN_TO_POSITION logic calculates an on-the-fly velocity goal
     * on each control cycle, then (logically) runs the RUN_WITH_ENCODER logic. Because of that double-
     * layering, only the proportional ('p') coefficient makes logical sense for use in the RUN_TO_POSITION
     * coefficients.
     *
     * @see #setVelocityPIDFCoefficients(double, double, double, double)
     * @see #setPIDFCoefficients(RunMode, PIDFCoefficients)
     */
    void setPositionPIDFCoefficients(double p);

    /**
     * Returns the PID control coefficients used when running in the indicated mode
     * on this motor.
     * @param mode either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @return the PID control coefficients used when running in the indicated mode on this motor
     *
     * @deprecated Use {@link #getPIDFCoefficients(RunMode)} instead
     */
    @Deprecated
    PIDCoefficients getPIDCoefficients(RunMode mode);

    /**
     * Returns the PIDF control coefficients used when running in the indicated mode
     * on this motor.
     * @param mode either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @return the PIDF control coefficients used when running in the indicated mode on this motor
     *
     * @see #setPIDFCoefficients(RunMode, PIDFCoefficients)
     */
    PIDFCoefficients getPIDFCoefficients(RunMode mode);

    /**
     * Sets the target positioning tolerance of this motor
     * @param tolerance the desired tolerance, in encoder ticks
     * @see DcMotor#setTargetPosition(int)
     */
    void setTargetPositionTolerance(int tolerance);

    /**
     * Returns the current target positioning tolerance of this motor
     * @return the current target positioning tolerance of this motor
     */
    int getTargetPositionTolerance();

    /**
     * Returns the current consumed by this motor.
     * @param unit current units
     * @return the current consumed by this motor.
     */
    double getCurrent(CurrentUnit unit);

    /**
     * Returns the current alert for this motor.
     * @param unit current units
     * @return the current alert for this motor
     */
    double getCurrentAlert(CurrentUnit unit);

    /**
     * Sets the current alert for this motor
     * @param current current alert
     * @param unit current units
     */
    void setCurrentAlert(double current, CurrentUnit unit);

    /**
     * Returns whether the current consumption of this motor exceeds the alert threshold.
     * @return whether the current consumption of this motor exceeds the alert threshold.
     */
    boolean isOverCurrent();
}
