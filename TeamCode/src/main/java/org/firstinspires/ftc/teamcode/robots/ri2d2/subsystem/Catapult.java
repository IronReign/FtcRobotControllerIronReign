package org.firstinspires.ftc.teamcode.robots.ri2d2.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Elastic catapult - launches all 4 balls at once out of the wide bucket.
 *
 * HARDWARE:
 * - "catapult"        DcMotorEx with encoder, spool winding a string on the boom
 * - "catapultRelease" Servo, 2 positions only: CAPTURED (holds boom down) and RELEASED
 *
 * ENCODER CONVENTION (tune on the robot):
 * - Zero the encoder with the boom resting UP (launched, string just barely taut).
 * - Winding the spool IN pulls the boom DOWN, counts go MORE POSITIVE.
 * - RETRACT_TICKS is boom fully cocked; SLACK_TICKS pays string back out so the
 *   string cannot drag on the boom during a launch. So SLACK_TICKS < RETRACT_TICKS.
 * - If the motor drives the wrong way, flip MOTOR_REVERSED.
 *
 * LATCH GEOMETRY (this drives the whole ordering):
 * The latch is pulled DOWN onto the boom pin by a weak rubber band. The servo's
 * job is to hold it UP out of the way, not to push it down.
 * - RELEASE_RELEASED = servo holding the latch OPEN (up, clear of the pin)
 * - RELEASE_CAPTURED = servo out of the way, band pulls the latch down on the pin
 * The latch MUST stay open until the pin is all the way down past it. Let it
 * down early and it misses the pin and the mechanism jams - which is why
 * RETRACTING deliberately overshoots the pin point and only then lets go, and
 * why a retract timeout leaves the latch OPEN rather than dropping it blind.
 * The latch is not actually secure when it first drops; only the weak band holds
 * it. SLACKENING lets the boom rise onto it, and that tension is what seats it.
 *
 * SEQUENCE: one press of retract() runs the whole arming sequence -
 *   RETRACTING -> CAPTURING -> SLACKENING -> READY
 * and stops there. READY is the normal state to drive around in: the winch is
 * unpowered and the cord slack is taken up by rubber bands, so accelerations
 * and turns won't despool it. launch() then just opens the latch.
 *
 * THE TWO THINGS THAT CAN HOLD A COCKED BOOM DOWN:
 * the winch (string under tension) or the latch. Exactly one of them is always
 * responsible, and the handoff between them is the dangerous part:
 * - The winch does NOT let go until the latch has been commanded CAPTURED and
 *   given CAPTURE_DWELL_MS to physically get there. A servo has no position
 *   feedback, so time is the only confirmation we have. Once that dwell is up
 *   the latch owns the boom and the winch has no further reason to hold it.
 * - If anything goes wrong with a cocked, unlatched boom (abort, jog released,
 *   retract timeout) we go to HOLDING and keep the winch powered rather than
 *   letting the elastic snap the boom back up.
 *
 * SAFETY: the release servo will not open unless the spool has paid out to the
 * slack position. launch() from any other state just slackens first, then fires.
 */
@Config(value = "newBot_Catapult")
public class Catapult implements Subsystem {

    // Spool positions, in encoder ticks
    public static int RETRACT_TICKS = 1500;   // Pin is far enough down for the latch to drop behind it
    public static int SLACK_TICKS = 900;      // String paid back out, safe to fire
    public static int POSITION_TOLERANCE = 25;

    /**
     * How far PAST RETRACT_TICKS the winch pulls before dropping the latch.
     * The latch must never come down short of the pin, so we deliberately
     * overshoot and hold there through the capture dwell. Overshooting is
     * harmless; coming up short misses the pin and jams the mechanism.
     */
    public static int RETRACT_OVERSHOOT_TICKS = 40;

    // Spool powers
    public static double RETRACT_POWER = 0.8;
    public static double SLACK_POWER = 0.5;
    public static double JOG_POWER = 0.3;     // Manual spool jog during setup

    // Release servo, 2 positions
    public static double RELEASE_CAPTURED = 0.25;
    public static double RELEASE_RELEASED = 0.70;

    /**
     * How long the winch keeps holding the boom past the pin after releasing the
     * latch, before it is allowed to slacken. Must cover the servo's travel AND
     * the weak rubber band's pull-down - measure it and add margin. Neither the
     * servo nor the band gives us any feedback, so time is all we have.
     */
    public static long CAPTURE_DWELL_MS = 500;

    // Timeouts so a stall or a bad tuning value can't wedge the state machine
    public static long RETRACT_TIMEOUT_MS = 4000;
    public static long SLACK_TIMEOUT_MS = 2000;
    public static long LAUNCH_DWELL_MS = 400;  // Time for the boom to clear before we call it done

    public static boolean MOTOR_REVERSED = false;

    public enum CatapultState {
        IDLE,        // Motor off, boom is not under tension - safe parking state
        RETRACTING,  // Winding the boom down, winch is holding it
        CAPTURING,   // Boom down, latch commanded closed, winch STILL holding
        SLACKENING,  // Latch confirmed - paying string back out, boom on the latch
        READY,       // Slack confirmed - armed, safe to drive, launch is allowed
        LAUNCHING,   // Release open, boom flying
        FIRED,       // Launch complete, waiting for the next retract()
        HOLDING,     // Something went wrong with a cocked boom - winch holds position
        MANUAL       // Driver is jogging the spool by hand
    }

    private final DcMotorEx winch;
    private final Servo release;

    private CatapultState state = CatapultState.IDLE;
    private long stateStartMs = 0;

    // Queued hardware commands, flushed in act()
    private int targetTicks = 0;
    private double motorPower = 0;
    private boolean positionMode = true;      // false = open-loop jog
    private double releaseTarget = RELEASE_CAPTURED;

    // Cached sensor values, read in readSensors()
    private int currentTicks = 0;

    private DcMotor.RunMode lastMode = null;
    private boolean launchRequested = false;
    private boolean latched = false;          // Latch commanded closed AND given time to get there
    private int holdTicks = 0;                // Position HOLDING parks at

    public Catapult(HardwareMap hardwareMap) {
        winch = hardwareMap.get(DcMotorEx.class, "catapult");
        release = hardwareMap.get(Servo.class, "catapultRelease");

        winch.setDirection(MOTOR_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        // FLOAT, not BRAKE: the only states that run the winch at zero power are the
        // ones where the boom is launching or already up, and a braked spool would
        // fight the string paying out mid-throw. Every state that has to hold the
        // boom does it with real power under RUN_TO_POSITION, not with brake.
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        zeroEncoder();

        // CAPTURED at init, deliberately. If the robot was powered down while armed
        // the latch is already down and this leaves it there; commanding RELEASED
        // would lift the latch off a loaded pin and fire the catapult during init.
        // With the boom up it just rests the latch down, and retract() lifts it.
        releaseTarget = RELEASE_CAPTURED;
        release.setPosition(releaseTarget);
    }

    // ==================== COMMANDS ====================

    /**
     * Run the whole arming sequence: wind the boom down, latch it, slacken the
     * string, and stop at READY.
     *
     * Ignored when already armed. Retracting starts by lifting the latch, and
     * lifting the latch on an armed boom IS a launch - so this must never be
     * the thing that fires the catapult.
     */
    public void retract() {
        if (isArmed()) {
            return;
        }
        launchRequested = false;
        setState(CatapultState.RETRACTING);
    }

    /**
     * Pay the string back out to the slack position, leaving the boom on the latch.
     * If the boom is cocked but not confirmed latched this captures it first - we
     * never hand the boom over to a latch that might not be closed.
     */
    public void slacken() {
        if (isStringTaut() && !latched) {
            setState(CatapultState.CAPTURING);
        } else {
            setState(CatapultState.SLACKENING);
        }
    }

    /**
     * Fire. If the string is not already slack this slackens first and fires
     * automatically once slack is confirmed - the release never opens taut.
     */
    public void launch() {
        if (state == CatapultState.LAUNCHING || state == CatapultState.FIRED) {
            return;
        }
        launchRequested = true;
        if (state == CatapultState.READY) {
            setState(CatapultState.LAUNCHING);
        } else {
            slacken();  // Routes through CAPTURING if the latch isn't confirmed yet
        }
    }

    /**
     * Cancel whatever is running. If the boom is cocked and unlatched the winch
     * keeps holding it - aborting must never mean dropping the boom.
     */
    public void abort() {
        launchRequested = false;
        if (isStringTaut() && !latched) {
            hold();
        } else {
            setState(CatapultState.IDLE);
        }
    }

    /**
     * Manually jog the spool. Positive winds in (boom down), negative pays out.
     * Pass 0 to stop jogging. Used to set up the mechanism before zeroing.
     */
    public void jog(double power) {
        if (power == 0) {
            if (state == CatapultState.MANUAL) {
                // Letting go of the stick must not drop a cocked boom either.
                if (isStringTaut() && !latched) {
                    hold();
                } else {
                    setState(CatapultState.IDLE);
                }
            }
            return;
        }
        launchRequested = false;
        setState(CatapultState.MANUAL);
        motorPower = Range.clip(power, -1.0, 1.0) * JOG_POWER;
    }

    /**
     * Call this with the boom resting UP and the string just taut.
     * Do NOT call it on a cocked boom - resetting the encoder drops motor power
     * for an instant, which is exactly the thing the rest of this class avoids.
     */
    public void zeroEncoder() {
        winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lastMode = DcMotor.RunMode.RUN_USING_ENCODER;
        currentTicks = 0;
    }

    // ==================== QUERIES ====================

    /** True when the string is paid out far enough that firing is allowed. */
    public boolean isSlack() {
        return currentTicks <= SLACK_TICKS + POSITION_TOLERANCE;
    }

    /**
     * True when the spool still has the string pulled in far enough to be taking
     * the elastic load. Combined with !latched this is the "the winch is the only
     * thing holding the boom, do not let go" condition.
     */
    public boolean isStringTaut() {
        return !isSlack();
    }

    /** True once the latch was commanded closed and given time to physically arrive. */
    public boolean isLatched() {
        return latched;
    }

    /** Cocked, on the latch, string slack - loaded and safe to drive around. */
    public boolean isArmed() {
        return latched && isSlack();
    }

    public boolean isReadyToLaunch() {
        return state == CatapultState.READY;
    }

    public CatapultState getState() {
        return state;
    }

    // ==================== THREE-PHASE METHODS ====================

    @Override
    public void readSensors() {
        currentTicks = winch.getCurrentPosition();
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        long elapsed = System.currentTimeMillis() - stateStartMs;

        switch (state) {
            case IDLE:
                positionMode = false;
                motorPower = 0;
                break;

            case RETRACTING:
                latched = false;
                // Servo HOLDS THE LATCH OPEN for the whole wind-down. If the latch
                // drops before the pin is under it, it misses and the system jams.
                releaseTarget = RELEASE_RELEASED;
                positionMode = true;
                targetTicks = RETRACT_TICKS + RETRACT_OVERSHOOT_TICKS;
                motorPower = RETRACT_POWER;
                // At or PAST the pin point - never a tolerance window that could
                // let the latch down while the boom is still short of it.
                if (currentTicks >= RETRACT_TICKS) {
                    // Let the latch go in the same loop we transition, so the dwell
                    // timer and the servo command start together.
                    releaseTarget = RELEASE_CAPTURED;
                    setState(CatapultState.CAPTURING);
                } else if (elapsed > RETRACT_TIMEOUT_MS) {
                    // Never reached the pin point. Keep the latch OPEN - dropping it
                    // here is the jam case - and hold the boom with the winch.
                    releaseTarget = RELEASE_RELEASED;
                    hold();
                }
                break;

            case CAPTURING:
                // Pin is past the latch and the servo has let go, so the weak rubber
                // band is pulling the latch down. Nothing confirms it seated, so the
                // winch holds the boom past the pin for the whole dwell.
                releaseTarget = RELEASE_CAPTURED;
                positionMode = true;
                targetTicks = RETRACT_TICKS + RETRACT_OVERSHOOT_TICKS;
                motorPower = RETRACT_POWER;
                if (elapsed >= CAPTURE_DWELL_MS) {
                    // Latch is down over the pin, but only the weak band is holding
                    // it there. Slackening is what lets the boom rise onto it and
                    // develops the tension that actually seats it - so go do that.
                    latched = true;
                    setState(CatapultState.SLACKENING);
                }
                break;

            case SLACKENING:
                // Defensive: never pay out on an unlatched cocked boom.
                if (isStringTaut() && !latched) {
                    setState(CatapultState.CAPTURING);
                    break;
                }
                releaseTarget = RELEASE_CAPTURED;
                positionMode = true;
                targetTicks = SLACK_TICKS;
                motorPower = SLACK_POWER;
                if (isSlack() || elapsed > SLACK_TIMEOUT_MS) {
                    setState(CatapultState.READY);
                }
                break;

            case READY:
                // Spool is off and floating so the string is free to feed out
                // during the throw. The latch is holding the boom now.
                positionMode = false;
                motorPower = 0;
                if (launchRequested) {
                    setState(CatapultState.LAUNCHING);
                }
                break;

            case LAUNCHING:
                positionMode = false;
                motorPower = 0;
                // The one place the release is allowed to open, and only if truly slack.
                if (isSlack()) {
                    releaseTarget = RELEASE_RELEASED;
                    latched = false;
                } else {
                    releaseTarget = RELEASE_CAPTURED;
                }
                if (elapsed > LAUNCH_DWELL_MS) {
                    launchRequested = false;
                    setState(CatapultState.FIRED);
                }
                break;

            case FIRED:
                positionMode = false;
                motorPower = 0;
                break;

            case HOLDING:
                // Cocked boom, latch not confirmed. Hold position indefinitely -
                // the driver can retry retract(), jog, or launch out of this.
                positionMode = true;
                targetTicks = holdTicks;
                motorPower = RETRACT_POWER;
                break;

            case MANUAL:
                positionMode = false;
                // motorPower is set by jog()
                break;
        }
    }

    @Override
    public void act() {
        if (positionMode) {
            winch.setTargetPosition(targetTicks);
            setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        winch.setPower(motorPower);
        release.setPosition(releaseTarget);
    }

    @Override
    public void stop() {
        // OpMode is ending - the SDK cuts motor power regardless, so there is no
        // holding the boom here. Leave the latch commanded closed on the way out.
        launchRequested = false;
        state = CatapultState.IDLE;
        positionMode = false;
        motorPower = 0;
        winch.setPower(0);
    }

    @Override
    public void resetStates() {
        launchRequested = false;
        latched = false;  // Assume nothing about the latch after a reset
        motorPower = 0;
        if (isStringTaut()) {
            hold();
        } else {
            setState(CatapultState.IDLE);
        }
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();
        telemetry.put("State", state);
        telemetry.put("Ticks", currentTicks);
        telemetry.put("Armed", isArmed() ? "ARMED" : "no");
        telemetry.put("Latched", latched ? "YES" : "no");
        telemetry.put("String", isStringTaut() ? "TAUT" : "slack");
        telemetry.put("Launch ok", isSlack() ? "YES" : "no");
        telemetry.put("Release", releaseTarget == RELEASE_RELEASED ? "RELEASED" : "CAPTURED");
        if (debug) {
            telemetry.put("Target Ticks", positionMode ? targetTicks : -1);
            telemetry.put("Motor Power", motorPower);
            telemetry.put("Launch Requested", launchRequested);
        }
        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "Catapult";
    }

    // ==================== INTERNAL ====================

    private void setState(CatapultState next) {
        if (state != next) {
            state = next;
            stateStartMs = System.currentTimeMillis();
        }
    }

    /** Park the winch on the current position and keep it powered. */
    private void hold() {
        holdTicks = currentTicks;
        setState(CatapultState.HOLDING);
    }

    private void setMode(DcMotor.RunMode mode) {
        if (lastMode != mode) {
            winch.setMode(mode);
            lastMode = mode;
        }
    }
}
