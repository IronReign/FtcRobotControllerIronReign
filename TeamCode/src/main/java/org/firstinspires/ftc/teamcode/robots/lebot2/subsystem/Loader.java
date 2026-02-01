package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.CachedDistanceSensor;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.LazyMotor;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Loader subsystem - manages the ball chamber and feeding mechanism.
 *
 * The loader is the central chamber that holds up to 3 balls. It includes:
 * - Side belt (conveyor) motor: moves balls from front to rear
 * - Front distance sensor: detects balls entering the chamber
 * - Back distance sensor: detects balls ready to launch
 *
 * BELT OWNERSHIP:
 * The belt is a shared resource between Intake and Launcher.
 * - INTAKE: Runs belt to pull balls deeper (requested by Intake)
 * - LAUNCHER: Runs belt to feed balls to paddle (claimed by Launcher)
 * Priority: LAUNCHER > INTAKE > NONE
 *
 * Ball counting logic:
 * - Front sensor detects new balls entering
 * - Back sensor detects balls at the launch position
 * - Count is decremented when Launcher fires a ball
 *
 * THREE-PHASE UPDATE:
 * - readSensors(): Refresh distance sensors (I2C)
 * - calc(): Ball counting logic, belt ownership resolution
 * - act(): Flush motor command
 */
@Config(value = "Lebot2_Loader")
public class Loader implements Subsystem {

    // Hardware (wrapped for three-phase pattern)
    private final LazyMotor beltMotor;
    private final CachedDistanceSensor frontSensor;
    private final CachedDistanceSensor backSensor;

    // Configuration
    // Negative = move balls toward rear (intake/feed), Positive = eject forward
    public static double BELT_POWER = -1.0;
    public static double FEED_POWER = -0.8; // Slower for controlled feeding
    public static double BELT_REVERSE_POWER = 0.5; // Positive = toward front, relieves fin pressure
    public static double BALL_DETECT_THRESHOLD_CM = 10.0; // Distance indicating ball present
    public static int MAX_BALLS = 3;
    public static long FULL_CONFIRM_MS = 100; // Debounce time for isFull virtual sensor

    // Belt ownership - priority: LAUNCHER > INTAKE > NONE
    public enum BeltOwner {
        NONE,       // Belt stopped
        INTAKE,     // Intake is using belt to pull balls in
        LAUNCHER    // Launcher is using belt to feed balls (highest priority)
    }
    private BeltOwner beltOwner = BeltOwner.NONE;
    private boolean intakeRequestsBelt = false;
    private boolean launcherClaimsBelt = false;

    // State
    public enum LoaderState {
        EMPTY,      // No balls in chamber
        HAS_BALLS,  // 1-2 balls in chamber
        FULL        // 3 balls (max capacity)
    }
    private LoaderState state = LoaderState.EMPTY;

    // Ball tracking (sensor-based, no counting)
    private double frontDistance = 100; // cm, starts high (no ball)
    private double backDistance = 100;  // cm, starts high (no ball)
    private boolean ballAtFront = false;
    private boolean ballAtBack = false;

    // Virtual sensor state (time-confirmed)
    private boolean isFullConfirmed = false;  // Debounced isFull virtual sensor
    private long fullConditionStartMs = 0;    // When raw full condition started

    // Belt power (resolved from ownership)
    private double beltPower = 0;

    public Loader(HardwareMap hardwareMap) {
        beltMotor = new LazyMotor(hardwareMap, "conveyor");
        beltMotor.setDirection(DcMotorSimple.Direction.REVERSE); // So negative = toward rear
        beltMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        beltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontSensor = new CachedDistanceSensor(hardwareMap, "frontDist", DistanceUnit.CM);
        backSensor = new CachedDistanceSensor(hardwareMap, "backDist", DistanceUnit.CM);
    }

    // ==================== THREE-PHASE METHODS ====================

    @Override
    public void readSensors() {
        // PHASE 1: Refresh I2C sensors
        frontSensor.refresh();
        backSensor.refresh();
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        // PHASE 2: Read cached values and process logic

        // Get cached sensor values
        frontDistance = frontSensor.getDistance();
        backDistance = backSensor.getDistance();

        // Determine ball positions from sensors
        ballAtFront = frontDistance < BALL_DETECT_THRESHOLD_CM;
        ballAtBack = backDistance < BALL_DETECT_THRESHOLD_CM;

        // Update state based on sensor readings (no counting - unreliable)
        if (ballAtBack && ballAtFront) {
            state = LoaderState.FULL;
        } else if (ballAtBack || ballAtFront) {
            state = LoaderState.HAS_BALLS;
        } else {
            state = LoaderState.EMPTY;
        }

        // Compute time-confirmed isFull virtual sensor (debouncing)
        // Raw condition: state is FULL and back sensor confirms ball present
        boolean rawFull = (state == LoaderState.FULL) && ballAtBack;

        if (rawFull) {
            if (fullConditionStartMs == 0) {
                // Rising edge - start timing
                fullConditionStartMs = System.currentTimeMillis();
            }
            // Confirm after debounce period
            isFullConfirmed = (System.currentTimeMillis() - fullConditionStartMs) >= FULL_CONFIRM_MS;
        } else {
            // Condition not met - reset timer and clear confirmed state
            fullConditionStartMs = 0;
            isFullConfirmed = false;
        }

        // Resolve belt ownership (priority: LAUNCHER > INTAKE > NONE)
        if (launcherClaimsBelt) {
            beltOwner = BeltOwner.LAUNCHER;
            beltPower = FEED_POWER;
        } else if (intakeRequestsBelt) {
            beltOwner = BeltOwner.INTAKE;
            beltPower = BELT_POWER;
        } else {
            beltOwner = BeltOwner.NONE;
            beltPower = 0;
        }

        // Queue belt power (will be flushed in act())
        beltMotor.setPower(beltPower);
    }

    @Override
    public void act() {
        // PHASE 3: Flush motor command
        beltMotor.flush();
    }

    // ==================== BELT OWNERSHIP ====================

    /**
     * Request belt for intake operations.
     * Belt will run unless Launcher has claimed it.
     */
    public void requestBeltForIntake() {
        intakeRequestsBelt = true;
    }

    /**
     * Release intake's belt request.
     */
    public void releaseBeltFromIntake() {
        intakeRequestsBelt = false;
    }

    /**
     * Claim belt for launcher operations (highest priority).
     * This will override intake's request.
     */
    public void claimBeltForLauncher() {
        launcherClaimsBelt = true;
    }

    /**
     * Release launcher's belt claim.
     * Belt will return to intake if intake still requests it.
     */
    public void releaseBeltFromLauncher() {
        launcherClaimsBelt = false;
    }

    /**
     * Get current belt owner.
     */
    public BeltOwner getBeltOwner() {
        return beltOwner;
    }

    /**
     * Check if launcher currently owns the belt.
     */
    public boolean isLauncherUsingBelt() {
        return beltOwner == BeltOwner.LAUNCHER;
    }

    // ==================== FIRING SUPPORT (bypasses ownership) ====================

    /**
     * Reverse belt briefly to relieve fin pressure before paddle lift.
     * Bypasses ownership model - only use during launch sequence after releasing belt.
     */
    public void reverseBeltForFiring() {
        beltMotor.setPower(BELT_REVERSE_POWER);
    }

    /**
     * Stop belt after reverse pulse.
     * Bypasses ownership model - only use during launch sequence.
     */
    public void stopBeltForFiring() {
        beltMotor.setPower(0);
    }

    // ==================== CURRENT MONITORING (for health check) ====================

    /**
     * Enable/disable current monitoring on the belt motor.
     * Only enable during health checks — adds I2C overhead.
     */
    public void enableBeltCurrentRead(boolean enabled) {
        beltMotor.enableCurrentRead(enabled);
    }

    /**
     * Get cached belt motor current (amps). Requires enableBeltCurrentRead(true).
     */
    public double getBeltCurrent() {
        return beltMotor.getCurrent();
    }

    // ==================== LEGACY METHODS (for compatibility) ====================

    /**
     * Run belt to pull balls toward rear (used during intake).
     * @deprecated Use requestBeltForIntake() instead
     */
    public void runBelt() {
        intakeRequestsBelt = true;
    }

    /**
     * Run belt slowly to feed next ball (used during launch sequence).
     * @deprecated Use claimBeltForLauncher() instead
     */
    public void feedBall() {
        launcherClaimsBelt = true;
    }

    /**
     * Release the belt even if we don't know who owns it.
     * @deprecated Use releaseBeltFromIntake() or releaseBeltFromLauncher() instead
     */
    public void releaseBelt() {
//        intakeRequestsBelt = false;
//        launcherClaimsBelt = false;
        releaseBeltFromIntake();
        releaseBeltFromLauncher();
    }

    /**
     * Set belt to a specific power (bypasses ownership - use for manual control only).
     *
     * @param power Belt power (-1 to 1). Negative = toward rear, Positive = eject forward.
     */
    public void setBeltPower(double power) {
        // For manual override, claim as launcher (highest priority) if power is non-zero
        if (power != 0) {
            launcherClaimsBelt = true;
        } else {
            launcherClaimsBelt = false;
        }
    }

    /**
     * Check if there's a ball at the back ready to launch.
     *
     * @return true if back sensor detects a ball
     */
    public boolean isBallAtBack() {
        return ballAtBack;
    }

    /**
     * Check if there's a ball at the front (just entered).
     *
     * @return true if front sensor detects a ball
     */
    public boolean isBallAtFront() {
        return ballAtFront;
    }

    /**
     * Get estimated ball count based on sensors.
     * @deprecated Ball counting is unreliable. Use isEmpty()/isFull() instead.
     * @return Estimated count: 0 if empty, 2 if full, 1 otherwise
     */
    @Deprecated
    public int getBallCount() {
        if (state == LoaderState.EMPTY) return 0;
        if (state == LoaderState.FULL) return 2;
        return 1;
    }

    /**
     * Check if chamber is full (time-confirmed virtual sensor).
     * Returns true only after the full condition has been stable for FULL_CONFIRM_MS.
     * This prevents flickering when balls are at sensor thresholds.
     *
     * @return true if confirmed full (debounced)
     */
    public boolean isFull() {
        return isFullConfirmed;
    }

    /**
     * Check if chamber is full (raw, unconfirmed).
     * Use for debugging or when immediate response is needed.
     *
     * @return true if both sensors detect balls
     */
    public boolean isFullRaw() {
        return state == LoaderState.FULL;
    }

    /**
     * Check if chamber is empty.
     *
     * @return true if no sensors detect balls
     */
    public boolean isEmpty() {
        return state == LoaderState.EMPTY;
    }

    /**
     * Called by Launcher when a ball is fired.
     * @deprecated No-op - ball counting removed. State is sensor-based now.
     */
    @Deprecated
    public void ballFired() {
        // No-op - ball counting removed, state is purely sensor-based
    }

    /**
     * Reset loader state (e.g., at start of match).
     * @deprecated State is sensor-based - this does nothing meaningful.
     */
    @Deprecated
    public void resetBallCount() {
        // State will update from sensors on next calc() cycle
    }

    /**
     * Get raw front sensor distance.
     *
     * @return Distance in cm
     */
    public double getFrontDistance() {
        return frontDistance;
    }

    /**
     * Get raw back sensor distance.
     *
     * @return Distance in cm
     */
    public double getBackDistance() {
        return backDistance;
    }

    @Override
    public void stop() {
        intakeRequestsBelt = false;
        launcherClaimsBelt = false;
        beltOwner = BeltOwner.NONE;
        beltPower = 0;
        beltMotor.stop();  // Immediate stop, bypasses lazy pattern
    }

    @Override
    public void resetStates() {
        intakeRequestsBelt = false;
        launcherClaimsBelt = false;
        beltOwner = BeltOwner.NONE;
        // Reset virtual sensor timing (but not ball count - that persists)
        fullConditionStartMs = 0;
        isFullConfirmed = false;
    }

    @Override
    public String getTelemetryName() {
        return "Loader";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("State", state);
        telemetry.put("Ball at Front", ballAtFront ? "YES" : "no");
        telemetry.put("Ball at Back", ballAtBack ? "YES" : "no");
        telemetry.put("Front Dist", String.format("%.1f", frontDistance));
        telemetry.put("Back Dist", String.format("%.1f", backDistance));
        telemetry.put("Belt Owner", beltOwner);

        if (debug) {
            telemetry.put("isFull (confirmed)", isFullConfirmed ? "YES" : "no");
            telemetry.put("Belt Power", String.format("%.2f", beltPower));
            telemetry.put("Intake Requests", intakeRequestsBelt);
            telemetry.put("Launcher Claims", launcherClaimsBelt);
        }

        return telemetry;
    }
}
