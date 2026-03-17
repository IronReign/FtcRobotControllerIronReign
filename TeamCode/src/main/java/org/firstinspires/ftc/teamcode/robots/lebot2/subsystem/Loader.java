package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    private final LazyMotor intakeMotor;
    public final ChamberSensor chamberSensor;  // Three-sensor virtual ball counter

    public static double AMPS_AT_FULL = 0;
    public double intakeAmps = 0;

    private Launcher launcher;

    // Configuration
    // Negative = move balls toward rear (intake/feed), Positive = eject forward
    public static double BELT_POWER = -1;
    public static double FEED_POWER = -.7; // Slower for controlled feeding
    public static double BELT_REVERSE_POWER = 0.5; // Positive = toward front, relieves fin pressure
    public static double EJECT_POWER = 0.3;        // Gentle forward pulse to clear 4th ball
    public static double EJECT_DURATION_MS = 300;   // Duration of overfull eject pulse
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

    public static boolean loaderFull = false;

    // State
    public enum LoaderState {
        EMPTY,      // No balls in chamber
        HAS_BALLS,  // 1-2 balls in chamber
        FULL,       // 3 balls (max capacity)
        OVERFULL    // 4th ball trapped — needs brief eject
    }
    private LoaderState state = LoaderState.EMPTY;

    // Virtual sensor state (time-confirmed)
    private boolean isFullConfirmed = false;  // Debounced isFull virtual sensor
    private long fullConditionStartMs = 0;    // When raw full condition started

    // Overfull eject state
    private boolean ejecting = false;
    private long ejectStartMs = 0;

    // Belt power (resolved from ownership)
    private double beltPower = 0;

    public Loader(HardwareMap hardwareMap) {
        beltMotor = new LazyMotor(hardwareMap, "conveyor");
        intakeMotor = new LazyMotor(hardwareMap, "intake");
        beltMotor.setDirection(DcMotorSimple.Direction.REVERSE); // So negative = toward rear
        beltMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        beltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        chamberSensor = new ChamberSensor(hardwareMap);
    }

    // ==================== THREE-PHASE METHODS ====================

    @Override
    public void readSensors() {
        // PHASE 1: Refresh I2C sensors
        chamberSensor.refresh();
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        // PHASE 2: Read cached values and process logic

        // Update chamber sensor and read ball count
        chamberSensor.update();
        intakeAmps = beltMotor.getCurrent();

        // Debounced full confirmation
        boolean rawFull = chamberSensor.isFull();
        if (rawFull) {
            if (fullConditionStartMs == 0) {
                fullConditionStartMs = System.currentTimeMillis();
            }
            isFullConfirmed = (System.currentTimeMillis() - fullConditionStartMs) >= FULL_CONFIRM_MS;
        } else {
            fullConditionStartMs = 0;
            isFullConfirmed = false;
        }

        // State determination
        if (chamberSensor.isOverfull()) {
            state = LoaderState.OVERFULL;
        } else if (isFullConfirmed) {
            state = LoaderState.FULL;
        } else if (chamberSensor.getBallCount() > 0) {
            state = LoaderState.HAS_BALLS;
        } else {
            state = LoaderState.EMPTY;
        }

        // Overfull eject: brief gentle reverse to clear trapped 4th ball
        // Only triggers during intake (not during firing) and only once per detection
        if (state == LoaderState.OVERFULL && !ejecting && !launcherClaimsBelt) {
            ejecting = true;
            ejectStartMs = System.currentTimeMillis();
        }
        if (ejecting) {
            if (System.currentTimeMillis() - ejectStartMs < EJECT_DURATION_MS) {
                // Override belt with eject pulse
                beltPower = EJECT_POWER;
                beltMotor.setPower(beltPower);
                intakeMotor.setPower(beltPower);
                return; // Skip normal belt ownership for this cycle
            } else {
                // Eject complete
                ejecting = false;
            }
        }
        // Reset eject flag when no longer overfull so it can trigger again if needed
        if (state != LoaderState.OVERFULL) {
            ejecting = false;
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
        intakeMotor.setPower(beltPower);
    }

    @Override
    public void act() {
        // PHASE 3: Flush motor command
        beltMotor.flush();
        intakeMotor.flush();
    }

    public void setLauncher(Launcher launcher){
        this.launcher = launcher;
    }

    // ==================== BELT OWNERSHIP ====================

    /**
     * Request belt for intake operations.
     * Belt will run unless Launcher has claimed it.
     */
    public void requestBeltForIntake() {
        intakeRequestsBelt = true;
        // Reset full confirmation so LOAD_ALL gets a fresh debounce window
        fullConditionStartMs = 0;
        isFullConfirmed = false;
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
        intakeMotor.setPower(BELT_REVERSE_POWER);
    }



    /**
     * Stop belt after reverse pulse.
     * Bypasses ownership model - only use during launch sequence.
     */
    public void stopBeltForFiring() {
        beltMotor.setPower(0);
        intakeMotor.setPower(0);
    }

    // ==================== CURRENT MONITORING (for health check) ====================

    /**
     * Enable/disable current monitoring on the belt motor.
     * Only enable during health checks — adds I2C overhead.
     */
    public void enableBeltCurrentRead(boolean enabled) {
        beltMotor.enableCurrentRead(enabled);
        intakeMotor.enableCurrentRead(enabled);
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
     * Check if there's a ball at the rear ready to launch.
     */
    public boolean isBallAtBack() {
        return chamberSensor.hasBallsAtRear();
    }

    /**
     * Check if a 4th ball is trapped at the front.
     */
    public boolean isOverfull() {
        return chamberSensor.isOverfull();
    }

    /**
     * Get ball count from chamber sensor (0-3 launchable balls).
     */
    public int getBallCount() {
        return chamberSensor.getBallCount();
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
     * Check if chamber is full (raw, unconfirmed — based on sensor count, no debounce).
     */
    public boolean isFullRaw() {
        return chamberSensor.isFull();
    }

    /**
     * Check if chamber is empty.
     */
    public boolean isEmpty() {
        return chamberSensor.isEmpty();
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
     */
    public double getFrontDistance() {
        return chamberSensor.getFrontDistance();
    }

    /**
     * Get raw rear sensor distance.
     */
    public double getBackDistance() {
        return chamberSensor.getRearDistance();
    }

    @Override
    public void stop() {
        intakeRequestsBelt = false;
        launcherClaimsBelt = false;
        beltOwner = BeltOwner.NONE;
        beltPower = 0;
        beltMotor.stop();  // Immediate stop, bypasses lazy pattern
        intakeMotor.stop();
    }

    @Override
    public void resetStates() {
        intakeRequestsBelt = false;
        launcherClaimsBelt = false;
        beltOwner = BeltOwner.NONE;
        // Reset virtual sensor timing (but not ball count - that persists)
        fullConditionStartMs = 0;
        isFullConfirmed = false;
        ejecting = false;
    }

    @Override
    public String getTelemetryName() {
        return "Loader";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Loader Amps: ", beltMotor.getCurrent());

        telemetry.put("State", state);
        telemetry.put("Ball Count", chamberSensor.getBallCount());
        telemetry.put("Overfull", chamberSensor.isOverfull() ? (ejecting ? "EJECTING" : "EJECT!") : "no");
        telemetry.put("Belt Owner", beltOwner);

        if (debug) {
            telemetry.putAll(chamberSensor.getTelemetry());
            telemetry.put("isFull (confirmed)", isFullConfirmed ? "YES" : "no");
            telemetry.put("Belt Power", String.format("%.2f", beltPower));
            telemetry.put("Intake Requests", intakeRequestsBelt);
            telemetry.put("Launcher Claims", launcherClaimsBelt);
        }

        return telemetry;
    }
}
