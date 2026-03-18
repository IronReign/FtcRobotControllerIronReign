package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.lebot2.util.LazyMotor;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Loader subsystem - manages the ball chamber and feeding mechanism.
 *
 * The loader is the central chamber that holds up to 3 balls. It includes:
 * - Intake motor: sole motor that moves balls through the channel
 * - Three distance sensors (via ChamberSensor): rear, mid, front for ball counting
 *
 * INTAKE OWNERSHIP:
 * The intake motor is a shared resource between Intake and Launcher.
 * - INTAKE: Runs motor to pull balls deeper (requested by Intake)
 * - LAUNCHER: Runs motor to feed balls to flywheel (claimed by Launcher)
 * Priority: LAUNCHER > INTAKE > NONE
 *
 * THREE-PHASE UPDATE:
 * - readSensors(): Refresh distance sensors (I2C)
 * - calc(): Ball counting logic, motor ownership resolution
 * - act(): Flush motor command
 */
@Config(value = "Lebot2_Loader")
public class Loader implements Subsystem {

    // Hardware (wrapped for three-phase pattern)
    private final LazyMotor intakeMotor;       // Sole motor — moves balls through channel
    public final ChamberSensor chamberSensor;  // Three-sensor virtual ball counter

    public static double AMPS_AT_FULL = 0;
    public double intakeAmps = 0;

    private Launcher launcher;

    // Configuration
    // Negative = move balls toward rear (intake/feed), Positive = eject forward
    public static double INTAKE_POWER = -1;
    public static double FEED_POWER = -.7;         // Slower for controlled feeding to flywheel
    public static double REVERSE_POWER = 0.5;      // Positive = toward front
    public static double EJECT_POWER = 0.8;         // Forward pulse to clear 4th ball
    public static double EJECT_DURATION_MS = 200;   // Duration of overfull eject pulse
    public static double EJECT_RECOVERY_MS = 300;   // Reverse pulse after eject to reseat 3rd ball
    public static double EJECT_RECOVERY_POWER = -1.0; // Toward rear — pull 3rd ball back
    public static int MAX_BALLS = 3;
    public static long FULL_CONFIRM_MS = 100; // Debounce time for isFull virtual sensor

    // Intake motor ownership - priority: LAUNCHER > INTAKE > NONE
    public enum BeltOwner {
        NONE,       // Motor stopped
        INTAKE,     // Intake is using motor to pull balls in
        LAUNCHER    // Launcher is using motor to feed balls (highest priority)
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

    // Overfull eject state: EJECT phase (forward) → RECOVERY phase (reverse to reseat)
    private enum EjectPhase { IDLE, EJECTING, RECOVERING }
    private EjectPhase ejectPhase = EjectPhase.IDLE;
    private long ejectStartMs = 0;

    // Motor power (resolved from ownership)
    private double motorPower = 0;

    public Loader(HardwareMap hardwareMap) {
        intakeMotor = new LazyMotor(hardwareMap, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        intakeAmps = intakeMotor.getCurrent();

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

        // Overfull eject: two-phase sequence
        //   EJECTING:   forward pulse to push 4th ball out
        //   RECOVERING:  reverse pulse to pull 3rd ball back into position
        long now = System.currentTimeMillis();
        if (state == LoaderState.OVERFULL && ejectPhase == EjectPhase.IDLE && !launcherClaimsBelt) {
            ejectPhase = EjectPhase.EJECTING;
            ejectStartMs = now;
        }
        if (ejectPhase == EjectPhase.EJECTING) {
            if (now - ejectStartMs < EJECT_DURATION_MS) {
                motorPower = EJECT_POWER;
                intakeMotor.setPower(motorPower);
                return;
            } else {
                // Eject done — transition to recovery
                ejectPhase = EjectPhase.RECOVERING;
                ejectStartMs = now;
            }
        }
        if (ejectPhase == EjectPhase.RECOVERING) {
            if (now - ejectStartMs < EJECT_RECOVERY_MS) {
                motorPower = EJECT_RECOVERY_POWER;
                intakeMotor.setPower(motorPower);
                return;
            } else {
                // Recovery done — back to idle
                ejectPhase = EjectPhase.IDLE;
            }
        }

        // Resolve motor ownership (priority: LAUNCHER > INTAKE > NONE)
        if (launcherClaimsBelt) {
            beltOwner = BeltOwner.LAUNCHER;
            motorPower = FEED_POWER;
        } else if (intakeRequestsBelt) {
            beltOwner = BeltOwner.INTAKE;
            motorPower = INTAKE_POWER;
        } else {
            beltOwner = BeltOwner.NONE;
            motorPower = 0;
        }

        // Queue motor power (will be flushed in act())
        intakeMotor.setPower(motorPower);
    }

    @Override
    public void act() {
        // PHASE 3: Flush motor command
        intakeMotor.flush();
    }

    public void setLauncher(Launcher launcher){
        this.launcher = launcher;
    }

    // ==================== MOTOR OWNERSHIP ====================

    /**
     * Request intake motor for intake operations.
     * Motor will run unless Launcher has claimed it.
     */
    public void requestBeltForIntake() {
        intakeRequestsBelt = true;
        // Reset full confirmation so LOAD_ALL gets a fresh debounce window
        fullConditionStartMs = 0;
        isFullConfirmed = false;
    }

    /**
     * Release intake's motor request.
     */
    public void releaseBeltFromIntake() {
        intakeRequestsBelt = false;
    }

    /**
     * Claim intake motor for launcher operations (highest priority).
     * This will override intake's request.
     */
    public void claimBeltForLauncher() {
        launcherClaimsBelt = true;
    }

    /**
     * Release launcher's motor claim.
     * Motor will return to intake if intake still requests it.
     */
    public void releaseBeltFromLauncher() {
        launcherClaimsBelt = false;
    }

    /**
     * Get current motor owner.
     */
    public BeltOwner getBeltOwner() {
        return beltOwner;
    }

    /**
     * Manually trigger an eject pulse (same mechanism as auto-eject on overfull).
     * Runs intake forward at EJECT_POWER for EJECT_DURATION_MS.
     */
    public void triggerEject() {
        ejectPhase = EjectPhase.EJECTING;
        ejectStartMs = System.currentTimeMillis();
    }

    /**
    public boolean isLauncherUsingBelt() {
        return beltOwner == BeltOwner.LAUNCHER;
    }

    // ==================== FIRING SUPPORT (bypasses ownership) ====================

    /**
     * Reverse intake briefly to relieve pressure before paddle lift.
     * Bypasses ownership model - only use during launch sequence.
     */
    public void reverseBeltForFiring() {
        intakeMotor.setPower(REVERSE_POWER);
    }

    /**
     * Stop intake after reverse pulse.
     * Bypasses ownership model - only use during launch sequence.
     */
    public void stopBeltForFiring() {
        intakeMotor.setPower(0);
    }

    // ==================== CURRENT MONITORING (for health check) ====================

    /**
     * Enable/disable current monitoring on the intake motor.
     * Only enable during health checks — adds I2C overhead.
     */
    public void enableBeltCurrentRead(boolean enabled) {
        intakeMotor.enableCurrentRead(enabled);
    }

    /**
     * Get cached intake motor current (amps). Requires enableBeltCurrentRead(true).
     */
    public double getBeltCurrent() {
        return intakeMotor.getCurrent();
    }

    // ==================== LEGACY METHODS (for compatibility) ====================

    /**
     * @deprecated Use requestBeltForIntake() instead
     */
    @Deprecated
    public void runBelt() {
        intakeRequestsBelt = true;
    }

    /**
     * @deprecated Use claimBeltForLauncher() instead
     */
    @Deprecated
    public void feedBall() {
        launcherClaimsBelt = true;
    }

    /**
     * @deprecated Use releaseBeltFromIntake() or releaseBeltFromLauncher() instead
     */
    @Deprecated
    public void releaseBelt() {
        releaseBeltFromIntake();
        releaseBeltFromLauncher();
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
     * @deprecated No-op - state is sensor-based now.
     */
    @Deprecated
    public void ballFired() {
    }

    /**
     * @deprecated State is sensor-based - this does nothing meaningful.
     */
    @Deprecated
    public void resetBallCount() {
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
        motorPower = 0;
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
        ejectPhase = EjectPhase.IDLE;
    }

    @Override
    public String getTelemetryName() {
        return "Loader";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Intake Amps", String.format("%.2f", intakeMotor.getCurrent()));

        telemetry.put("State", state);
        telemetry.put("Ball Count", chamberSensor.getBallCount());
        telemetry.put("Overfull", chamberSensor.isOverfull() ?
                (ejectPhase == EjectPhase.EJECTING ? "EJECTING" :
                 ejectPhase == EjectPhase.RECOVERING ? "RECOVERING" : "EJECT!") : "no");
        telemetry.put("Motor Owner", beltOwner);

        if (debug) {
            telemetry.putAll(chamberSensor.getTelemetry());
            telemetry.put("isFull (confirmed)", isFullConfirmed ? "YES" : "no");
            telemetry.put("Motor Power", String.format("%.2f", motorPower));
            telemetry.put("Intake Requests", intakeRequestsBelt);
            telemetry.put("Launcher Claims", launcherClaimsBelt);
        }

        return telemetry;
    }
}
