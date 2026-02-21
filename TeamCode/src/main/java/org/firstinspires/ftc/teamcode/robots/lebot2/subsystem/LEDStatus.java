package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem;

import android.graphics.Color;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunLEDStick;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.lebot2.Robot;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * LED Status subsystem - visual feedback for drivers via SparkFun QWIIC LED Stick.
 *
 * Priority-based state machine (highest priority wins):
 * 1. FIRING - pulsing white (launcher actively firing)
 * 2. READY_TO_FIRE - bright white (vision locked + flywheel at speed)
 * 3. AIMING - cyan (vision has target, turret converging)
 * 4. FULL - green (loader full, timed on rising edge)
 * 5. HAS_BALLS - amber (loader has balls)
 * 6. ALLIANCE - red/blue (default)
 */
@Config(value = "Lebot2_LEDStatus")
public class LEDStatus implements Subsystem {

    // Hardware
    private SparkFunLEDStick ledStick;
    private boolean hardwarePresent = false;

    // Configuration
    public static int BRIGHTNESS = 10;  // 0-31
    public static int AMBER = Color.rgb(255, 140, 0);
    public static int CYAN = Color.rgb(0, 255, 255);
    public static long FULL_DISPLAY_MS = 1000;
    public static double PULSE_FREQUENCY_HZ = 2.0;

    // State
    public enum State {
        ALLIANCE,
        HAS_BALLS,
        FULL,
        AIMING,
        READY_TO_FIRE,
        FIRING
    }
    private State state = State.ALLIANCE;

    // Timing
    private long fullStartMs = 0;
    private boolean wasFullLastCycle = false;

    // References
    private Loader loader = null;
    private Launcher launcher = null;
    private Vision vision = null;
    private Turret turret = null;

    // Current display state
    private int currentColor = Color.BLUE;

    public LEDStatus(HardwareMap hardwareMap) {
        try {
            ledStick = hardwareMap.get(SparkFunLEDStick.class, "leds");
            ledStick.setBrightness(BRIGHTNESS);
            hardwarePresent = true;
        } catch (Exception e) {
            hardwarePresent = false;
        }
    }

    public void setLoader(Loader loader) { this.loader = loader; }
    public void setLauncher(Launcher launcher) { this.launcher = launcher; }
    public void setVision(Vision vision) { this.vision = vision; }
    public void setTurret(Turret turret) { this.turret = turret; }

    // ==================== THREE-PHASE METHODS ====================

    @Override
    public void readSensors() {
        // No sensors to read
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        if (!hardwarePresent) return;

        // Priority-based state determination (highest first)
        boolean isFiring = (launcher != null) &&
                (launcher.getState() == Launcher.LaunchState.FIRING ||
                 launcher.getState() == Launcher.LaunchState.LIFTING);

        boolean isReadyToFire = (turret != null) && turret.isLockedOnTarget() &&
                (launcher != null) && launcher.isFlywheelAtSpeed();

        boolean isAiming = (vision != null) && vision.hasTarget();

        boolean isFull = (loader != null) && loader.isFull();
        boolean hasBalls = (loader != null) && !loader.isEmpty();

        if (isFiring) {
            state = State.FIRING;
        } else if (isReadyToFire) {
            state = State.READY_TO_FIRE;
        } else if (isAiming) {
            state = State.AIMING;
        } else if (isFull && !wasFullLastCycle) {
            // Rising edge of full - start green display
            state = State.FULL;
            fullStartMs = System.currentTimeMillis();
        } else if (state == State.FULL) {
            // Check if green display time expired
            if (System.currentTimeMillis() - fullStartMs > FULL_DISPLAY_MS) {
                state = hasBalls ? State.HAS_BALLS : State.ALLIANCE;
            }
        } else if (hasBalls) {
            state = State.HAS_BALLS;
        } else {
            state = State.ALLIANCE;
        }

        wasFullLastCycle = isFull;

        // Determine color
        switch (state) {
            case FIRING:
                double phase = (System.currentTimeMillis() / 1000.0) * PULSE_FREQUENCY_HZ * 2 * Math.PI;
                int pulseValue = (int) (127 + 127 * Math.sin(phase));
                currentColor = Color.rgb(pulseValue, pulseValue, pulseValue);
                break;
            case READY_TO_FIRE:
                currentColor = Color.WHITE;
                break;
            case AIMING:
                currentColor = CYAN;
                break;
            case FULL:
                currentColor = Color.GREEN;
                break;
            case HAS_BALLS:
                currentColor = AMBER;
                break;
            case ALLIANCE:
            default:
                currentColor = Robot.isRedAlliance ? Color.RED : Color.BLUE;
                break;
        }
    }

    @Override
    public void act() {
        if (!hardwarePresent) return;

        ledStick.setBrightness(BRIGHTNESS);
        ledStick.setColor(currentColor);
    }

    @Override
    public void stop() {
        if (hardwarePresent) {
            ledStick.turnAllOff();
        }
    }

    @Override
    public void resetStates() {
        state = State.ALLIANCE;
        wasFullLastCycle = false;
        fullStartMs = 0;
    }

    // ==================== TELEMETRY ====================

    @Override
    public String getTelemetryName() {
        return "LEDStatus";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Present", hardwarePresent ? "YES" : "no");
        if (hardwarePresent) {
            telemetry.put("State", state);
            if (debug) {
                telemetry.put("Color", String.format("#%06X", currentColor & 0xFFFFFF));
            }
        }

        return telemetry;
    }
}
