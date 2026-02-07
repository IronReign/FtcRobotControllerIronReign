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
 * States:
 * - ALLIANCE: All LEDs show alliance color (blue/red) - default state
 * - HAS_BALLS: Amber when loader has balls but not full
 * - FULL: Green for 1 second when loader becomes full, then reverts to ALLIANCE
 * - FIRING: Pulsing white while launching, then reverts to ALLIANCE
 *
 * THREE-PHASE UPDATE:
 * - readSensors(): Nothing to read
 * - calc(): Determine state based on robot/loader/launcher status
 * - act(): Update LED colors
 */
@Config(value = "Lebot2_LEDStatus")
public class LEDStatus implements Subsystem {

    // Hardware
    private SparkFunLEDStick ledStick;
    private boolean hardwarePresent = false;

    // Configuration
    public static int BRIGHTNESS = 10;  // 0-31
    public static int AMBER = Color.rgb(255, 140, 0);  // Orange-amber
    public static long FULL_DISPLAY_MS = 1000;  // How long to show green when full
    public static double PULSE_FREQUENCY_HZ = 2.0;  // White pulse rate when firing

    // State
    public enum State {
        ALLIANCE,   // Default - show alliance color
        HAS_BALLS,  // Amber - loader has balls but not full
        FULL,       // Green - loader just became full (timed)
        FIRING      // Pulsing white - launching balls
    }
    private State state = State.ALLIANCE;
    private State previousState = State.ALLIANCE;

    // Timing
    private long fullStartMs = 0;  // When we entered FULL state
    private boolean wasFullLastCycle = false;  // Track rising edge of full

    // References
    private Loader loader = null;
    private Launcher launcher = null;

    // Current display state
    private int currentColor = Color.BLUE;

    public LEDStatus(HardwareMap hardwareMap) {
        try {
            ledStick = hardwareMap.get(SparkFunLEDStick.class, "leds");
            ledStick.setBrightness(BRIGHTNESS);
            hardwarePresent = true;
        } catch (Exception e) {
            // LED stick not configured - subsystem will be a no-op
            hardwarePresent = false;
        }
    }

    /**
     * Set reference to Loader for ball detection.
     */
    public void setLoader(Loader loader) {
        this.loader = loader;
    }

    /**
     * Set reference to Launcher for firing detection.
     */
    public void setLauncher(Launcher launcher) {
        this.launcher = launcher;
    }

    // ==================== THREE-PHASE METHODS ====================

    @Override
    public void readSensors() {
        // No sensors to read
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        if (!hardwarePresent) return;

        previousState = state;

        // Determine state based on priority (highest first)
        boolean isFiring = (launcher != null) &&
                (launcher.getState() == Launcher.LaunchState.FIRING ||
                 launcher.getState() == Launcher.LaunchState.LIFTING);

        boolean isFull = (loader != null) && loader.isFull();
        boolean hasBalls = (loader != null) && !loader.isEmpty();

        if (isFiring) {
            // Firing takes priority
            state = State.FIRING;
        } else if (isFull && !wasFullLastCycle) {
            // Rising edge of full - start green display
            state = State.FULL;
            fullStartMs = System.currentTimeMillis();
        } else if (state == State.FULL) {
            // Check if green display time expired
            if (System.currentTimeMillis() - fullStartMs > FULL_DISPLAY_MS) {
                state = State.ALLIANCE;
            }
            // Otherwise stay in FULL state
        } else if (hasBalls && state != State.FULL) {
            // Has balls but not full (and not in green flash)
            state = State.HAS_BALLS;
        } else if (state != State.FULL) {
            // Default to alliance color
            state = State.ALLIANCE;
        }

        wasFullLastCycle = isFull;

        // Determine color based on state
        switch (state) {
            case ALLIANCE:
                currentColor = Robot.isRedAlliance ? Color.RED : Color.BLUE;
                break;
            case HAS_BALLS:
                currentColor = AMBER;
                break;
            case FULL:
                currentColor = Color.GREEN;
                break;
            case FIRING:
                // Pulse white by modulating brightness with time
                double phase = (System.currentTimeMillis() / 1000.0) * PULSE_FREQUENCY_HZ * 2 * Math.PI;
                int pulseValue = (int) (127 + 127 * Math.sin(phase));  // 0-254
                currentColor = Color.rgb(pulseValue, pulseValue, pulseValue);
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
