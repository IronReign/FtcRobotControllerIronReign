package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem;

import android.graphics.Color;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunLEDStick;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
    private DcMotorSimple godMotor;
    private Servo glowServo;
    private boolean hardwarePresent = false;

    // Configuration
    public static double GOD_POWER = 0.3;          // Dashboard tunable — max safe value is 0.5
    private static final double GOD_MAX_POWER = 0.5; // Hard clamp — higher burns out the LED
    public static double GLOW_PATTERN = 0.0;        // Dashboard tunable — Blinkin pattern select (0.0-1.0)
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
        try {
            godMotor = hardwareMap.get(DcMotorSimple.class, "god");
        } catch (Exception e) {
            godMotor = null;
        }
        try {
            glowServo = hardwareMap.get(Servo.class, "glow");
        } catch (Exception e) {
            glowServo = null;
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
        } else if (isFull) {
            state = State.FULL;   // Stay green as long as full
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
        if (hardwarePresent) {
            ledStick.setBrightness(BRIGHTNESS);
            ledStick.setColor(currentColor);
        }
        if (godMotor != null) {
            godMotor.setPower(Math.min(GOD_POWER, GOD_MAX_POWER));
        }
        if (glowServo != null) {
            glowServo.setPosition(GLOW_PATTERN);
        }
    }

    @Override
    public void stop() {
        if (hardwarePresent) {
            ledStick.turnAllOff();
        }
        if (godMotor != null) {
            godMotor.setPower(0);
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
