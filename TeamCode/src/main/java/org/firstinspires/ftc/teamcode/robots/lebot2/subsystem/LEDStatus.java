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
    public static int YELLOW = Color.rgb(255, 255, 0);
    public static double PULSE_FREQUENCY_HZ = 2.0;   // firing white pulse
    public static double OVERFULL_FLASH_HZ = 4.0;    // overfull green flash

    // Zone layout on the SparkFun QWIIC LED Stick (10 LEDs, array indices 0-9):
    //   0 and 9  -> alliance color, always on (so the alliance is always visible)
    //   1-4      -> loader zone:  off=empty, amber=has balls, green=full, flashing green=overfull
    //   5-8      -> turret/vision zone: off, yellow=odo pose-seeking, cyan=vision tracking,
    //               white=locked/ready, pulsing white=firing
    private static final int LED_COUNT = 10;
    private final int[] ledColors = new int[LED_COUNT];

    // References
    private Loader loader = null;
    private Launcher launcher = null;
    private Vision vision = null;
    private Turret turret = null;

    // Telemetry labels for the two zones
    private String loaderZone = "off";
    private String turretZone = "off";

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

        long now = System.currentTimeMillis();

        // ===== Alliance ends (0, 9) =====
        int allianceColor = Robot.isRedAlliance ? Color.RED : Color.BLUE;

        // ===== Loader zone (1-4) =====
        int loaderColor;
        if (loader == null) {
            loaderColor = Color.BLACK;
            loaderZone = "n/a";
        } else if (loader.isOverfull()) {
            // Flashing green — full AND a 4th ball needs clearing
            long halfPeriodMs = (long) (500.0 / OVERFULL_FLASH_HZ);
            boolean on = (now / Math.max(1, halfPeriodMs)) % 2 == 0;
            loaderColor = on ? Color.GREEN : Color.BLACK;
            loaderZone = "OVERFULL";
        } else if (loader.isFull()) {
            loaderColor = Color.GREEN;
            loaderZone = "FULL";
        } else if (!loader.isEmpty()) {
            loaderColor = AMBER;
            loaderZone = "has balls";
        } else {
            loaderColor = Color.BLACK;
            loaderZone = "empty";
        }

        // ===== Turret / vision zone (5-8) =====
        boolean isFiring = (launcher != null) &&
                (launcher.getState() == Launcher.LaunchState.FIRING ||
                 launcher.getState() == Launcher.LaunchState.LIFTING);
        int turretColor;
        if (isFiring) {
            double phase = (now / 1000.0) * PULSE_FREQUENCY_HZ * 2 * Math.PI;
            int p = (int) (127 + 127 * Math.sin(phase));
            turretColor = Color.rgb(p, p, p);   // pulsing white
            turretZone = "FIRING";
        } else if (turret != null && (turret.isReadyToLaunch() || turret.isReadyToLaunchDegraded())) {
            turretColor = Color.WHITE;
            turretZone = "ready";
        } else if (turret != null && turret.getPhase() == Turret.TargetingPhase.VISION_TRACKING) {
            turretColor = CYAN;
            turretZone = "vision";
        } else if (turret != null && turret.getPhase() == Turret.TargetingPhase.POSE_SEEKING) {
            turretColor = YELLOW;
            turretZone = "pose-seek";
        } else {
            turretColor = Color.BLACK;
            turretZone = "off";
        }

        // ===== Assemble the strip =====
        ledColors[0] = allianceColor;
        ledColors[9] = allianceColor;
        for (int i = 1; i <= 4; i++) ledColors[i] = loaderColor;
        for (int i = 5; i <= 8; i++) ledColors[i] = turretColor;
    }

    @Override
    public void act() {
        if (hardwarePresent) {
            ledStick.setBrightness(BRIGHTNESS);
            ledStick.setColors(ledColors);   // whole strip in one update (3 I2C writes)
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
        loaderZone = "off";
        turretZone = "off";
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
            telemetry.put("Loader zone", loaderZone);
            telemetry.put("Turret zone", turretZone);
        }

        return telemetry;
    }
}
