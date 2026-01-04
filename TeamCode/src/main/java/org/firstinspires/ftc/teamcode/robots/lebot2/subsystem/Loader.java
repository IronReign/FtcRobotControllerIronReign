package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
 * The side belt has dual roles:
 * 1. During INTAKE: runs to pull balls deeper, making room for more
 * 2. During LAUNCH: runs to feed the next ball to the paddle
 *
 * Ball counting logic:
 * - Front sensor detects new balls entering
 * - Back sensor detects balls at the launch position
 * - Count is decremented when Launcher fires a ball
 */
@Config(value = "Lebot2_Loader")
public class Loader implements Subsystem {

    // Hardware
    private final DcMotorEx beltMotor;
    private final Rev2mDistanceSensor frontSensor;
    private final Rev2mDistanceSensor backSensor;

    // Configuration
    public static double BELT_POWER = 1.0;
    public static double FEED_POWER = 0.8; // Slower for controlled feeding
    public static double BALL_DETECT_THRESHOLD_CM = 10.0; // Distance indicating ball present
    public static int MAX_BALLS = 3;

    // State
    public enum LoaderState {
        EMPTY,      // No balls in chamber
        HAS_BALLS,  // 1-2 balls in chamber
        FULL        // 3 balls (max capacity)
    }
    private LoaderState state = LoaderState.EMPTY;

    // Ball tracking
    private int ballCount = 0;
    private double frontDistance = 100; // cm, starts high (no ball)
    private double backDistance = 100;  // cm, starts high (no ball)
    private boolean ballAtFront = false;
    private boolean ballAtBack = false;
    private boolean wasBallAtFront = false; // For edge detection

    // Belt control
    private double beltPower = 0;

    public Loader(HardwareMap hardwareMap) {
        beltMotor = hardwareMap.get(DcMotorEx.class, "conveyor");
        beltMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        beltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontSensor = hardwareMap.get(Rev2mDistanceSensor.class, "frontDist");
        backSensor = hardwareMap.get(Rev2mDistanceSensor.class, "backDist");
    }

    @Override
    public void update(Canvas fieldOverlay) {
        /*// READ: Get sensor values
        frontDistance = frontSensor.getDistance(DistanceUnit.CM);
        backDistance = backSensor.getDistance(DistanceUnit.CM);

        // PROCESS: Determine ball positions
        wasBallAtFront = ballAtFront;
        ballAtFront = frontDistance < BALL_DETECT_THRESHOLD_CM;
        ballAtBack = backDistance < BALL_DETECT_THRESHOLD_CM;

        // Detect new ball entering (rising edge on front sensor)
        if (ballAtFront && !wasBallAtFront && ballCount < MAX_BALLS) {
            ballCount++;
        }

        // Update state based on ball count
        if (ballCount == 0) {
            state = LoaderState.EMPTY;
        } else if (ballCount >= MAX_BALLS) {
            state = LoaderState.FULL;
        } else {
            state = LoaderState.HAS_BALLS;
        }*/

        // WRITE: Apply belt power
        beltMotor.setPower(beltPower);
    }

    /**
     * Run belt to pull balls toward rear (used during intake).
     */
    public void runBelt() {
        beltPower = BELT_POWER;
    }

    /**
     * Run belt slowly to feed next ball (used during launch sequence).
     */
    public void feedBall() {
        beltPower = FEED_POWER;
    }

    /**
     * Stop the belt.
     */
    public void stopBelt() {
        beltPower = 0;
    }

    /**
     * Set belt to a specific power.
     *
     * @param power Belt power (-1 to 1). Positive = toward rear (normal).
     */
    public void setBeltPower(double power) {
        beltPower = power;
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
     * Get the current ball count.
     *
     * @return Number of balls (0 to MAX_BALLS)
     */
    public int getBallCount() {
        return ballCount;
    }

    /**
     * Check if chamber is full.
     *
     * @return true if ballCount == MAX_BALLS
     */
    public boolean isFull() {
        return state == LoaderState.FULL;
    }

    /**
     * Check if chamber is empty.
     *
     * @return true if ballCount == 0
     */
    public boolean isEmpty() {
        return state == LoaderState.EMPTY;
    }

    /**
     * Called by Launcher when a ball is fired.
     * Decrements the ball count.
     */
    public void ballFired() {
        if (ballCount > 0) {
            ballCount--;
        }
    }

    /**
     * Reset ball count (e.g., at start of match).
     */
    public void resetBallCount() {
        ballCount = 0;
        state = LoaderState.EMPTY;
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
        beltPower = 0;
        beltMotor.setPower(0);
    }

    @Override
    public void resetStates() {
        stopBelt();
        // Don't reset ball count - that persists
    }

    @Override
    public String getTelemetryName() {
        return "Loader";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("State", state);
        telemetry.put("Ball Count", ballCount + "/" + MAX_BALLS);
        telemetry.put("Ball at Back", ballAtBack ? "YES" : "no");

        if (debug) {
            telemetry.put("Ball at Front", ballAtFront ? "YES" : "no");
            telemetry.put("Front Dist (cm)", String.format("%.1f", frontDistance));
            telemetry.put("Back Dist (cm)", String.format("%.1f", backDistance));
            telemetry.put("Belt Power", String.format("%.2f", beltPower));
        }

        return telemetry;
    }
}
