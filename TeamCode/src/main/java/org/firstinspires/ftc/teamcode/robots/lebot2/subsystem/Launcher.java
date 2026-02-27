package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.LazyServo;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;

import org.firstinspires.ftc.teamcode.util.CsvLogKeeper;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

/**
 * Launcher subsystem - controls the flywheel and paddle for launching balls.
 *
 * The launcher consists of:
 * - Flywheel motor: spins up to calculated speed based on target distance
 * - Paddle servo: lifts balls into contact with the flywheel
 *
 * BEHAVIOR PATTERN:
 * User sets behavior via setBehavior():
 * - IDLE: Flywheel off, paddle down (default)
 * - SPINNING: Flywheel at speed, ready to fire. Speed auto-calculated from Vision.
 *
 * Call fire() when behavior is SPINNING and isReady() returns true.
 *
 * INTERNAL STATE MACHINE (LaunchState):
 * IDLE → SPINNING_UP → READY → FIRING → COOLDOWN → (back to READY or IDLE)
 *
 * The internal state handles the complexity of spin-up timing, firing sequence,
 * and cooldown. Users only interact with Behavior and fire().
 *
 * THREE-PHASE UPDATE:
 * - readSensors(): Nothing needed (motor velocity is SDK bulk-cached)
 * - calc(): Update target speed from Vision, run state machine
 * - act(): Flush servo command
 */
@Config(value = "Lebot2_Launcher")
public class Launcher implements Subsystem {

    public static double NEW_P = 400;
    public static double NEW_I = 0;
    public static double NEW_D = 0;
    public static double NEW_F = 20;


    // Hardware
    private final DcMotorEx flywheel;  // Not wrapped - SDK handles velocity control
    private final DcMotorEx flywheelHelp;
    private final LazyServo paddle;    // Wrapped for three-phase pattern

    PIDFCoefficients pidfOrig;
    PIDFCoefficients pidNew;
    PIDFCoefficients pidfModified;

    // Trigger type selection (Dashboard-tunable)
    public enum TriggerType {
        STAR,   // Continuous rotation star — just spins to feed balls
        STAR_POSE,
        RAMP,   // TPU ramp with CUP/RAMP/LIFT positions
        PADDLE,  // Original paddle with DOWN/UP positions
        GATE
    }
    public static TriggerType TRIGGER_TYPE = TriggerType.STAR_POSE;

    //gate trigger positions
    public static int GATE_CLOSE = 800;      ////TODO: measure
     public static int GATE_OPEN = 1800;      ////TODO: measure

    // Star trigger positions (continuous rotation servo)
    // 0.5 = stopped/idle, 1.0 = spinning to feed balls into flywheel
    public static double STAR_IDLE = 0.5;
    public static double STAR_FEEDING = 1.0;

    // Star discrete position pulse widths (microseconds, converted via servoNormalize)
    public static int STAR_REST = 1900;       // Position 0: intake/idle - all 3 sockets open
    public static int STAR_LAUNCH_1 = 1500;   // Position 1: fires ball 1
    public static int STAR_LAUNCH_2 = 1100;   // Position 2: fires ball 2
    public static int STAR_LAUNCH_3 = 700;    // Position 3: fires ball 3
    // Current star position: 0=REST/intake, 1-3=successive firing positions
    public static int STAR_FIRED = 0;

    // Ramp trigger positions (servo units 0-1)
    public static double PADDLE_CUP = 0.4;
    public static double PADDLE_RAMP = 0.64;
    public static double PADDLE_LIFT = 0.7;

    // Original paddle positions (kept as fallback)
    public static double PADDLE_DOWN = 0.167;
    public static double PADDLE_UP = 0.53;
    public static double PADDLE_PASS = 0.53;

    // Flywheel configuration
    public static double SPEED_MULTIPLIER = 1.0;    // Tunable fudge factor until speed formula is recalibrated
    public static double MIN_LAUNCH_SPEED = 1040;   //720 <--old     // degrees/sec - hardcoded working speed from known position
    public static double SPEED_TOLERANCE = 15;      // degrees/sec margin for "at speed" check
    public static double FLYWHEEL_SPINDOWN_TIME = 0.5; // seconds
    public static double FLYWHEEL_IDLE_SPEED = 800;

    // Launch timing for TPU ramp design
    public static double FIRING_TIME = 2.1;         // seconds to allow all 3 balls through at conveyor speed
    public static double LIFT_TIME = 0.3;           // seconds to hold LIFT position for last ball

    // Post-fire behavior
    public static boolean STAY_SPINNING_AFTER_FIRE = true;  // Keep flywheel spinning for faster follow-up

    // Speed calculation constants (from physics model)
    public static double LIMELIGHT_OFFSET = 0.23;   // meters from launch point
    public static double LAUNCH_ANGLE = 60;         // degrees
    public static double TARGET_HEIGHT = 0.711;     // meters

    // ==================== BEHAVIOR (user-facing) ====================

    /**
     * User-selectable launcher behaviors.
     * Set via setBehavior(), query via getBehavior().
     */
    public enum Behavior {
        IDLE,       // Flywheel off, paddle down (default)
        SPINNING    // Flywheel at speed, ready to fire
    }
    private Behavior behavior = Behavior.IDLE;

    // ==================== INTERNAL STATE MACHINE ====================

    /**
     * Internal state for launch sequence execution.
     * Users don't set this directly - it's managed by the behavior.
     */
    //LAUNCH STATES FOR PADDLE DESIGN
//    public enum LaunchState {
//        IDLE,           // Flywheel off, paddle down
//        SPINNING_UP,    // Flywheel ramping to speed
//        READY,          // Flywheel at speed, waiting for fire command
//        BELT_REVERSING, // Brief reverse pulse to relieve fin pressure
//        FIRING,         // Paddle lifting ball into flywheel
//        COOLDOWN,        // Paddle returning, preparing for next
//        MANUAL
//    }

    // Simplified states for TPU ramp design
    public enum LaunchState {
        IDLE_SPIN,
        IDLE,           // Flywheel off, paddle in CUP
        SPINNING_UP,    // Flywheel ramping to speed, paddle in CUP
        READY,          // Flywheel at speed, waiting for fire command, paddle in CUP
        FIRING,         // Paddle at RAMP, conveyor pushing balls through
        LIFTING,        // Paddle at LIFT to push last ball into flywheel
        COMPLETE,       // Firing done, transitioning based on STAY_SPINNING_AFTER_FIRE
        MANUAL          // Manual control for testing
    }
    private LaunchState state = LaunchState.IDLE;

    // State data
    private double targetSpeed = MIN_LAUNCH_SPEED;
    private double currentSpeed = 0;            //primary flywheel velocity
    private double helperSpeed = 0;             //helper flywheel velocity

    private long stateTimer = 0;
    private long launchSpacerTimer = 0;
    public static double LAUNCH_SPACER_TIMER = .5;
    public static double LAUNCH_SPACER_TIMER_LAST = 1.0;  // longer for last ball - no balls behind it pushing
    private boolean fireRequested = false;
    private boolean passThroughMode = false;

    // Debug: track speed when firing was allowed
    private double speedAtFireApproval = 0;
    private int shotNumber = 0;

    // References to other subsystems for coordination
    private Loader loader = null;
    private Intake intake = null;
    private Vision vision = null;
    private Turret turret = null;

    // Track if we've claimed resources this firing cycle
    private boolean resourcesClaimed = false;

    // CSV logging for flywheel debugging
    public static boolean LOGGING_ENABLED = false;  // Dashboard tunable
    private CsvLogKeeper flywheelLog = null;
    private long logStartTime = 0;

    public Launcher(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");
//        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pidfOrig = flywheel.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //pidfOrig = flywheel.get

        flywheelHelp = hardwareMap.get(DcMotorEx.class, "shooter helper");
        flywheelHelp.setDirection(DcMotorEx.Direction.REVERSE);         //check if reversed or not
        flywheelHelp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelHelp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        paddle = new LazyServo(hardwareMap, "paddle");
        paddle.setPosition(getTriggerIdlePosition());
        paddle.flush();  // Apply initial position immediately
    }

    /**
     * Set reference to Loader for coordinated operations.
     * Call this after all subsystems are created.
     */
    public void setLoader(Loader loader) {
        this.loader = loader;
    }

    /**
     * Set reference to Intake for suppression during firing.
     * Call this after all subsystems are created.
     */
    public void setIntake(Intake intake) {
        this.intake = intake;
    }

    /**
     * Set reference to Vision for distance-based speed calculation.
     * Call this after all subsystems are created.
     */
    public void setVision(Vision vision) {
        this.vision = vision;
    }

    public void setTurret(Turret turret){
        this.turret = turret;
    }

    // ==================== BEHAVIOR CONTROL ====================

    /**
     * Set the launcher behavior.
     *
     * @param newBehavior IDLE or SPINNING
     */
    public void setBehavior(Behavior newBehavior) {
        if (behavior != newBehavior) {
            behavior = newBehavior;

            // Handle behavior transitions
            if (newBehavior == Behavior.IDLE) {
                // Abort any ongoing launch sequence
                releaseResources();
                state = LaunchState.IDLE;
                fireRequested = false;
            } else if (newBehavior == Behavior.SPINNING) {
                // Start spinning up from IDLE or IDLE_SPIN
                if (state == LaunchState.IDLE || state == LaunchState.IDLE_SPIN) {
                    state = LaunchState.SPINNING_UP;
                    shotNumber = 0;  // Reset shot counter for new sequence
                }
            }
        }
    }

    /**
     * Get the current behavior.
     */
    public Behavior getBehavior() {
        return behavior;
    }

    // ==================== RESOURCE MANAGEMENT ====================

    /**
     * Claim shared resources (belt and intake suppression).
     * Called when entering a firing state.
     */
    private void claimResources() {
        if (!resourcesClaimed) {
            if (loader != null) {
                loader.claimBeltForLauncher();
            }
            if (intake != null) {
                intake.startLaunchAssist();
            }
            resourcesClaimed = true;
        }
    }

    /**
     * Release shared resources (belt and intake).
     * Called when exiting firing states.
     */
    private void releaseResources() {
        if (resourcesClaimed) {
            if (loader != null) {
                loader.releaseBeltFromLauncher();
            }
            if (intake != null) {
                intake.stopLaunchAssist();
            }
            resourcesClaimed = false;
        }
    }

    // ==================== THREE-PHASE METHODS ====================

    @Override
    public void readSensors() {
        // PHASE 1: Motor velocity is SDK bulk-cached, no I2C read needed
        // We read currentSpeed in calc() from the bulk cache
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        // PHASE 2: Read cached velocity and run state machine


        pidNew = new PIDFCoefficients(NEW_P,NEW_I,NEW_D,NEW_F);
        flywheel.setVelocityPIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        flywheelHelp.setVelocityPIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
//        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
//        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        pidfModified = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Get flywheel speed (bulk-cached by SDK)
        currentSpeed = flywheel.getVelocity(AngleUnit.DEGREES);
        helperSpeed = flywheelHelp.getVelocity(AngleUnit.DEGREES);

        // CSV logging for debugging flywheel behavior
        if (LOGGING_ENABLED) {
            if (behavior == Behavior.SPINNING && logStartTime == 0) {
                // Start new log session
                flywheelLog = new CsvLogKeeper("flywheel_log", 6,
                    "elapsedMs,state,primarySpeed,helperSpeed,targetSpeed,speedDiff,fireRequested,isAtSpeed");
                logStartTime = System.currentTimeMillis();
            }
            if (logStartTime > 0 && behavior == Behavior.SPINNING) {
                // Log this cycle
                ArrayList<Object> row = new ArrayList<>();
                row.add(System.currentTimeMillis() - logStartTime);
                row.add(state.name());
                row.add(String.format("%.1f", currentSpeed));
                row.add(String.format("%.1f", helperSpeed));
                row.add(String.format("%.1f", targetSpeed));
                row.add(String.format("%.1f", targetSpeed - helperSpeed));      //difference between motors
                row.add(fireRequested);
                row.add(isFlywheelAtSpeed());
                flywheelLog.UpdateLog(row);
            }
            if (behavior == Behavior.IDLE && logStartTime > 0) {
                // End log session
                flywheelLog.CloseLog();
                logStartTime = 0;
                flywheelLog = null;
            }
        }

        // Hardcode target speed until botpose/distance calculation is calibrated
        // TODO: Re-enable vision-based speed calculation once Limelight is working
        //targetSpeed = MIN_LAUNCH_SPEED;

        // Sync behavior to state (if behavior changed externally)
        if (behavior == Behavior.IDLE && state != LaunchState.IDLE) {

                // Force back to idle and release resources
                releaseResources();
                state = LaunchState.IDLE;


        }
        else if(behavior == Behavior.SPINNING && state == LaunchState.MANUAL){
            state = LaunchState.MANUAL;
        }
        else if (behavior == Behavior.SPINNING && state == LaunchState.IDLE) {
            // Start spinning
            state = LaunchState.SPINNING_UP;
        }else if (behavior == Behavior.SPINNING && state == LaunchState.IDLE_SPIN) {
            // Start spinning
            state = LaunchState.SPINNING_UP;
        }
//        else if(behavior == Behavior.SPINNING && state == LaunchState.MANUAL){
//            state = LaunchState.MANUAL;
//        }

        // Run state machine
        //FOR PADDLE DESIGN
//        switch (state) {
//            case IDLE:
//                handleIdleState();
//                break;
//
//            case SPINNING_UP:
//                handleSpinningUpState();
//                break;
//
//            case READY:
//                handleReadyState();
//                break;
//
//            case BELT_REVERSING:
//                handleBeltReversingState();
//                break;
//
//            case FIRING:
//                handleFiringState();
//                break;
//
//            case COOLDOWN:
//                handleCooldownState();
//                break;
//            case MANUAL:
//                handleManualFlyWheel();
//                break;
//        }
        // Run simplified state machine for TPU ramp design
        switch(state) {
            case IDLE_SPIN:
                handleIdleSpinState();
                break;
            case IDLE:
                handleIdleState();
                break;
            case SPINNING_UP:
                handleSpinningUpState();
                break;
            case READY:
                handleReadyState();
                break;
            case FIRING:
                handleFiringState();
                break;
            case LIFTING:
                handleLiftingState();
                break;
            case COMPLETE:
                handleCompleteState();
                break;
            case MANUAL:
                handleManualState();
                break;
        }
    }

    @Override
    public void act() {
        // PHASE 3: Flush paddle servo command
        paddle.flush();
    }

    // ==================== TRIGGER HELPERS ====================

    /**
     * Get the idle/resting position for the current trigger type.
     */
    private double getTriggerIdlePosition() {
        switch (TRIGGER_TYPE) {
            case STAR:      return STAR_IDLE;
            case STAR_POSE: return servoNormalize(STAR_REST);
            case PADDLE:    return PADDLE_DOWN;
            case RAMP:
            case GATE:      return servoNormalize(GATE_CLOSE);
            default:        return PADDLE_CUP;
        }
    }

    public void changeStar(){
        if(STAR_FIRED>=3){
            STAR_FIRED=0;
            return;
        }
        STAR_FIRED++;
    }

    /**
     * Get the firing/feeding position for the current trigger type.
     */
    private double getTriggerFiringPosition() {
        switch (TRIGGER_TYPE) {
            case STAR:   return STAR_FEEDING;
            case STAR_POSE:
                switch (STAR_FIRED) {
                    case 0:  return servoNormalize(STAR_REST);
                    case 1:  return servoNormalize(STAR_LAUNCH_1);
                    case 2:  return servoNormalize(STAR_LAUNCH_2);
                    case 3:  return servoNormalize(STAR_LAUNCH_3);
                    default: return servoNormalize(STAR_REST);
                }
            case PADDLE: return PADDLE_UP;
            case RAMP:
            case GATE:  return servoNormalize(GATE_OPEN);
            default:     return PADDLE_RAMP;
        }
    }

    public void paddleCup() {
        setPaddlePosition(PADDLE_CUP);
    }

    public void paddleRamp() {
        setPaddlePosition(PADDLE_RAMP);
    }

    public void paddleLift() {
        setPaddlePosition(PADDLE_LIFT);
    }

    /**
     * Enter manual mode for testing paddle and flywheel.
     */
    public void requestManual() {
        state = LaunchState.MANUAL;
    }

    // ==================== STATE HANDLERS ====================

    /**
     * IDLE: Flywheel off, paddle in CUP position.
     */
    private void handleIdleState() {
        releaseResources();
        flywheel.setVelocity(0);
        flywheelHelp.setVelocity(0);
        //setPaddlePosition(passThroughMode ? getTriggerFiringPosition() : getTriggerIdlePosition());
        setPaddlePosition(getTriggerIdlePosition());
    }

    private void handleIdleSpinState() {
        releaseResources();
        flywheel.setVelocity(FLYWHEEL_IDLE_SPEED);
        flywheelHelp.setVelocity(FLYWHEEL_IDLE_SPEED);
        //setPaddlePosition(passThroughMode ? getTriggerFiringPosition() : getTriggerIdlePosition());
        setPaddlePosition(getTriggerIdlePosition());
    }


    /**
     * Update target speed from vision distance calculation.
     * Falls back to MIN_LAUNCH_SPEED if vision doesn't have a valid pose
     * (e.g., sun washing out the camera).
     */
    public void updateTargetSpeed(){
        if (vision != null && vision.hasBotPose()) {
            targetSpeed = vision.getFlywheelSpeed() * SPEED_MULTIPLIER;
        } else {
            targetSpeed = MIN_LAUNCH_SPEED * SPEED_MULTIPLIER;
        }
    }
    private void handleSpinningUpState() {

        flywheel.setVelocity(targetSpeed, AngleUnit.DEGREES);
        flywheelHelp.setVelocity(targetSpeed, AngleUnit.DEGREES);
        setPaddlePosition(getTriggerIdlePosition());

        if(TRIGGER_TYPE == TriggerType.GATE) {
            if (isFlywheelAtSpeed() && turret.isReadyToLaunch()) {
                state = LaunchState.READY;
            }
        }else{
            if (isFlywheelAtSpeed()) {
                state = LaunchState.READY;
            }
        }
    }

    /**
     * READY: Flywheel at speed, paddle in CUP, waiting for fire() command.
     */
    private void handleReadyState() {
        flywheel.setVelocity(targetSpeed, AngleUnit.DEGREES);
        flywheelHelp.setVelocity(targetSpeed, AngleUnit.DEGREES);
        setPaddlePosition(getTriggerIdlePosition());

        if (fireRequested) {
            shotNumber++;
            speedAtFireApproval = currentSpeed;
            fireRequested = false;

            // Claim resources and start firing
            claimResources();
            state = LaunchState.FIRING;
            stateTimer = futureTime(FIRING_TIME);
            if (TRIGGER_TYPE == TriggerType.STAR_POSE) {
                STAR_FIRED = 0;  // Reset to REST before firing sequence
            }
            launchSpacerTimer = futureTime(LAUNCH_SPACER_TIMER);
        }
    }

    /**
     * FIRING: Paddle at RAMP position, conveyor pushing balls through flywheel.
     * Transitions to LIFTING when timeout expires OR sensor shows balls remain.
     */
    private void handleFiringState() {
        flywheel.setVelocity(targetSpeed, AngleUnit.DEGREES);
        flywheelHelp.setVelocity(targetSpeed, AngleUnit.DEGREES);

        // STAR_POSE: step through discrete positions when spacer timer elapsed AND flywheel at speed
        // Both conditions required: timer gives conveyor time to load, speed check ensures
        // flywheel has recovered (compensates for low battery or lost belt on a motor)
        if (TRIGGER_TYPE == TriggerType.STAR_POSE) {
            if (isPast(launchSpacerTimer) && isFlywheelAtSpeed()) {
                STAR_FIRED++;
                if (STAR_FIRED > 3) {
                    state = LaunchState.COMPLETE;
                    return;
                }
                // Use longer timer for last ball: loading (pos 2) and firing dwell (pos 3)
                // Nothing pushing it from behind on conveyor, needs more time to seat and fire
                double spacer = (STAR_FIRED >= 2) ? LAUNCH_SPACER_TIMER_LAST : LAUNCH_SPACER_TIMER;
                launchSpacerTimer = futureTime(spacer);
            }
            setPaddlePosition(getTriggerFiringPosition());
            return;
        }

        // Other trigger types: timeout-based firing
        setPaddlePosition(getTriggerFiringPosition());

        if (isPast(stateTimer)) {
            if (TRIGGER_TYPE == TriggerType.STAR || TRIGGER_TYPE == TriggerType.PADDLE || TRIGGER_TYPE == TriggerType.GATE) {
                state = LaunchState.COMPLETE;
            } else {
                // Ramp needs LIFTING phase to push last ball into flywheel
                state = LaunchState.LIFTING;
                stateTimer = futureTime(LIFT_TIME);
            }
        }
    }

    /**
     * LIFTING: Paddle at LIFT position to push last ball into flywheel contact.
     */
    private void handleLiftingState() {
        flywheel.setVelocity(targetSpeed, AngleUnit.DEGREES);
        flywheelHelp.setVelocity(targetSpeed, AngleUnit.DEGREES);
        setPaddlePosition(PADDLE_LIFT);

        if (isPast(stateTimer)) {
            state = LaunchState.COMPLETE;
        }
    }

    /**
     * COMPLETE: Firing sequence done. Transition based on STAY_SPINNING_AFTER_FIRE.
     */
    private void handleCompleteState() {
        releaseResources();
        setPaddlePosition(getTriggerIdlePosition());

        if (STAY_SPINNING_AFTER_FIRE) {
            // Keep flywheel at idle spin speed for faster follow-up
            state = LaunchState.IDLE_SPIN;
        } else {
            // Spin down and return to IDLE
            behavior = Behavior.IDLE;
            state = LaunchState.IDLE;
        }
        loader.loaderFull = false;
    }

    /**
     * MANUAL: For testing - flywheel at target speed, paddle controlled externally.
     */
    private void handleManualState() {
        flywheel.setVelocity(targetSpeed, AngleUnit.DEGREES);
        flywheelHelp.setVelocity(targetSpeed, AngleUnit.DEGREES);
        // Paddle position controlled via paddleCup()/paddleRamp()/paddleLift()
    }


    private void setPaddlePosition(double position) {
        paddle.setPosition(position);  // Queues position, flushed in act()
    }

    /**
     * Check if flywheel is at target speed.
     */
    public boolean isFlywheelAtSpeed() {
        return currentSpeed >= (targetSpeed - SPEED_TOLERANCE);
    }

    /**
     * Prepare launcher for firing at a given distance.
     * @deprecated Use setBehavior(Behavior.SPINNING) instead. Distance is auto-pulled from Vision.
     */
    @Deprecated
    public void prepareToLaunch(double distanceMeters) {
        targetSpeed = calculateLaunchSpeed(distanceMeters);
        setBehavior(Behavior.SPINNING);
    }

    /**
     * Prepare launcher with a specific speed (for manual testing).
     * @deprecated Use setBehavior(Behavior.SPINNING) instead. Set MIN_LAUNCH_SPEED if needed.
     */
    @Deprecated
    public void prepareToLaunchAtSpeed(double speed) {
        targetSpeed = speed;
        setBehavior(Behavior.SPINNING);
    }

    /**
     * Fire the ball (only works when in READY state).
     * Behavior must be SPINNING and isReady() must return true.
     */
    public void fire() {
        if (state == LaunchState.READY) {
            fireRequested = true;
        }
    }

    /**
     * Abort launch and return to idle.
     * Equivalent to setBehavior(Behavior.IDLE).
     */
    public void abort() {
        STAR_FIRED = 0;  // Return star to REST/intake position
        setBehavior(Behavior.IDLE);
    }

    /**
     * Check if launcher is ready to fire.
     */
    public boolean isReady() {
        return state == LaunchState.READY;
    }

    /**
     * Check if launcher is currently in a launch sequence.
     */
    public boolean isBusy() {
        return state != LaunchState.IDLE && state != LaunchState.READY;
    }

    // ==================== HEALTH CHECK ACCESSORS ====================

    /** Get primary flywheel encoder position. */
    public int getFlywheelEncoder() {
        return flywheel.getCurrentPosition();
    }

    /** Get helper flywheel encoder position. */
    public int getHelperEncoder() {
        return flywheelHelp.getCurrentPosition();
    }

    /** Get primary flywheel current draw (amps). I2C call. */
    public double getFlywheelCurrent() {
        return flywheel.getCurrent(CurrentUnit.AMPS);
    }

    /** Get helper flywheel current draw (amps). I2C call. */
    public double getHelperCurrent() {
        return flywheelHelp.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * Enable pass-through mode (paddle at RAMP position).
     * Used for passing balls to partner robot.
     */
    public void setPassThroughMode(boolean enabled) {
        passThroughMode = enabled;
        if (enabled && state == LaunchState.IDLE) {
            setPaddlePosition(getTriggerFiringPosition());
        }
    }

    /**
     * Calculate launch speed based on distance using physics model.
     * Based on projectile motion equations.
     *
     * @param distanceMeters Horizontal distance to target
     * @return Required flywheel speed in degrees/sec
     */
    public double calculateLaunchSpeed(double distanceMeters) {
        double d = distanceMeters + LIMELIGHT_OFFSET;
        double theta = Math.toRadians(LAUNCH_ANGLE);
        double h = TARGET_HEIGHT;

        // Projectile motion: d = v*cos(θ)*t, h = v*sin(θ)*t - 0.5*g*t²
        // Solve for v given d, h, θ
        double tanTheta = Math.tan(theta);
        double cosTheta = Math.cos(theta);

        // Time of flight from vertical motion
        double discriminant = (tanTheta * d - h) / 4.905;
        if (discriminant < 0) discriminant = 0.1; // Fallback

        double time = Math.sqrt(discriminant);

        // Required velocity
        double velocity = d / (cosTheta * time);

        // Convert to flywheel speed (degrees/sec)
        // This factor depends on flywheel diameter and motor characteristics
        double flywheelSpeed = velocity * 1090 / 6.67;

        // Add safety margin
        return flywheelSpeed + 50;
    }

    /**
     * Get current launch state.
     */
    public LaunchState getState() {
        return state;
    }

    /**
     * Get current flywheel speed.
     */
    public double getCurrentSpeed() {
        return currentSpeed;
    }

    /**
     * Get target flywheel speed.
     */
    public double getTargetSpeed() {
        return targetSpeed;
    }

    @Override
    public void stop() {
        behavior = Behavior.IDLE;
        releaseResources();
        state = LaunchState.IDLE;
        fireRequested = false;
        STAR_FIRED = 0;  // Return star to REST/intake position
        flywheel.setVelocity(0);
        flywheelHelp.setVelocity(0);
        paddle.setPosition(getTriggerIdlePosition());
        paddle.flush();  // Immediate write for emergency stop
    }

    @Override
    public void resetStates() {
        behavior = Behavior.IDLE;
        releaseResources();
        state = LaunchState.IDLE;
        fireRequested = false;
        passThroughMode = false;
        STAR_FIRED = 0;  // Return star to REST/intake position
    }

    @Override
    public String getTelemetryName() {
        return "Launcher";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Trigger", TRIGGER_TYPE);
        telemetry.put("Star Fired", STAR_FIRED);
        telemetry.put("Behavior", behavior);
        telemetry.put("Launch State: ", state);
        telemetry.put("Flywheel Speed", String.format("%.0f / %.0f", currentSpeed, targetSpeed));
        telemetry.put("flywheelSpeed", currentSpeed);
        telemetry.put("flywheelTarget", targetSpeed);
        telemetry.put("Ready", isReady() ? "YES" : "no");

        telemetry.put("P, I, D, F (orig)", String.format("%.04f, %.04f, %.04f, %.04f", pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f));
        telemetry.put("algorithm", pidfOrig.algorithm);
        if(pidfModified!=null){
            telemetry.put("P, I, D, F (modified)", String.format("%.04f, %.04f, %.04f, %.04f", pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f));
        }


        if (debug) {
            telemetry.put("Internal State", state);
            telemetry.put("Primary Speed", String.format("%.0f",currentSpeed));
            telemetry.put("Helper Speed", String.format("%.0f",helperSpeed));
            telemetry.put("Motor Diff", String.format("%.0f",currentSpeed-helperSpeed));
            telemetry.put("flywheelAmps", flywheel.getCurrent(CurrentUnit.AMPS));
            telemetry.put("helperAmps", flywheelHelp.getCurrent(CurrentUnit.AMPS));
            telemetry.put("flywheelEncoder", flywheel.getCurrentPosition());
            telemetry.put("helperEncoder", flywheelHelp.getCurrentPosition());
            telemetry.put("At Speed", isFlywheelAtSpeed() ? "YES" : "no");
            telemetry.put("Shot #", shotNumber);
            telemetry.put("Speed@Approval", String.format("%.0f", speedAtFireApproval));
            telemetry.put("Resources Claimed", resourcesClaimed);
            telemetry.put("Paddle Pos", paddle.getPendingPosition());
            telemetry.put("Pass-Through", passThroughMode ? "ON" : "off");
            telemetry.put("Fire Requested", fireRequested);
        }

        return telemetry;
    }
    public static double servoNormalize(int pulse) {
        double normalized = (double) pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }
}
