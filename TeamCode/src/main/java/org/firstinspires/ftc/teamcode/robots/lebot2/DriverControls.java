package org.firstinspires.ftc.teamcode.robots.lebot2;

import static org.firstinspires.ftc.teamcode.robots.lebot2.Lebot2_6832.gameState;
import static org.firstinspires.ftc.teamcode.robots.lebot2.Lebot2_6832.gameStateIndex;
import static org.firstinspires.ftc.teamcode.robots.lebot2.Lebot2_6832.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Launcher;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Loader;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.TelemetryProvider;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Driver controls handler for Lebot2.
 *
 * Maps gamepad inputs to robot actions. Separating this from the Robot class
 * keeps the control logic clean and makes it easy to modify button mappings.
 *
 * ARCHITECTURE:
 * - Drive is ALWAYS called (never blocked by other subsystems)
 * - Drive input automatically interrupts RR trajectories and PID turns
 * - Intake, loader, launcher operate independently via their own behaviors
 * - Robot articulations only used for multi-subsystem coordination
 *
 * Uses StickyGamepad for edge detection (button press vs button held).
 */
@Config(value = "Lebot2_DriverControls")
public class DriverControls implements TelemetryProvider {

    // Configuration
    public static double DRIVE_DAMPENER = 0.7;
    public static double SLOW_MODE_DAMPENER = 0.3;
    public static boolean slowMode = false;

    // Gamepads
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final StickyGamepad stickyGamepad1;
    private final StickyGamepad stickyGamepad2;
    private int tiltIndex=0;

    public DriverControls(Gamepad pad1, Gamepad pad2) {
        this.gamepad1 = pad1;
        this.gamepad2 = pad2;
        this.stickyGamepad1 = new StickyGamepad(pad1);
        this.stickyGamepad2 = new StickyGamepad(pad2);
    }

    /**
     * Update sticky gamepads - call once per loop.
     */
    public void updateStickyGamepads() {
        stickyGamepad1.update();
        stickyGamepad2.update();
    }

    /**
     * Handle controls during init_loop (before match starts).
     */
    public void initLoop() {
        updateStickyGamepads();
        handleGameStateSwitch();
        handleAllianceSelection();
        handleStartingPositionSelection();
        joystickDrive();
    }

    /**
     * Handle main driving controls during teleop.
     */
    void joystickDrive() {
        boolean tank = false; // set true to drive motors independently for true tank drive

        if(tank){
            double left= gamepad1.left_stick_y;
            double right = gamepad1.right_stick_y;
            robot.driveTrain.drive(left,right,0);
        }else {

            // Get drive inputs
            double throttle = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // Abort any running mission if driver provides significant input
            if (robot.missions.isActive() && (Math.abs(throttle) > 0.1 || Math.abs(turn) > 0.1)) {
                robot.missions.abort();
            }

            // Apply dampening
            double dampener = slowMode ? SLOW_MODE_DAMPENER : DRIVE_DAMPENER;
            turn *= dampener;

            // ALWAYS call drive - the drivetrain handles mode switching internally
            // If joystick input is significant, it will automatically interrupt
            // any running RR trajectory or PID turn
            robot.driveTrain.drive(throttle, 0, turn);

        }
    }

    public void handleButtons() {
        // A button: Toggle intake LOAD_ALL behavior
        // Intake runs until loader is full, then auto-stops
        if (stickyGamepad1.a) {
            if (robot.intake.isActive()) {
                robot.intake.off();
                robot.loader.releaseBeltFromIntake();
            } else {
                robot.intake.loadAll();
                robot.loader.requestBeltForIntake();
            }
        }

        // B button: Cancel/stop all subsystem behaviors
        if (stickyGamepad1.b) {
            robot.setBehavior(Robot.Behavior.MANUAL);
            robot.launcher.setBehavior(Launcher.Behavior.IDLE);
            robot.intake.off();
            robot.loader.releaseBelt();
        }

        if(stickyGamepad1.x){
            robot.launcher.requestManual();
            robot.launcher.setBehavior(Launcher.Behavior.SPINNING);
        }

//        // X button: Spin up launcher / fire
//        // Launcher will claim belt when actually firing
//        if (stickyGamepad1.x) {
//            if (robot.launcher.isReady()) {
//                // If already ready, fire one ball
//                robot.launcher.fire();
//            } else if (robot.launcher.getBehavior() == Launcher.Behavior.IDLE) {
//                // Start spinning up - launcher pulls distance from Vision automatically
//                robot.launcher.setBehavior(Launcher.Behavior.SPINNING);
//            }
//        }

        // Y button: Center on vision target (runs to completion)
        // DriveTrain queries Vision directly for continuous tx updates
        // Driver can override with joystick if needed
        if (stickyGamepad1.y) {
            robot.driveTrain.centerOnTarget();
        }

        // Left bumper: Toggle slow mode
        if (stickyGamepad1.left_bumper) {
            slowMode = !slowMode;
        }

        // Right bumper: Launch all balls in sequence
        if (stickyGamepad1.right_bumper) {
            robot.setBehavior(Robot.Behavior.LAUNCH_ALL);
        }

        // D-pad up/down: Manual paddle control (CUP/RAMP positions)
        if (stickyGamepad1.dpad_up) {
            robot.launcher.paddleRamp();
            robot.launcher.setPassThroughMode(true);
        }
        if (stickyGamepad1.dpad_down) {
            robot.launcher.paddleCup();
            robot.launcher.setPassThroughMode(false);
        }

        // D-pad left: Simple intake on (not LOAD_ALL)
        if (stickyGamepad1.dpad_left) {
            robot.intake.on();
            robot.loader.requestBeltForIntake();
        }

        // D-pad right: Eject balls
        if (stickyGamepad1.dpad_right) {
            //robot.launcher.manualFire();
            robot.intake.eject();         //bring back after manual test
            // For eject, we use setBeltPower which claims as launcher priority
            // Positive = eject forward
            robot.loader.setBeltPower(0.5);       //bring back after manual test
        }

        // Guide button: Reset encoders and ball count
        if (stickyGamepad1.guide) {
            robot.driveTrain.resetEncoders();
            robot.loader.resetBallCount();
        }

        // Back button: Apply vision pose correction to Pinpoint
        // Only works when vision has a valid botpose (facing AprilTag)
        if (stickyGamepad1.back && robot.canApplyVisionCorrection()) {
            boolean applied = robot.applyVisionPoseCorrection();
            if (applied) {
                // Brief rumble feedback to confirm correction was applied
                gamepad1.rumble(100);
            }
        }

        //Start button: change tilt index which correspond to servo tilt of limelight
        if(stickyGamepad1.start){
            if(tiltIndex!=3){
                tiltIndex++;
            }
            else{
                tiltIndex=0;
            }
        }
        if(tiltIndex==0){
            robot.vision.setTiltDown();
        } else if (tiltIndex==1) {
            robot.vision.setTiltStraight();
        } else if (tiltIndex==2) {
            robot.vision.setTiltUpMin();
        }else{
            robot.vision.setTiltUpMax();
        }

    }

    /**
     * Handle tuning mission controls in TEST mode.
     * Uses gamepad1 - safe because handleButtons() is NOT called in TEST mode.
     * Called from Lebot2_6832.handleTest().
     *
     * Gamepad1 buttons (TEST mode only):
     *   A = Start Rotation Test (90° turns CW/CCW)
     *   B = Start Square Test (drive 24", turn 90°, repeat 4x)
     *   X = Start Straight Line Test (48" forward and back)
     *   Y = Start Turn Accuracy Test (45°, 90°, 180° turns)
     *   RB = Start Ramsete Test (trajectory with heading disturbance)
     *   Back = Abort current tuning mission
     */
    public void handleTuningControls() {
        // Only allow starting new missions if none running
        if (!robot.missions.isActive()) {
            if (stickyGamepad1.a) {
                robot.missions.initLogging();  // Ensure logging is on
                robot.missions.startTuningRotation();
            }
            if (stickyGamepad1.b) {
                robot.missions.initLogging();
                robot.missions.startTuningSquare();
            }
            if (stickyGamepad1.x) {
                robot.missions.initLogging();
                robot.missions.startTuningStraight();
            }
            if (stickyGamepad1.y) {
                robot.missions.initLogging();
                robot.missions.startTuningTurn();
            }
            if (stickyGamepad1.right_bumper) {
                robot.missions.initLogging();
                robot.missions.startTuningRamsete();
            }
        }

        // Back button aborts current mission
        if (stickyGamepad1.back && robot.missions.isActive()) {
            robot.missions.abort();
        }
    }

    public int getTiltIndex(){return tiltIndex;}

    /**
     * Handle game state switching during init.
     */
    private void handleGameStateSwitch() {
        // Right trigger: Cycle through game states
        if (gamepad1.right_trigger > 0.5) {
            gameStateIndex = (gameStateIndex + 1) % Lebot2_6832.GameState.getNumGameStates();
            gameState = Lebot2_6832.GameState.getGameState(gameStateIndex);
            // Debounce
            try { Thread.sleep(200); } catch (InterruptedException ignored) {}
        }
    }

    /**
     * Handle alliance selection during init.
     */
    private void handleAllianceSelection() {
        // X button: Blue alliance
        if (stickyGamepad1.x) {
            Robot.isRedAlliance = false;
            robot.setAlliance(false);
        }

        // B button: Red alliance
        if (stickyGamepad1.b) {
            Robot.isRedAlliance = true;
            robot.setAlliance(true);
        }
    }

    /**
     * Handle starting position selection during init.
     *
     * Starting positions determine initial Pinpoint pose:
     * - AUDIENCE: Touching audience wall, can see goal AprilTag (long range)
     * - GOAL: Touching goal wall, too close for vision initially
     * - UNKNOWN: For teleop testing, position unknown until first vision fix
     * - CALIBRATION: Field center, facing red goal - for MT1/MT2 comparison
     *
     * Blue positions are reflections of Red positions across field X axis.
     */
    private void handleStartingPositionSelection() {
        // A button: Audience wall position
        if (stickyGamepad1.a) {
            robot.setStartingPosition(Robot.StartingPosition.AUDIENCE);
        }

        // Y button: Goal wall position
        if (stickyGamepad1.y) {
            robot.setStartingPosition(Robot.StartingPosition.GOAL);
        }

        // Back button: Unknown position (teleop and relocalization testing)
        if (stickyGamepad1.back) {
            robot.setStartingPosition(Robot.StartingPosition.UNKNOWN);
        }

        // Guide button: Calibration position (field center, facing red goal)
        // Used for MT1/MT2 pose comparison testing
        if (stickyGamepad1.guide) {
            robot.setStartingPosition(Robot.StartingPosition.CALIBRATION);
        }
    }

    @Override
    public String getTelemetryName() {
        return "Robot";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        return telemetry;
    }
}
