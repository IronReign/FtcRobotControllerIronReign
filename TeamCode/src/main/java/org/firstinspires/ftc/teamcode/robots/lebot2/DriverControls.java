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
    }

    /**
     * Handle main driving controls during teleop.
     */
    public void joystickDrive() {
        // Get drive inputs
        double throttle = -gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;

        // Apply dampening
        double dampener = slowMode ? SLOW_MODE_DAMPENER : DRIVE_DAMPENER;
        turn *= dampener;

        // ALWAYS call drive - the drivetrain handles mode switching internally
        // If joystick input is significant, it will automatically interrupt
        // any running RR trajectory or PID turn
        robot.driveTrain.drive(throttle, 0, turn);

        // Handle button inputs
        handleButtons();
    }

    private void handleButtons() {
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
            robot.loader.stopBelt();
        }

        // X button: Spin up launcher / fire
        // Launcher will claim belt when actually firing
        if (stickyGamepad1.x) {
            if (robot.launcher.isReady()) {
                // If already ready, fire one ball
                robot.launcher.fire();
            } else if (robot.launcher.getBehavior() == Launcher.Behavior.IDLE) {
                // Start spinning up - launcher pulls distance from Vision automatically
                robot.launcher.setBehavior(Launcher.Behavior.SPINNING);
            }
        }

        // Y button: Turn to vision target
        // Driver can override with joystick if vision glares
        if (stickyGamepad1.y) {
            if (robot.vision.hasTarget()) {
                robot.driveTrain.turnToTarget(robot.vision.getTx(), 0.5);
            }
        }

        // Left bumper: Toggle slow mode
        if (stickyGamepad1.left_bumper) {
            slowMode = !slowMode;
        }

        // Right bumper: Launch all balls in sequence
        if (stickyGamepad1.right_bumper) {
            robot.setBehavior(Robot.Behavior.LAUNCH_ALL);
        }

        // D-pad up/down: Manual paddle control
        if (stickyGamepad1.dpad_up) {
            robot.launcher.setPassThroughMode(true);
        }
        if (stickyGamepad1.dpad_down) {
            robot.launcher.setPassThroughMode(false);
        }

        // D-pad left: Simple intake on (not LOAD_ALL)
        if (stickyGamepad1.dpad_left) {
            robot.intake.on();
            robot.loader.requestBeltForIntake();
        }

        // D-pad right: Eject balls
        if (stickyGamepad1.dpad_right) {
            robot.intake.eject();
            // For eject, we use setBeltPower which claims as launcher priority
            // Positive = eject forward
            robot.loader.setBeltPower(0.5);
        }

        // Guide button: Reset encoders and ball count
        if (stickyGamepad1.guide) {
            robot.driveTrain.resetEncoders();
            robot.loader.resetBallCount();
        }
    }

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
