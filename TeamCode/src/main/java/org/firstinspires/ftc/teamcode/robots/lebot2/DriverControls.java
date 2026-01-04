package org.firstinspires.ftc.teamcode.robots.lebot2;

import static org.firstinspires.ftc.teamcode.robots.lebot2.Lebot2_6832.gameState;
import static org.firstinspires.ftc.teamcode.robots.lebot2.Lebot2_6832.gameStateIndex;
import static org.firstinspires.ftc.teamcode.robots.lebot2.Lebot2_6832.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;
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
        // Cancel any articulation if driver moves stick
        if (hasSignificantInput()) {
            if (robot.getArticulation() != Robot.Articulation.MANUAL &&
                robot.getArticulation() != Robot.Articulation.INTAKE) {
                robot.articulate(Robot.Articulation.MANUAL);
            }
        }

        // Get drive inputs
        double throttle = -gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;

        // Apply dampening
        double dampener = slowMode ? SLOW_MODE_DAMPENER : DRIVE_DAMPENER;
        turn *= dampener;

        // Drive (only if in manual or transit mode)
        if (robot.getArticulation() == Robot.Articulation.MANUAL ||
            robot.getArticulation() == Robot.Articulation.TRANSIT) {
            robot.driveTrain.drive(throttle, 0, turn);
        }

        if (turning()){
            robot.driveTrain.drive(0,0, turn);
        }

        // Handle button inputs
        handleButtons();
    }

    private boolean turning(){
        return (Math.abs(gamepad1.right_stick_x) > 0.3);
    }

    private boolean hasSignificantInput() {
        return Math.abs(gamepad1.left_stick_y) > 0.3 ||
               Math.abs(gamepad1.right_stick_x) > 0.3 ||
               Math.abs(gamepad1.left_stick_x) > 0.3;
    }

    private void handleButtons() {
        // A button: Start/stop intake
        if (stickyGamepad1.a) {
            if (robot.getArticulation() == Robot.Articulation.INTAKE) {
                robot.articulate(Robot.Articulation.MANUAL);
                robot.intake.off();
                robot.loader.stopBelt();
            } else {
                robot.articulate(Robot.Articulation.INTAKE);
            }
        }

        // B button: Cancel/stop everything
        if (stickyGamepad1.b) {
            robot.articulate(Robot.Articulation.MANUAL);
            robot.launcher.abort();
            robot.intake.off();
            robot.loader.stopBelt();
        }

        // X button: Launch single ball
        if (stickyGamepad1.x) {
            robot.articulate(Robot.Articulation.LAUNCHING);
        }

        // Y button: Targeting mode (turn to target)
        if (stickyGamepad1.y) {
            if (robot.vision.hasTarget()) {
                robot.driveTrain.turnToTarget(robot.vision.getTx(), 0.5);
            }
        }

        // Left bumper: Toggle slow mode
        if (stickyGamepad1.left_bumper) {
            slowMode = !slowMode;
        }

        // Right bumper: Launch all balls
        if (stickyGamepad1.right_bumper) {
            robot.articulate(Robot.Articulation.LAUNCH_ALL);
        }

        // D-pad up/down: Manual paddle control
        if (stickyGamepad1.dpad_up) {
            robot.launcher.setPassThroughMode(true);
        }
        if (stickyGamepad1.dpad_down) {
            robot.launcher.setPassThroughMode(false);
        }

        // D-pad left: Manual intake on
        if (stickyGamepad1.dpad_left) {
            robot.intake.on();
            robot.loader.runBelt();
        }

        // D-pad right: Eject balls
        if (stickyGamepad1.dpad_right) {
            robot.intake.eject();
            robot.loader.setBeltPower(-0.5);
        }

        // Guide button: Reset encoders/IMU
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
