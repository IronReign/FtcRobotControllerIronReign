package org.firstinspires.ftc.teamcode.robots.lebot2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.drivetrain.TankDrive;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.TelemetryProvider;

import java.util.Map;

/**
 * Unified OpMode for Lebot2.
 *
 * This single OpMode handles both Autonomous and TeleOp modes.
 * The game state can be switched during init_loop, allowing:
 * - Testing autonomous routines
 * - Driver override during autonomous testing
 * - Consistent initialization across all modes
 *
 * This pattern is used by Iron Reign's competition robots and provides
 * flexibility without code duplication.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Lebot2_6832", group = "Lebot2")
@Config(value = "Lebot2_OpMode")
public class Lebot2_6832 extends OpMode {

    // ==================== COMPONENTS ====================
    public static Robot robot;
    public static DriverControls driverControls;
    private FtcDashboard dashboard;
    private Autonomous autonomous;

    // ==================== GAME STATES ====================
    public enum GameState {
        AUTONOMOUS("Autonomous", true),
        TELE_OP("Tele-Op", false),
        TEST("Test", false);

        private final String name;
        private final boolean isAutonomous;

        GameState(String name, boolean isAutonomous) {
            this.name = name;
            this.isAutonomous = isAutonomous;
        }

        public String getName() {
            return name;
        }

        public boolean isAutonomous() {
            return isAutonomous;
        }

        public static GameState getGameState(int index) {
            switch (index) {
                case 0: return AUTONOMOUS;
                case 1: return TELE_OP;
                case 2: return TEST;
                default: return TELE_OP;
            }
        }

        public static int getNumGameStates() {
            return 3;
        }
    }

    public static GameState gameState = GameState.TELE_OP;
    public static int gameStateIndex = 1;

    // ==================== CONFIGURATION ====================
    public static boolean debugTelemetryEnabled = false;

    // ==================== TIMING ====================
    private long startTime;
    private long loopCount = 0;
    private long lastLoopTime;
    private double averageLoopTime;
    private static final double LOOP_TIME_SMOOTHING = 0.1;

    // ==================== LIFECYCLE METHODS ====================

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize dashboard
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(250);

        // Initialize robot
        robot = new Robot(hardwareMap, false);
        driverControls = new DriverControls(gamepad1, gamepad2);
        
        // Create Auton
        autonomous = new Autonomous(robot);

        // Set telemetry transmission rate
        telemetry.setMsTransmissionInterval(250);

        telemetry.addData("Status", "Initialized. Use triggers to change mode.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        TelemetryPacket packet = new TelemetryPacket();

        // Handle pre-match controls
        driverControls.initLoop();

        // Update robot (for any calibration routines)
        robot.update(packet.fieldOverlay());

        // Telemetry
        telemetry.addData("Game State", gameState.getName());
        telemetry.addData("Alliance", Robot.isRedAlliance ? "RED" : "BLUE");
        telemetry.addLine();
        telemetry.addData("Controls", "X=Blue, B=Red, RT=Cycle Mode");

        handleTelemetry(robot.getTelemetry(false), robot.getTelemetryName(), packet);

        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        lastLoopTime = System.nanoTime();
        loopCount = 0;

        // Reset robot for match start
        robot.resetStates();

        // If starting in TeleOp, ensure manual control
        if (!gameState.isAutonomous()) {
            robot.articulate(Robot.Articulation.MANUAL);
        }
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        loopCount++;

        // Update timing
        long now = System.nanoTime();
        double loopTime = (now - lastLoopTime) / 1e6; // ms
        averageLoopTime = averageLoopTime * (1 - LOOP_TIME_SMOOTHING) + loopTime * LOOP_TIME_SMOOTHING;
        lastLoopTime = now;

        // Update sticky gamepads
        driverControls.updateStickyGamepads();

        // Handle game state
        switch (gameState) {
            case AUTONOMOUS:
                handleAutonomous(packet);
                break;

            case TELE_OP:
                handleTeleOp(packet);
                break;

            case TEST:
                handleTest(packet);
                break;
        }

        // Update robot
        robot.update(packet.fieldOverlay());

        // Telemetry
        buildTelemetry(packet);

        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        robot.stop();
        gameState = GameState.TELE_OP; // Reset for next run
    }

    // ==================== GAME STATE HANDLERS ====================

    private void handleAutonomous(TelemetryPacket packet) {
        // TODO: Implement autonomous routines
        // For now, just run a simple test sequence
        autonomous.execute();

        // Auto-transition to TeleOp after 30 seconds
        long elapsed = System.currentTimeMillis() - startTime;
        if (elapsed > 30000) {
            gameState = GameState.TELE_OP;
        }
    }

    private void handleTeleOp(TelemetryPacket packet) {
        driverControls.joystickDrive();
    }

    private void handleTest(TelemetryPacket packet) {
        // In test mode, enable debug telemetry and allow all controls
        debugTelemetryEnabled = true;
        driverControls.joystickDrive();
    }

    // ==================== TELEMETRY ====================

    private void buildTelemetry(TelemetryPacket packet) {
        // OpMode info
        telemetry.addData("Mode", gameState.getName());
        telemetry.addData("Loop Time", String.format("%.1f ms (%.0f Hz)",
                averageLoopTime, 1000 / averageLoopTime));
        telemetry.addLine();
        telemetry.addData("Forward/Backward", -gamepad1.left_stick_y);
        telemetry.addData("Left/Right", -gamepad1.right_stick_x);
        telemetry.addData("Launch All States", robot.getArticulation());
        telemetry.addData("Articulation", robot.launchSequenceState);

        // Robot telemetry
        handleTelemetry(robot.getTelemetry(debugTelemetryEnabled),
                robot.getTelemetryName(), packet);

        // Subsystem telemetry
        if (debugTelemetryEnabled) {
            for (Subsystem subsystem : robot.subsystems) {
                if (subsystem instanceof TelemetryProvider) {
                    TelemetryProvider provider = (TelemetryProvider) subsystem;
                    handleTelemetry(provider.getTelemetry(true),
                            provider.getTelemetryName(), packet);
                }
            }
        } else {
            // In non-debug mode, still show key subsystem info
            handleTelemetry(robot.launcher.getTelemetry(false), "Launcher", packet);
            handleTelemetry(robot.loader.getTelemetry(false), "Loader", packet);
        }
    }

    private void handleTelemetry(Map<String, Object> telemetryMap, String name, TelemetryPacket packet) {
        telemetry.addLine(name);
        packet.addLine(name);

        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
            telemetry.addLine(line);
            packet.addLine(line);
        }

        telemetry.addLine();
        packet.addLine("");
    }
}
