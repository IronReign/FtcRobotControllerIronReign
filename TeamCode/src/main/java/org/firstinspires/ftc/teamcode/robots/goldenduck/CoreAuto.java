package org.firstinspires.ftc.teamcode.robots.goldenduck;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.csbot.util.ExponentialSmoother;
import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.robots.goldenduck.subsystem.Robot;

import java.util.LinkedHashMap;
import java.util.Map;

//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Core Tele", group = "Challenge")
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Core Auto", group = "Challenge")
@Config(value = "GD_Auton")
public class CoreAuto extends OpMode {

    private static final double LOW_BATTERY_VOLTAGE = 12;
    private ElapsedTime runtime = new ElapsedTime();

    //COMPONENTS
    public static Robot robot;
    static Autonomous auton;
    private FtcDashboard dashboard;

    DriverControls dc;


    //GLOBAL STATES
    public static boolean active;
    public static boolean debugTelemetryEnabled;
    private boolean initializing;
    public boolean endGameHandled;
    public static boolean initPosition = false;
    public static boolean ignoreCachePosition = false;

    //GAMESTATES
    public enum GameState {
        //MAIN OPMODES
        AUTONOMOUS("Autonomous", true), TELE_OP("Tele-Op"),

        //TEST & TEMP MODES
        TEST("Test"), DEMO("Demo"), MANUAL_DIAGNOSTIC("Manual Diagnostic"), SQUARE("Square"), TURN("Turn");

        private final String name;
        private final boolean autonomous;

        GameState(String name, boolean autonomous) {
            this.name = name;
            this.autonomous = autonomous;
        }

        GameState(String name) {
            this(name, false);
        }

        public static GameState getGameState(int gameStateIndex) {
            switch(gameStateIndex) {
                case 0: return GameState.AUTONOMOUS;
                case 1: return GameState.TELE_OP;
                case 2: return GameState.TEST;
                case 3: return GameState.DEMO;
                case 4: return GameState.MANUAL_DIAGNOSTIC;
                default: return GameState.TEST;
            }
        }

        public String getName() {
            return name;
        }

        public static int getNumGameStates() {return 5;}

        public boolean  isAutonomous() {
            return autonomous;
        }

    }

    public static GameState gameState = GameState.AUTONOMOUS;
    static public int gameStateIndex;


    //CONSTANTS FOR GAME
    public static boolean DEFAULT_DEBUG_TELEMETRY_ENABLED = false;
    public static Constants.Alliance alliance = Constants.Alliance.RED;
    public static Constants.Position startingPosition = Constants.Position.START_LEFT_RED;
    long startTime;


    //CONSTANTS
    private ExponentialSmoother loopTimeSmoother, averageUpdateTimeSmoother, voltageSmoother;
    public static double AVERAGE_LOOP_TIME_SMOOTHING_FACTOR = 0.1;
    public final double FORWARD_SMOOTHING_FACTOR = 0.3;
    public final double ROTATE_SMOOTHING_FACTOR = 0.25;
    ExponentialSmoother forwardSmoother = new ExponentialSmoother(FORWARD_SMOOTHING_FACTOR);
    ExponentialSmoother rotateSmoother = new ExponentialSmoother(ROTATE_SMOOTHING_FACTOR);


    //LIVE DATA
    private double averageLoopTime;
    long loopClockTime = System.nanoTime();
    long lastLoopClockTime;
    private double averageVoltage;

    @Override
    public void init() {
        Robot.initPositionIndex = 0;
        telemetry.addData("Status", "Hold right_trigger to enable debug mode");
        telemetry.update();

        //SET GLOBAL STATES
        active = true;
        initializing = true;
        debugTelemetryEnabled = DEFAULT_DEBUG_TELEMETRY_ENABLED;

        //INITIALIZE TIMING
        loopTimeSmoother = new ExponentialSmoother(AVERAGE_LOOP_TIME_SMOOTHING_FACTOR);
        averageUpdateTimeSmoother = new ExponentialSmoother(AVERAGE_LOOP_TIME_SMOOTHING_FACTOR);
        voltageSmoother = new ExponentialSmoother(.025);

        //INITIALIZE COMPONENTS
        robot = new Robot(hardwareMap, false);
        dc = new DriverControls(gamepad1, gamepad2);
        auton = new Autonomous(robot);

        robot.updatePositionCache = false;
        robot.driveTrain.setPose(startingPosition);

        //TELEMETRY SETUP
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry.setMsTransmissionInterval(25);
        //TODO - Differentiate between debug and non debug telemetry
        if (debugTelemetryEnabled) {

        } else {

        }
        robot.behave(Robot.Behavior.CALIBRATE);
    }
    //end init()

    public void init_loop() {

        dc.init_loop();
        dc.robotOrientedDrive();

        robot.updateVision();

        robot.visionProviderBack.setRedAlliance(startingPosition.getMod());

        if(gameState.isAutonomous()) {
            auton.updateIndexOffsets();
            //calc auton based on alliance, starting position and team prop position
            auton.pickAutonToRun(alliance);
        }

        robot.driveTrain.updatePoseEstimate();

        telemetry.addData("visionProviderIndex", Robot.visionProviderIndex);
        telemetry.addData("visionProviderOnRed", robot.visionProviderBack.isRedAlliance);
        telemetry.addData("blobLocation", robot.visionProviderBack.getMostFrequentPosition().getIndex());
        telemetry.addData("fetched", robot.fetched);
        telemetry.addData("gameState", gameState);
        telemetry.addData("active", active);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("startingPosition", startingPosition);
        telemetry.addData("initPositionIndex", Robot.initPositionIndex);

        update();
    }
    //end init_loop()


    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        lastLoopClockTime = System.nanoTime();

        //FETCH CACHE
        robot.fetchCachedCSPosition();

        resetGame();
        robot.visionProviderBack.shutdownVision();
        if(gameState.equals(GameState.AUTONOMOUS)){
        }

        if(gameState.equals(GameState.TELE_OP)){

        }

        if(gameState.equals(GameState.TEST) ||  gameState.equals(GameState.DEMO)){

        }

        if(robot.fetched && !gameState.isAutonomous()) {
            robot.driveTrain.setPose(robot.fetchedPosition.getPose());
        }
        else {
            robot.driveTrain.setPose(startingPosition);
        }

        robot.updatePositionCache = true;

        robot.start();
    }
    //end start()
    public void resetGame()
    {
        robot.resetRobotPosFromCache(5, ignoreCachePosition);
    }


    @Override
    public void loop() {
        dc.updateStickyGamepads();
        dc.handleStateSwitch();

        if (active) {
            long currentTime = System.currentTimeMillis();
            if (!endGameHandled && gameState == CoreAuto.GameState.TELE_OP && (currentTime - startTime) * 1e-3 >= 80) {
                //TODO - handle endgame actions
//                robot.articulate(Robot.Articulation.START_END_GAME);
                endGameHandled = true;

            }

            switch(gameState) {
                case AUTONOMOUS:
                    if(auton.execute(dashboard )) gameState = GameState.TELE_OP;
                    //auton.execute(dashboard);
                    break;

                case TELE_OP:
                    dc.joystickDrive();
                    //implement teleop
                    break;

                case TEST:
                    dc.joystickDrive();
                    break;

                case DEMO:
//                    dc.joystickDriveDemoMode();
                    break;

                case MANUAL_DIAGNOSTIC:
                    dc.manualDiagnosticMethods();
                    break;

            }
        }
        else {
            dc.handlePregameControls();
        }
        update();
    }
    //end loop()

    @Override
    public void stop(){
        robot.stop();
    }

    private void update() {
        // handling dashboard changes
        forwardSmoother.setSmoothingFactor(FORWARD_SMOOTHING_FACTOR);
        rotateSmoother.setSmoothingFactor(ROTATE_SMOOTHING_FACTOR);


        TelemetryPacket packet = new TelemetryPacket();

        long updateStartTime = System.nanoTime();
        robot.update(packet.fieldOverlay());
        long updateTime = (System.nanoTime() - updateStartTime);
        double averageUpdateTime = averageUpdateTimeSmoother.update(updateTime);

        Map<String, Object> opModeTelemetryMap = new LinkedHashMap<>();
        // handling op mode telemetry
        opModeTelemetryMap.put("Active", active);
        if(initializing) {
            opModeTelemetryMap.put("Starting Position", startingPosition);
        }
        opModeTelemetryMap.put("Battery Voltage", averageVoltage);
        opModeTelemetryMap.put("Average Loop Time", Misc.formatInvariant("%d ms (%d hz)", (int) (averageLoopTime * 1e-6), (int) (1 / (averageLoopTime * 1e-9))));
        opModeTelemetryMap.put("Last Loop Time", Misc.formatInvariant("%d ms (%d hz)", (int) (averageLoopTime * 1e-6), (int) (1 / (averageLoopTime * 1e-9))));
        opModeTelemetryMap.put("Average Robot Update Time", Misc.formatInvariant("%d ms (%d hz)", (int) (averageUpdateTime * 1e-6), (int) (1 / (averageUpdateTime * 1e-9))));
        opModeTelemetryMap.put("Last Robot Update Time", Misc.formatInvariant("%d ms (%d hz)", (int) (updateTime * 1e-6), (int) (1 / (updateTime * 1e-9))));

        //IF NECESSARY, ADD TELEMETRY FOR EACH GAME STATE
        switch(gameState) {
            case TELE_OP:
                break;
            case AUTONOMOUS:
                handleTelemetry(auton.getTelemetry(debugTelemetryEnabled),  auton.getTelemetryName(), packet);
                break;
        }

        //handle this class' telemetry
        handleTelemetry(opModeTelemetryMap,  gameState.getName(), packet);

        //handle robot telemetry
        handleTelemetry(robot.getTelemetry(debugTelemetryEnabled), robot.getTelemetryName(), packet);

        for(TelemetryProvider telemetryProvider: robot.subsystems)
            handleTelemetry(telemetryProvider.getTelemetry(debugTelemetryEnabled), telemetryProvider.getTelemetryName(), packet);

        Map<String, Object> visionTelemetryMap = robot.visionProviderBack.getTelemetry(debugTelemetryEnabled);
        visionTelemetryMap.put("Backend",
                Misc.formatInvariant("%s (%s)",
                        VisionProviders.VISION_PROVIDERS[Robot.visionProviderIndex].getSimpleName(),
                        robot.visionProviderFinalized ?
                                "finalized" :
                                System.currentTimeMillis() / 500 % 2 == 0 ? "**NOT FINALIZED**" : "  NOT FINALIZED  "
                )
        );

        handleTelemetry(visionTelemetryMap, robot.visionProviderBack.getTelemetryName(), packet);

        packet.put("imu/roadrunner error", robot.driveTrain.imuRoadrunnerError);
        packet.put("imu angle", robot.driveTrain.imuAngle);
        packet.put("roadrunner angle", Math.toDegrees(robot.driveTrain.pose.heading.toDouble()));
        packet.put("imu PID error", robot.driveTrain.PIDError);
        packet.put("imu PID Correction", robot.driveTrain.PIDCorrection);

        dashboard.sendTelemetryPacket(packet);
        telemetry.update();

        updateLiveStates();

    }

    //HELPER METHODS
    private void updateLiveStates() {
        loopClockTime = System.nanoTime();
        long loopTime = loopClockTime - lastLoopClockTime;
        averageLoopTime = loopTimeSmoother.update(loopTime);
        averageVoltage = voltageSmoother.update(robot.getVoltage());
        lastLoopClockTime = loopClockTime;
    }
    private void handleTelemetry(Map<String, Object> telemetryMap, String telemetryName, TelemetryPacket packet) {
        telemetry.addLine(telemetryName);
        packet.addLine(telemetryName);

        if (averageVoltage <= LOW_BATTERY_VOLTAGE) {
            telemetryMap = new LinkedHashMap<>();
            for (int i = 0; i < 5; i++) {
                telemetryMap.put(i + (System.currentTimeMillis() / 500 % 2 == 0 ? "**BATTERY VOLTAGE LOW**" : "  BATTERY VOLTAGE LOW  "), (System.currentTimeMillis() / 500 % 2 == 0 ? "**CHANGE BATTERY ASAP!!**" : "  CHANGE BATTERY ASAP!!  "));
            }
        }
        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
            packet.addLine(line);
            telemetry.addLine(line);
        }

        telemetry.addLine();
        packet.addLine("");
    }
}


