package org.firstinspires.ftc.teamcode.robots.deepthought;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Constants;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.ExponentialSmoother;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.deepthought.vision.VisionProviders;

import java.util.LinkedHashMap;
import java.util.Map;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "0 IntoTheDeep_6832", group = "Challenge")
@Config(value = "0 - ITD_6832")
public class IntoTheDeep_6832 extends OpMode {

    public static long totalRunTime;
    //COMPONENTS
    public static Robot robot;
    static Autonomous auton;
    private FtcDashboard dashboard;
    public static Field field;
    public static DriverControls dc;
//    static AutoNav autoNav;


    //GLOBAL STATES
    public static boolean active;
    public static boolean debugTelemetryEnabled;
    private boolean initializing;
    public static boolean initPosition = false;
    public static boolean ignoreCachePosition = false;

    //GAMESTATES
    public enum GameState {
        //MAIN OPMODES
        AUTONOMOUS("Autonomous", true), TELE_OP("Tele-Op"),

        //TEST & TEMP MODES
        TEST("Test"),
        DEMO("Demo"),
        MANUAL_DIAGNOSTIC("Manual Diagnostic"), SQUARE("Square"), TURN("Turn"),
        RELOCALIZATION_TEST("Relocalize");
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
//                case 3: return GameState.DEMO;
//                case 4: return GameState.MANUAL_DIAGNOSTIC;
                case 3: return GameState.RELOCALIZATION_TEST;
                default: return GameState.TEST;
            }
        }

        public String getName() {
            return name;
        }

        public static int getNumGameStates() {return 4;}

        public boolean isAutonomous() {
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
        field = new Field();
//        autoNav = new AutoNav(robot, field);

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

    }
    //end init()

    public void init_loop() {

        dc.init_loop();
        dc.robotOrientedDrive();
        initVision();

        robot.driveTrain.updatePoseEstimate();
        telemetry.addData("fetched", robot.fetched);
        telemetry.addData("gameState", gameState);
        telemetry.addData("active", active);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("startingPosition", startingPosition);
        telemetry.addData("initPositionIndex", Robot.initPositionIndex);

        update();
    }
    //end init_loop()

    private void initVision() {
        //TODO - HANDLE INIT VISION
        robot.enableVision();
        robot.visionProviderBack.setRedAlliance(startingPosition.isRed());
        robot.visionProviderFront.setRedAlliance(startingPosition.isRed());
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        lastLoopClockTime = System.nanoTime();
        totalRunTime = 0;
        //FETCH CACHE
        robot.fetchCachedDTPosition();

        field.finalizeField();
        resetGame();



        if(robot.fetched && !gameState.isAutonomous()) {
            robot.driveTrain.setPose(robot.fetchedPosition.getPose());

        }
        else {
            robot.driveTrain.setPose(startingPosition);
        }

        robot.updatePositionCache = true;

        if(gameState.equals(GameState.AUTONOMOUS)){
            robot.driveTrain.imu.resetYaw();
        }

        if(gameState.equals(GameState.TELE_OP)){

        }

        if(gameState.equals(GameState.TEST) ||  gameState.equals(GameState.DEMO)){

        }

        robot.start();
    }
    //end start()
    public void resetGame()
    {
        robot.resetRobotPosFromCache(5, ignoreCachePosition);
    }


    @Override
    public void loop() {
        totalRunTime = (System.currentTimeMillis()-startTime)/1000;
        dc.updateStickyGamepads();
        dc.handleStateSwitch();

        if (active) {
            long currentTime = System.currentTimeMillis();

            update();

            switch(gameState) {
                case AUTONOMOUS:
                    if(auton.execute(dashboard)) gameState = GameState.TELE_OP;
                    break;

                case TELE_OP:
                    dc.joystickDrive();
                    break;
                case TEST:
                    dc.manualDiagnosticMethods();
                    break;
                case MANUAL_DIAGNOSTIC:
                    break;
                case RELOCALIZATION_TEST:
                    dc.joystickDrive();
                    break;
            }
        }
        else {
            dc.handlePregameControls();
        }
    }
    //end loop()

    @Override
    public void stop(){
        gameState = GameState.TELE_OP;
        robot.stop();
    }

    private void update() {
        long updateStartTime = System.nanoTime();
        long updateTime = (System.nanoTime() - updateStartTime);
        double averageUpdateTime = averageUpdateTimeSmoother.update(updateTime);

        TelemetryPacket packet = new TelemetryPacket();
        robot.update(packet.fieldOverlay());
        Map<String, Object> opModeTelemetryMap = new LinkedHashMap<>();
        // handling op mode telemetry
        opModeTelemetryMap.put("Active", active);
        if(initializing) {
            opModeTelemetryMap.put("Starting Position", startingPosition);
        }
        if(debugTelemetryEnabled) {
            opModeTelemetryMap.put("Average Robot Update Time", Misc.formatInvariant("%d ms (%d hz)", (int) (averageUpdateTime * 1e-6), (int) (1 / (averageUpdateTime * 1e-9))));
        }

        opModeTelemetryMap.put("Battery Voltage", averageVoltage);
        opModeTelemetryMap.put("Average Loop Time", Misc.formatInvariant("%d ms (%d hz)", (int) (averageLoopTime * 1e-6), (int) (1 / (averageLoopTime * 1e-9))));


        //IF NECESSARY, ADD TELEMETRY FOR EACH GAME STATE
        switch(gameState) {
            case RELOCALIZATION_TEST:

                break;
            case TELE_OP:

                break;
            case AUTONOMOUS:
                handleTelemetry(auton.getTelemetry(debugTelemetryEnabled),  auton.getTelemetryName(), packet);
                break;
            case TEST:

                break;
        }
        //handle this class' telemetry
        handleTelemetry(opModeTelemetryMap,  gameState.getName(), packet);

        Map<String, Object> visionTelemetryMap = robot.visionProviderBack.getTelemetry(debugTelemetryEnabled);
        visionTelemetryMap.put("Backend",
                Misc.formatInvariant("%s (%s)",
                        VisionProviders.VISION_PROVIDERS[Robot.backVisionProviderIndex].getSimpleName(),
                        robot.visionProviderFinalized ?
                                "finalized" :
                                System.currentTimeMillis() / 500 % 2 == 0 ? "**NOT FINALIZED**" : "  NOT FINALIZED  "
                )
        );

        handleTelemetry(visionTelemetryMap, "BACK VISION - \t" + robot.visionProviderBack.getTelemetryName(), packet);
        handleTelemetry(visionTelemetryMap, "FRONT VISION - \t" + robot.visionProviderFront.getTelemetryName(), packet);

        handleTelemetry(robot.getTelemetry(debugTelemetryEnabled), robot.getTelemetryName(), packet);

        if(debugTelemetryEnabled) {
            for (TelemetryProvider telemetryProvider : robot.subsystems)
                handleTelemetry(telemetryProvider.getTelemetry(debugTelemetryEnabled), telemetryProvider.getTelemetryName(), packet);
        }
        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
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

        if (averageVoltage <= 12) {
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


