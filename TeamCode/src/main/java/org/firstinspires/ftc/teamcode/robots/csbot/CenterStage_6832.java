package org.firstinspires.ftc.teamcode.robots.csbot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.csbot.util.ExponentialSmoother;
import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProviders;

import java.util.LinkedHashMap;
import java.util.Map;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "0 CenterStage_6832", group = "Challenge")
@Config(value = "CS_6832")
public class CenterStage_6832 extends OpMode {

    private static final double LOW_BATTERY_VOLTAGE = 12;
    private ElapsedTime runtime = new ElapsedTime();
    public static long totalRunTime;
    //COMPONENTS
    public static Robot robot;
    static Autonomous auton;
    private FtcDashboard dashboard;
    public static Field field;
    public static DriverControls dc;
    static AutoNav autoNav;


    //GLOBAL STATES
    public static boolean active;
    //whether we have the ability to autonav or not
    public static boolean AUTONAV_ENABLED = true;
    //whether we are autonaving or not
    public static boolean autoNavOn = false;
    public static boolean autoEndgameOn = false;
    //to handle transfer b/w driver and autonav
    public static boolean autoNavInitialized = false;
    public static boolean frontAuton = true;
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
        TEST("Test"), DEMO("Demo"), MANUAL_DIAGNOSTIC("Manual Diagnostic"), SQUARE("Square"), TURN("Turn"),
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
                case 3: return GameState.DEMO;
                case 4: return GameState.MANUAL_DIAGNOSTIC;
                case 5: return GameState.RELOCALIZATION_TEST;
                default: return GameState.TEST;
            }
        }

        public String getName() {
            return name;
        }

        public static int getNumGameStates() {return 6;}

        public boolean  isAutonomous() {
            return autonomous;
        }

    }

    public static GameState gameState = GameState.AUTONOMOUS;
    static public int gameStateIndex;
    public static boolean startAuton = true;


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
        field = new Field();
        autoNav = new AutoNav(robot, field);

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

        //if driverside, always front, else only front on backstageside
        frontAuton = Constants.driverSide? true: (startingPosition.equals(Constants.Position.START_RIGHT_RED) || startingPosition.equals(Constants.Position.START_LEFT_BLUE)) ? true : false;

        int blobLocation;
        if(frontAuton) {
            blobLocation = robot.visionProviderFront.getPosition().getIndex();
        } else {
            blobLocation = robot.visionProviderBack.getMostFrequentPosition().getIndex();
        }


        if(frontAuton && blobLocation == -1) {
            blobLocation = alliance.isRed()?0:2;
        }
        auton.saveRandomizer(blobLocation);
        if(gameState.isAutonomous())
            robot.initPosition();
        robot.driveTrain.updatePoseEstimate();
        robot.enableVision();
        telemetry.addData("blobLocation", blobLocation);
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
        robot.enableVision();
        robot.visionProviderBack.setRedAlliance(startingPosition.isRed());
        robot.visionProviderFront.setRedAlliance(startingPosition.isRed());
        robot.frontVision = frontAuton;
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        lastLoopClockTime = System.nanoTime();
        totalRunTime = 0;
        //FETCH CACHE
        robot.fetchCachedCSPosition();

        field.finalizeField();
        resetGame();



        if(robot.fetched && !gameState.isAutonomous()) {
            robot.driveTrain.setPose(robot.fetchedPosition.getPose());
            robot.skyhook.skyhookLeft.setPosition(-robot.fetchedPosition.getSkyhookLeftTicks());
            robot.skyhook.skyhookRight.setPosition(-robot.fetchedPosition.getSkyhookRightTicks());
        }
        else {
            robot.driveTrain.setPose(startingPosition);
        }

        robot.updatePositionCache = true;

        if(gameState.equals(GameState.AUTONOMOUS)){
            //robot.driveTrain.localizer.resetYaw(); //todo - imu didn't want to play nice
            robot.skyhook.skyhookLeft.setPosition(0);
            robot.skyhook.skyhookRight.setPosition(0);
            //calc auton based on alliance, starting position and team prop position
            auton.pickAutonToRun(startingPosition);
        }

        if(gameState.equals(GameState.TELE_OP)){
            robot.outtake.setTargetAngle(Outtake.ELBOW_TRAVEL_ANGLE, Outtake.WRIST_TRAVEL_ANGLE, Outtake.ELEVATOR_START_ANGLE);
            robot.articulate(Robot.Articulation.TRAVEL);
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
            if (!endGameHandled && gameState == GameState.TELE_OP && (currentTime - startTime) * 1e-3 >= 80) {
                //TODO - handle endgame actions
//                robot.articulate(Robot.Articulation.START_END_GAME);
                endGameHandled = true;

            }

            update();

            switch(gameState) {
                case AUTONOMOUS:
                    if(auton.execute(dashboard)) gameState = GameState.TELE_OP;
                    break;

                case TELE_OP:
                    if(AUTONAV_ENABLED) {
                        if(autoNavOn) {
                            if(dc.joysticksInactive()){
                                //autonaving
                                if(!autoNavInitialized){
                                    autoNav.assumeControl();
                                    //assumes diagonal route
                                    autoNav.setPreferredRoute(3);
                                    autoNavInitialized = true;
                                }
                                else autoNav.run(dashboard);
                            }
                            //autonav is interrupted
                            else {
                                autoNav.relinquishControl();
                                autoNavInitialized = false;
                                autoNavOn = false;
                                autoEndgameOn = false;
                            }
                        }
                        if(autoEndgameOn) {
                            if(dc.joysticksInactive()){
                                autoNav.autoEndgame();
                            }
                            else {
                                autoNav.relinquishControl();
                                autoNavInitialized = false;
                                autoNavOn = false;
                                autoEndgameOn = false;
                            }
                        }
                    }
                    dc.joystickDrive();
                    break;
                case TEST:
                    dc.joystickDrive();
                    break;

                case DEMO:
//                    dc.joystickDriveDemoMode();
                    break;

                case MANUAL_DIAGNOSTIC:
                    dc.manualDiagnosticMethods();
                    robot.intake.articulate(Intake.Articulation.MANUAL);
                    break;

                case SQUARE:
                    auton.square.execute();
                    break;

                case TURN:
                    auton.turn.execute();
                    break;
                case RELOCALIZATION_TEST:
                    if(robot.visionOn) {
                        robot.enableVision();
                        robot.aprilTagRelocalization(2);
                    }
                    dc.joystickDrive();
//                    robot.articulate(Robot.Articulation.LINE);
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
        if(debugTelemetryEnabled) {
            if (field.finalized) {
                field.update(packet, robot);
                opModeTelemetryMap.put("Current Robot Zone", field.getZone(robot.driveTrain.pose));
                opModeTelemetryMap.put("Current Robot SubZones", field.getSubZones(robot.driveTrain.pose));
                opModeTelemetryMap.put("Current Robot POI", field.getPOI(robot.driveTrain.pose));
            }
        }

        opModeTelemetryMap.put("Battery Voltage", averageVoltage);
        opModeTelemetryMap.put("Average Loop Time", Misc.formatInvariant("%d ms (%d hz)", (int) (averageLoopTime * 1e-6), (int) (1 / (averageLoopTime * 1e-9))));
        opModeTelemetryMap.put("Last Loop Time", Misc.formatInvariant("%d ms (%d hz)", (int) (averageLoopTime * 1e-6), (int) (1 / (averageLoopTime * 1e-9))));
        opModeTelemetryMap.put("Average Robot Update Time", Misc.formatInvariant("%d ms (%d hz)", (int) (averageUpdateTime * 1e-6), (int) (1 / (averageUpdateTime * 1e-9))));
        opModeTelemetryMap.put("Last Robot Update Time", Misc.formatInvariant("%d ms (%d hz)", (int) (updateTime * 1e-6), (int) (1 / (updateTime * 1e-9))));
        opModeTelemetryMap.put("Last Centerstage Update Time", Misc.formatInvariant("%d ms (%d hz)", (int) (updateTime * 1e-6), (int) (1 / (updateTime * 1e-9))));
        //IF NECESSARY, ADD TELEMETRY FOR EACH GAME STATE
        switch(gameState) {
            case RELOCALIZATION_TEST:
                opModeTelemetryMap.put("forward?", robot.forward);
                opModeTelemetryMap.put("sensorAvg", robot.sensors.averageDistSensorValue);
                break;
            case TELE_OP:
                if(autoNavOn){
                    handleTelemetry(autoNav.getTelemetry(debugTelemetryEnabled), autoNav.getTelemetryName(), packet);
                }
                break;
            case AUTONOMOUS:
                handleTelemetry(auton.getTelemetry(debugTelemetryEnabled),  auton.getTelemetryName(), packet);
                break;
            case MANUAL_DIAGNOSTIC:
//                handleTelemetry(robot.visionProviderBack.getTelemetry(true), robot.visionProviderBack.getTelemetryName(), packet);
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
        telemetry.addData("apriltag pose x", robot.aprilTagPose.position.x);
        telemetry.addData("apriltag pose y", robot.aprilTagPose.position.y);
        telemetry.addData("robot x", robot.driveTrain.pose.position.x);
        telemetry.addData("robot y", robot.driveTrain.pose.position.y);
        if(debugTelemetryEnabled) {
            //handle robot telemetry
            handleTelemetry(robot.getTelemetry(debugTelemetryEnabled), robot.getTelemetryName(), packet);

            for (TelemetryProvider telemetryProvider : robot.subsystems)
                handleTelemetry(telemetryProvider.getTelemetry(debugTelemetryEnabled), telemetryProvider.getTelemetryName(), packet);


            packet.put("imu/roadrunner error", robot.driveTrain.imuRoadrunnerError);
            packet.put("imu angle", robot.sensors.driveIMUYaw);
            packet.put("roadrunner angle", Math.toDegrees(robot.driveTrain.pose.heading.toDouble()));
            packet.put("left", robot.sensors.leftDistSensorValue);
            packet.put("right", robot.sensors.rightDistSensorValue);


            dashboard.sendTelemetryPacket(packet);
        }
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

//        if (averageVoltage <= LOW_BATTERY_VOLTAGE) {
//            telemetryMap = new LinkedHashMap<>();
//            for (int i = 0; i < 5; i++) {
//                telemetryMap.put(i + (System.currentTimeMillis() / 500 % 2 == 0 ? "**BATTERY VOLTAGE LOW**" : "  BATTERY VOLTAGE LOW  "), (System.currentTimeMillis() / 500 % 2 == 0 ? "**CHANGE BATTERY ASAP!!**" : "  CHANGE BATTERY ASAP!!  "));
//            }
//        }
        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
            packet.addLine(line);
            telemetry.addLine(line);
        }

        telemetry.addLine();
        packet.addLine("");
    }
}


