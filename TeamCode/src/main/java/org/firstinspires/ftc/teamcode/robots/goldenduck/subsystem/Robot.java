package org.firstinspires.ftc.teamcode.robots.goldenduck.subsystem;

import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.alliance;
import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.gameState;
import static org.firstinspires.ftc.teamcode.robots.csbot.DriverControls.fieldOrientedDrive;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832;
import org.firstinspires.ftc.teamcode.robots.csbot.util.CSPosition;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.csbot.util.PositionCache;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.Target;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProviders;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;


@Config(value = "GD_Robot")
public class Robot implements Subsystem {

    //components and subsystems
    public Subsystem[] subsystems;
    public DriveTrain driveTrain;
    public VisionProvider visionProviderBack = null;
    public static boolean visionOn = true;
    public Arm arm;
    public Skyhook skyhooks;
    //TODO - create a field
//    public Field field;

    public static boolean updatePositionCache = false;
    public PositionCache positionCache;
    public CSPosition currPosition;

    public CSPosition fetchedPosition;

    //vision variables
    public boolean visionProviderFinalized = false;
    public static int visionProviderIndex = 2;

    private long[] subsystemUpdateTimes;
    private final List<LynxModule> hubs;
    public HardwareMap hardwareMap;
    private VoltageSensor batteryVoltageSensor;
    private Behavior curBehavior, prevBehavior;
    public List<Target> targets = new ArrayList<Target>();
    public boolean fetched;
    public boolean selfDriving = true;

    public enum Behavior {
        //beater bar, drivetrain, drone launcher, outtake
        MANUAL,
        AUTON,
        CALIBRATE,
        BACKDROP_PREP,
        BACKDROP,
        FOLD,
        INGEST,
        HANG,
        PREP_FOR_HANG,
        LAUNCH_DRONE,
        TRAVEL_FROM_BACKDROP,
        TRAVEL_FROM_INGEST,
        TRAVEL

    }

    public void start() {
        //TODO - articulate starting position
        curBehavior = Robot.Behavior.MANUAL;
    }
    //end start


    public Robot(HardwareMap hardwareMap, boolean simulated) {
        this.hardwareMap = hardwareMap;
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : hubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        //initialize vision
        createVisionProvider();

        positionCache = new PositionCache(5);

        // initializing subsystems
        driveTrain = new DriveTrain(hardwareMap, this, simulated);
        arm = new Arm(hardwareMap, this);
        skyhooks = new Skyhook(hardwareMap,this);

        subsystems = new Subsystem[]{driveTrain, arm, skyhooks};
        subsystemUpdateTimes = new long[subsystems.length];

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        curBehavior = Robot.Behavior.MANUAL;

//        field = new Field(true);
    }
    //end constructor

    public double deltaTime = 0;
    long lastTime = 0;

    @Override
    public void update(Canvas fieldOverlay) {
        deltaTime = (System.nanoTime() - lastTime) / 1e9;
        lastTime = System.nanoTime();
        clearBulkCaches(); //ALWAYS FIRST LINE IN UPDATE

        //cache current position of robot
        if (updatePositionCache) {
            currPosition = new CSPosition(driveTrain.pose,0,0);
            positionCache.update(currPosition, false);
        }

        behave(curBehavior);
        //TODO - DELETE
        driveTrain.updatePoseEstimate();

        drawRobot(fieldOverlay, driveTrain.pose);

        //update subsystems
        for (int i = 0; i < subsystems.length; i++) {
            Subsystem subsystem = subsystems[i];
            long updateStartTime = System.nanoTime();
            subsystem.update(fieldOverlay);
            subsystemUpdateTimes[i] = System.nanoTime() - updateStartTime;
        }
    }
    //end update

    public void updateVision() {
        if (visionOn) {
            if (!visionProviderFinalized) {
                createVisionProvider();
                visionProviderBack.initializeVision(hardwareMap);
                visionProviderFinalized = true;

            }
            visionProviderBack.update();
        }
    }

    private static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }

    public void switchVisionProviders() {
        visionProviderBack.shutdownVision();
        if (visionProviderIndex == 2) {
            //switch to AprilTags
            visionProviderIndex = 0;
            visionProviderFinalized = false;

        } else if (visionProviderIndex == 0) {
            //switch back to ColorBlob
            visionProviderIndex = 2;
            visionProviderFinalized = false;
        }
    }

    public void fetchCachedCSPosition() {
        fetchedPosition = positionCache.readPose();
        fetched = fetchedPosition != null;
    }

    public void resetRobotPosFromCache(double loggerTimeoutMinutes, boolean ignoreCache) {
        if(!ignoreCache) {
            fetchCachedCSPosition();
            if (gameState.equals(CenterStage_6832.GameState.TELE_OP) || gameState.equals((CenterStage_6832.GameState.TEST))) {
                int loggerTimeout = (int) (loggerTimeoutMinutes * 60000);
                if (!(System.currentTimeMillis() - fetchedPosition.getTimestamp() > loggerTimeout || ignoreCache)) {
                    //apply cached position
//                    driveTrain.pose = fetchedPosition.getPose();
                }
            }
        }
    }

    public static int initPositionIndex = 0;
    public long initPositionTimer;

    public void initPosition() {
        switch (initPositionIndex) {
            case 0:
                initPositionTimer = futureTime(1);
                initPositionIndex++;
                break;
            case 1:
                    initPositionIndex ++;
                break;
            case 2:

                break;
            case 3:

                break;
            case 4:
                break;
            case 5:
                break;
            case 6:
                break;
        }
    }

    public Behavior behave(Behavior target) {
        curBehavior = target;
        switch (this.curBehavior) {
            case MANUAL:
                break;
            case TRAVEL:
                break;
            case BACKDROP:
                break;
            case CALIBRATE:
                arm.behave(Arm.Behavior.CALIBRATE);
                curBehavior=Robot.Behavior.MANUAL;
                break;
            case INGEST:
                if (Ingest()) {
                    curBehavior = Robot.Behavior.TRAVEL_FROM_INGEST;
                }
                break;
            case HANG:
                //deploy the hooks
                break;
            case PREP_FOR_HANG:
                arm.behave(Arm.Behavior.INGEST_FROM_TRAVEL);
                curBehavior = Robot.Behavior.TRAVEL;
                break;
            case LAUNCH_DRONE:
                //trigger the drone launcher
                break;
            case TRAVEL_FROM_INGEST:
                //get arm into a position to safely travel through rigging
                if (!(arm.getBehavior() == Arm.Behavior.TRAVEL)) {
                    arm.behave(Arm.Behavior.TRAVEL_FROM_INGEST);
                }
                break;
            case TRAVEL_FROM_BACKDROP:
                //assume intake is already in travel
                arm.behave(Arm.Behavior.TRAVEL_FROM_BACKDROP);
                curBehavior = Robot.Behavior.TRAVEL;
                break;
            case BACKDROP_PREP:
                arm.behave(Arm.Behavior.BACKDROP_PREP);
                curBehavior = Robot.Behavior.BACKDROP;
                break;
        }
        return curBehavior;
    }

    public void toggleBackdropPrep(){
        if(curBehavior.equals(Robot.Behavior.BACKDROP)){
            curBehavior = Robot.Behavior.TRAVEL_FROM_BACKDROP;
        }
        else{
            curBehavior = Robot.Behavior.BACKDROP_PREP;
        }
    }

    @Override
    public void stop() {
        //force update the cached position
        currPosition = new CSPosition(driveTrain.pose, 0, 0);
        positionCache.update(currPosition, true);
        for (Subsystem component : subsystems) {
            component.stop();
        }
    }
    //end stop


    public static int ingestStage = 0;
    public long ingestTimer = 0;

    //todo this a template behavior that should be used to automate picking up pixels
    //todo adapt for GD
    public boolean Ingest() {
        switch (ingestStage) {
            case 0:
                arm.behave(Arm.Behavior.INGEST_FROM_TRAVEL);
                ingestStage++;
            case 1: //wait for outake to dock before proceeding
                if (arm.getBehavior() == Arm.Behavior.MANUAL) {
                    //intake can start eating

                }
                break;
            case 2:
                ingestStage++;
                break;
            case 3:
                ingestStage=0;
                return true;

        }
        return false;
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("Behavior", curBehavior);
        telemetryMap.put("fieldOrientedDrive?", fieldOrientedDrive);
        telemetryMap.put("Ingest Stage", ingestStage);
        telemetryMap.put("initPositionIndex", initPositionIndex);
//        telemetryMap.put("MemoryPose", positionCache.readPose());
        for (int i = 0; i < subsystems.length; i++) {
            String name = subsystems[i].getClass().getSimpleName();
            telemetryMap.put(name + " Update Time", Misc.formatInvariant("%d ms (%d hz)", (int) (subsystemUpdateTimes[i] * 1e-6), (int) (1 / (subsystemUpdateTimes[i] * 1e-9))));
        }


        telemetryMap.put("Delta Time", deltaTime);


        return telemetryMap;
    }
    //end getTelemetry

    public void createVisionProvider() {
        try {
            visionProviderBack = VisionProviders.VISION_PROVIDERS[visionProviderIndex].newInstance().setRedAlliance(alliance==Constants.Alliance.RED);
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException("Error while instantiating vision provider");
        }
    }

    public void clearBulkCaches() {
        for (LynxModule module : hubs)
            module.clearBulkCache();
    }

    public double getVoltage() {
        return batteryVoltageSensor.getVoltage();
    }

    @Override
    public String getTelemetryName() {
        return "ROBOT";
    }
}
